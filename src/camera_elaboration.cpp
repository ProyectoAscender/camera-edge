#include "camera_elaboration.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "tkDNN/utils.h"

#include "Profiler.h"

#include "undistort.h"

//Includes for udp sockets
#include "zmq.hpp"
#include <sys/stat.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>



void sendUDPMessage(int sockfd,int n_frame, char* data, unsigned int *size ){


    char clientname[1024] = "";
    struct sockaddr_in clientaddr = sockaddr_in();
    socklen_t len = sizeof(clientaddr);

    inet_ntop(AF_INET,&clientaddr.sin_addr,clientname,sizeof(clientname));
    std::string client_ip_str(clientname);


    char buffer_recv[1024];
    int recv_error = recvfrom(sockfd, buffer_recv, 1024,
        MSG_DONTWAIT, ( struct sockaddr *) &clientaddr,
        &len);

    inet_ntop(AF_INET,&clientaddr.sin_addr,clientname,sizeof(clientname));
    client_ip_str = std::string(clientname);


    if (client_ip_str != "0.0.0.0"){
        sendto(sockfd, (const char *)data, *size,MSG_DONTWAIT, (const struct sockaddr *) &clientaddr,sizeof(clientaddr));
    }
    free(data); 
}

void collectBoxInfo( std::vector<std::vector<tk::dnn::box>>& batchDetected,
                std::vector<tk::dnn::box>& box_vector,std::vector<std::tuple<double, double>>& coords,
                std::vector<std::tuple<double, double>>& coordsGeo, 
                std::vector<std::tuple<double, double, double, double, double, double, double, double>>& boxCoords,
                float& scale_x, float& scale_y, edge::camera& camera)
{
    double lat_ur, lat_lr, lat_ll, lat_ul, lon_ur, lon_lr, lon_ll, lon_ul, debugLat, debugLong, debugLat2, debugLong2,debugAlt;
    double north, east;

    for (auto &box_batch : batchDetected) { // iterating per frame grouping
            for (auto &box : box_batch) { // iterating for boxes in frame
                convertCameraPixelsToMapMeters((box.x + box.w / 2)*scale_x, (box.y + box.h)*scale_y, box.cl, camera, east, north);
                convertCameraPixelsToGeodetic( (box.x + box.w / 2)*scale_x, (box.y + box.h)*scale_y, box.cl, camera, debugLat, debugLong);
                camera.geoConv.enu2Geodetic(east, north, 0, &debugLat2, &debugLong2, &debugAlt);
                //convertCameraPixelsToMapMeters(box.x + box.w/2, box.y + box.h/2, box.cl, camera, north,
                //                               east); // box center
                // convertCameraPixelsToGeodetic(box.x + box.w/2, box.y + box.h/2, box.cl, camera, lat,
                //                              lon); // box center
                convertCameraPixelsToGeodetic((box.x + box.w) * scale_x, box.y * scale_y, box.cl, camera, lat_ur,
                                              lon_ur); // box upper right corner
                convertCameraPixelsToGeodetic((box.x + box.w) * scale_x, (box.y + box.h) * scale_y, box.cl, camera, lat_lr,
                                              lon_lr); // box lower right corner
                convertCameraPixelsToGeodetic(box.x * scale_x, (box.y + box.h) * scale_y, box.cl, camera, lat_ll,
                                              lon_ll); // box lower left corner
                convertCameraPixelsToGeodetic(box.x * scale_x, box.y * scale_y, box.cl, camera, lat_ul,
                                              lon_ul); // box upper left corner
                box_vector.push_back(box);
                coords.push_back(std::make_tuple(north, east));
                // coordsGeo.push_back(std::make_tuple(lat, lon));
                boxCoords.push_back(std::make_tuple(lat_ur, lon_ur, lat_lr, lon_lr, lat_ll, lon_ll, lat_ul, lon_ul));
            }
        }

}


int openUDPsockets(std::string& socketPort){


    zmq::message_t unimportant_message;
    zmq::context_t context(1);

    struct sockaddr_in servaddr;
    int sockfd;
    
    std::cout << "Listening to tcp://0.0.0.0:" << socketPort << " waiting for COMPSs ack to start" << std::endl;
    zmq::socket_t *app_socket = new zmq::socket_t(context, ZMQ_REP);
    app_socket->bind("tcp://0.0.0.0:" + socketPort);
    app_socket->recv(&unimportant_message); // wait for python workflow to ack to start processing frames
    app_socket->close();
    delete app_socket;

    std::cout << "Listening to udp in://0.0.0.0:" << socketPort << std::endl;

    // Creating socket file descriptor
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(atoi(socketPort.c_str())); //socketPort
    servaddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
        std::cout << "Failed to bind socket! " << strerror(errno) << "\n";
        return 1;
    }

    std::cout << socketPort << std::endl;
    //connect(sockfd, (struct sockaddr *) &servaddr, sizeof(servaddr));
        

    return sockfd;
}

void pixel2GPS(const int x, const int y, double &lat, double &lon, double* adfGeoTransform)
{
    //conversion from pixels to GPS, via georeferenced map parameters
    double xoff, a, b, yoff, d, e;
    xoff    = adfGeoTransform[0];
    a       = adfGeoTransform[1];
    b       = adfGeoTransform[2];
    yoff    = adfGeoTransform[3];
    d       = adfGeoTransform[4];
    e       = adfGeoTransform[5];

    lon     = a * x + b * y + xoff;
    lat     = d * x + e * y + yoff;
}
void GPS2pixel(double lat, double lon, int &x, int &y, double* adfGeoTransform)
{
    //conversion from GPS to pixels, via georeferenced map parameters
    x = int(round( (lon - adfGeoTransform[0]) / adfGeoTransform[1]) );
    y = int(round( (lat - adfGeoTransform[3]) / adfGeoTransform[5]) );
}

void convertCameraPixelsToGeodetic(const int x, const int y, const int cl, edge::camera& cam, double& lat, double& lon)
{
    double up;

    //transform camera pixel into georeferenced map pixel
    std::vector<cv::Point2f> x_y, ll;
    x_y.push_back(cv::Point2f(x, y));
    cv::perspectiveTransform(x_y, ll, cam.prjMat);

    //transform to map pixel into GPS
    pixel2GPS(ll[0].x, ll[0].y, lat, lon, cam.adfGeoTransform);
} 

void convertCameraPixelsToMapMeters(const int x, const int y, const int cl, edge::camera& cam, double& east, double& north)
{
    double latitude, longitude;
    double up;
    
    //transform camera pixel into georeferenced map pixel
    std::vector<cv::Point2f> x_y, ll;
    x_y.push_back(cv::Point2f(x, y));
    cv::perspectiveTransform(x_y, ll, cam.prjMat);
    int precision = std::numeric_limits<double>::max_digits10;

    //tranform to map pixel into GPS
    //pixel2GPS(ll[0].x, ll[0].y, latitude, longitude, cam.adfGeoTransform);

    //conversion from GPS to meters 
    cam.geoConv.geodetic2Enu(ll[0].y, ll[0].x, 0, &east, &north, &up);    
}

std::vector<edge::tracker_line> getTrackingLines(const tracking::Tracking& t, edge::camera& cam, const float scale_x, const float scale_y, bool verbose){
    std::vector<edge::tracker_line>  lines;
    std::vector<cv::Point2f> map_pixels;
    std::vector<cv::Point2f> camera_pixels;

    double north, east, up;
    double latitude, longitude, altitude;
    int map_pix_x, map_pix_y; 

    for(auto tr: t.trackers){
        if(tr.predList.size()){
            edge::tracker_line line;

            map_pixels.clear();
            camera_pixels.clear();
            for(int i=0; i < tr.predList.size(); ++i){
                //convert from meters to GPS
                cam.geoConv.enu2Geodetic(tr.predList[i].x, tr.predList[i].y, 0, &latitude, &longitude, &altitude);
                //convert from GPS to map pixels
                GPS2pixel(latitude, longitude, map_pix_x, map_pix_y, cam.adfGeoTransform);
                map_pixels.push_back(cv::Point2f(map_pix_x, map_pix_y));
            }

            //transform map pixels to camers pixels
            cv::perspectiveTransform(map_pixels, camera_pixels, cam.invPrjMat);
            
            //convert into viewer coordinates
            for(auto cp: camera_pixels)
            {
                if(verbose)
                    std::cout<<"x:\t"<<cp.x<<"\t y:\t"<<cp.y<<std::endl;
                line.points.push_back(viewer->convertPosition(cp.x*scale_x, cp.y*scale_y, -0.004, cam.id));
            }
            line.color = tk::gui::Color_t {tr.r, tr.g, tr.b, 255};
            lines.push_back(line);
        }
    }
    return lines;
}

void prepareMessage(const tracking::Tracking& t, MasaMessage& message, tk::common::GeodeticConverter& geoConv, 
                    const int cam_id, edge::Dataset_t dataset, uint64_t t_stamp_acquisition_ms)
{
    message.objects.clear();
    double latitude, longitude, altitude;
    std::vector<int> cam_id_vector {cam_id};
    std::vector<int> obj_id_vector;
    int i = 0;
    for(auto tr: t.trackers){
        if(tr.predList.size()){
            //convert from meters to GPS
            i = tr.predList.size() -1;
            geoConv.enu2Geodetic(tr.predList[i].x, tr.predList[i].y, 0, &latitude, &longitude, &altitude);
            //add RoadUser to the message
            if(checkClass(tr.cl, dataset)){
                
                obj_id_vector.push_back(tr.id);
                RoadUser tmp = getRoadUser(cam_id_vector, latitude, longitude, obj_id_vector, tr.predList[i].vel, tr.predList[i].yaw, tr.traj.back().error, tr.cl, dataset);
                message.objects.push_back(tmp);
                obj_id_vector.clear();
            }
                
        }
    }
    message.cam_idx = cam_id;
    message.t_stamp_ms = t_stamp_acquisition_ms;
    message.num_objects = message.objects.size();
}

char* prepareMessageUDP(std::vector<tk::dnn::box> &box_vector, std::vector<std::tuple<double, double>> &coords,
                     // std::vector<std::tuple<double, double>> &coordsGeo,
                     std::vector<std::tuple<double, double, double, double, double, double, double, double>> &boxCoords,
                     unsigned int n_frame, int cam_id, double lat_init, double lon_init, unsigned int *size, float scale_x, float scale_y) {
    /*box_vector.erase(std::remove_if(box_vector.begin(), box_vector.end(), [](tk::dnn::box &box) {
        return box.cl == 7 || box.cl == 8;
    }), box_vector.end()); // if traffic signs or traffic lights*/
    for (int i = box_vector.size() - 1; i >= 0; i--) {
        // if traffic signs or traffic lights

        /*std::cout << "In removing boxes: pixel x: " << box_vector[i].x << " pixel y: " << box_vector[i].y <<
            " north: " << std::get<0>(coords[i]) << " east: " << std::get<1>(coords[i]) <<
            " lat: " << std::get<0>(coordsGeo[i]) << " lon: " << std::get<1>(coordsGeo[i]) << std::endl;*/
        if (box_vector[i].cl == 7 || box_vector[i].cl == 8 || box_vector[i].cl == 6) {

            box_vector.erase(box_vector.begin()+i);
            coords.erase(coords.begin()+i);
            //coordsGeo.erase(coordsGeo.begin()+i);
            boxCoords.erase(boxCoords.begin()+i);
        }
    }
    *size = box_vector.size() * (sizeof(double) * 10 + sizeof(int) + 1 + sizeof(float) * 4) + 1 + sizeof(int)
            + sizeof(unsigned long long) + sizeof(double) * 2;
    char *data = (char *) malloc(*size);
    char *data_origin = data;
    char flag = ~0;
    memcpy(data++, &flag, 1);
    memcpy(data, &cam_id, sizeof(int));
    data += sizeof(int);
    unsigned long long timestamp = getTimeMs();
    memcpy(data, &timestamp, sizeof(unsigned long long));
    data += sizeof(unsigned long long);
    memcpy(data, &lat_init, sizeof(double));
    data += sizeof(double);
    memcpy(data, &lon_init, sizeof(double));
    data += sizeof(double);
    float box_x, box_y, box_w, box_h;
    for (int i = 0; i < box_vector.size(); i++) {
        tk::dnn::box box = box_vector[i];
        std::tuple<double, double> coord = coords[i];
        double north = std::get<0>(coord);
        double east = std::get<1>(coord);
        /*std::tuple<double, double> coordGeo = coordsGeo[i];
        double lat = std::get<0>(coordGeo);
        double lon = std::get<1>(coordGeo);*/
        memcpy(data, &north, sizeof(double));
        data += sizeof(double);
        memcpy(data, &east, sizeof(double));
        data += sizeof(double);
        /*memcpy(data, &lat, sizeof(double));
        data += sizeof(double);
        memcpy(data, &lon, sizeof(double));
        data += sizeof(double);*/
        memcpy(data, &n_frame, sizeof(unsigned int));
        data += sizeof(unsigned int);
        memcpy(data, &box.cl, sizeof(char));
        data += sizeof(char);
	box_x = box.x * scale_x;
        memcpy(data, &box_x, sizeof(float));
        data += sizeof(float);
	box_y = box.y * scale_y;
        memcpy(data, &box_y, sizeof(float));
        data += sizeof(float);
	box_w = box.w * scale_x;
        memcpy(data, &box_w, sizeof(float));
        data += sizeof(float);
	box_h = box.h * scale_y;
        memcpy(data, &box_h, sizeof(float));
        data += sizeof(float);
        std::tuple<double, double, double, double, double, double, double, double> boxCoord = boxCoords[i];
        double lat_ur = std::get<0>(boxCoord);
        double lon_ur = std::get<1>(boxCoord);
        double lat_lr = std::get<2>(boxCoord);
        double lon_lr = std::get<3>(boxCoord);
        double lat_ll = std::get<4>(boxCoord);
        double lon_ll = std::get<5>(boxCoord);
        double lat_ul = std::get<6>(boxCoord);
        double lon_ul = std::get<7>(boxCoord);
        memcpy(data, &lat_ur, sizeof(double));
        data += sizeof(double);
        memcpy(data, &lon_ur, sizeof(double));
        data += sizeof(double);
        memcpy(data, &lat_lr, sizeof(double));
        data += sizeof(double);
        memcpy(data, &lon_lr, sizeof(double));
        data += sizeof(double);
        memcpy(data, &lat_ll, sizeof(double));
        data += sizeof(double);
        memcpy(data, &lon_ll, sizeof(double));
        data += sizeof(double);
        memcpy(data, &lat_ul, sizeof(double));
        data += sizeof(double);
        memcpy(data, &lon_ul, sizeof(double));
        data += sizeof(double);
    }
    // printf("\n");
    return data_origin;
}


void *elaborateSingleCamera(void *ptr)
{
    edge::camera* cam = (edge::camera*) ptr;
    std::cout<<"Starting camera: "<< cam->id << std::endl;
    pthread_t video_cap;
    edge::video_cap_data data;
    data.input          = (char*)cam->input.c_str();
    std::cout<<"Camera input: "<< data.input << std::endl;
    data.width          = cam->streamWidth;
    data.height         = cam->streamHeight;
    data.camId          = cam->id;
    data.frameConsumed  = true;
    data.framesToProcess = cam->framesToProcess;
    data.gstreamer      = cam->gstreamer;
    MasaMessage message;
    Communicator<MasaMessage> communicator;

    if(show)
        viewer->bindCamera(cam->id, &cam->show);
    
    if (pthread_create(&video_cap, NULL, readVideoCapture, (void *)&data)){
        fprintf(stderr, "Error creating thread\n");
        return (void *)1;
    }
    
    int sockfd=-1;
    //instantiate the communicator and the socket
    if (!use_udp_socket){
        communicator.open_client_socket((char*)cam->ipCommunicator.c_str(), cam->portCommunicator);
        int socket = communicator.get_socket();
    } else {
        std::string camCommunicatorPort = std::to_string(cam->portCommunicator);
        sockfd = openUDPsockets(camCommunicatorPort);
    }
    
    //initiate the tracker
    float   dt              = 0.03;
    int     n_states        = 5;
    int     initial_age     = 10;
    bool    tr_verbose      = false;
    tracking::Tracking t(n_states, dt, initial_age, (tracking::Filters_t) cam->filterType);

    cv::Mat frame;
    std::vector<cv::Mat> batch_frame;
    std::vector<cv::Mat> dnn_input;
    cv::Mat distort;
    uint64_t timestamp_acquisition = 0;
    cv::Mat map1, map2;

    std::vector<tk::dnn::box>       detected;
    std::vector<tracking::obj_m>    cur_frame;

    cv::VideoWriter boxes_video;
    double north, east;
    bool ce_verbose = false;
    bool first_iteration = true; 
    bool new_frame = false;

    float scale_x   = cam->hasCalib ? (float)cam->calibWidth  / (float)cam->streamWidth : 1;
    float scale_y   = cam->hasCalib ? (float)cam->calibHeight / (float)cam->streamHeight: 1;

    float err_scale_x = !cam->precision.empty() ? (float)cam->precision.cols  / (float)cam->streamWidth: 1;
    float err_scale_y = !cam->precision.empty() ? (float)cam->precision.rows  / (float)cam->streamHeight: 1;

    int pixel_prec_x, pixel_prec_y;
    uint8_t *d_input, *d_output; 
    float *d_map1, *d_map2;

    //profiling
    edge::Profiler prof(std::to_string(cam->id));
    
    if (recordBoxes){
        boxes_video.open("../data/output/boxes_cam" +
                          std::to_string(data.camId)  + "_" + 
                          std::to_string(data.width)  + "x" +
                          std::to_string(data.height) + "_" +
                          std::to_string(data.framesToProcess) + "frames.mp4",
                          cv::VideoWriter::fourcc('M','P','4','V'), 30, cv::Size(data.width, data.height));
    }
    
    // Declaring UDP variables
    std::vector<tk::dnn::box> box_vector;
    std::vector<std::tuple<double, double>> coords;
    std::vector<std::tuple<double, double>> coordsGeo;
    std::vector<std::tuple<double, double, double, double, double, double, double, double>> boxCoords;
    unsigned int n_frame=0;

    while(gRun){
        prof.tick("Total time");
        batch_frame.clear();
        //copy the frame that the last frame read by the video capture thread
        prof.tick("Copy frame");
        data.mtxF.lock();
        distort = data.frame.clone();
        //std::cout << " Copying in a new iteration" << std::endl;

        timestamp_acquisition = data.tStampMs;
        new_frame = data.frameConsumed;
        data.frameConsumed = true;
        data.mtxF.unlock();
        //std::cout << " ->ELABORATION  FRAME: " <<  distort.empty() << std::endl;
        if(!distort.empty() && !new_frame) {
            n_frame++;
            prof.tock("Copy frame");

            //eventual undistort 
            prof.tick("Undistort");
            if(false){
                if (first_iteration){
                    cam->calibMat.at<double>(0,0)*=  double(cam->streamWidth) / double(cam->calibWidth);
                    cam->calibMat.at<double>(0,2)*=  double(cam->streamWidth) / double(cam->calibWidth);
                    cam->calibMat.at<double>(1,1)*=  double(cam->streamWidth) / double(cam->calibWidth);
                    cam->calibMat.at<double>(1,2)*=  double(cam->streamWidth) / double(cam->calibWidth);

                    cv::initUndistortRectifyMap(cam->calibMat, cam->distCoeff, cv::Mat(), cam->calibMat, cv::Size(cam->streamWidth, cam->streamHeight), CV_32F, map1, map2);

                    checkCuda( cudaMalloc(&d_input, distort.cols*distort.rows*distort.channels()*sizeof(uint8_t)) );
                    checkCuda( cudaMalloc(&d_output, distort.cols*distort.rows*distort.channels()*sizeof(uint8_t)) );
                    checkCuda( cudaMalloc(&d_map1, map1.cols*map1.rows*map1.channels()*sizeof(float)) );
                    checkCuda( cudaMalloc(&d_map2, map2.cols*map2.rows*map2.channels()*sizeof(float)) );
                    frame = distort.clone();

                    checkCuda( cudaMemcpy(d_map1, (float*)map1.data,  map1.cols*map1.rows*map1.channels()*sizeof(float), cudaMemcpyHostToDevice));
                    checkCuda( cudaMemcpy(d_map2, (float*)map2.data,  map2.cols*map2.rows*map2.channels()*sizeof(float), cudaMemcpyHostToDevice));
                    
                    first_iteration = false;
                }
                checkCuda( cudaMemcpy(d_input, (uint8_t*)distort.data,  distort.cols*distort.rows*distort.channels()*sizeof(uint8_t), cudaMemcpyHostToDevice));
                remap(d_input, cam->streamWidth, cam->streamHeight, 3, d_map1, d_map2, d_output, cam->streamWidth , cam->streamHeight, 3);
                checkCuda( cudaMemcpy((uint8_t*)frame.data , d_output, distort.cols*distort.rows*distort.channels()*sizeof(uint8_t), cudaMemcpyDeviceToHost));
                // cv::remap(distort, frame, map1, map2, cv::INTER_LINEAR);
            }
            else{
                frame = distort;
            }

            prof.tock("Undistort");
            batch_frame.push_back(frame);
            //inference
            prof.tick("Inference");
            dnn_input.clear();
            dnn_input.push_back(frame.clone());
            cam->detNN->update(dnn_input);
            detected= cam->detNN->detected;
            prof.tock("Inference");

            cam->detNN->draw(batch_frame);
            if (recordBoxes) boxes_video << frame;

            
            if (!use_udp_socket){

                //feed the tracker
                prof.tick("Tracker feeding");
                cur_frame.clear();
                for(auto d:detected){
                    if(checkClass(d.cl, cam->dataset)){
                        convertCameraPixelsToMapMeters((d.x + d.w / 2)*scale_x, (d.y + d.h)*scale_y, d.cl, *cam, north, east);
                        tracking::obj_m obj;
                        obj.frame       = 0;
                        obj.cl          = d.cl;
                        obj.x           = north;
                        obj.y           = east;
                        pixel_prec_x    = (int)(d.x + d.w / 2)*err_scale_x > cam->precision.cols ? cam->precision.cols : (int)(d.x + d.w / 2)*err_scale_x;
                        pixel_prec_y    = (int)(d.y + d.h)*err_scale_y > cam->precision.rows ? cam->precision.rows : (int)(d.y + d.h)*err_scale_y;
                        obj.error       = !cam->precision.empty() ? cam->precision.at<float>(pixel_prec_y, pixel_prec_x) : 0.0;
                        cur_frame.push_back(obj);
                    }
                }
                t.track(cur_frame,tr_verbose);
                prof.tock("Tracker feeding");
            } else { // NOSOTROS NO EJECUTAMOS EL TRACKER
                // box_vector: 4 esquinas en pixeles
                // coords:convertToMeters -> north,east
                // boxCoords: 4 esquinas coordenadas
                // corrdsGeo: NO se usa
                collectBoxInfo(cam->detNN->batchDetected, box_vector, coords, coordsGeo, boxCoords, scale_x, scale_y, *cam);
                unsigned int size;
                char *data = prepareMessageUDP(box_vector, coords, boxCoords, n_frame, cam->id,
                                               cam->adfGeoTransform[3], cam->adfGeoTransform[0], // pasamos pto.ref
                                               &size, scale_x, scale_y);
                sendUDPMessage(sockfd, n_frame, data, &size);
            }

            //feed the viewer
            prof.tick("Viewer feeding");
            if(show && cam->show)
                viewer->setFrameData(frame, detected, getTrackingLines(t, *cam, 1/scale_x, 1/scale_y,ce_verbose), cam->id);
            prof.tock("Viewer feeding");

            prof.tick("Prepare message"); 
            //send the data if the message is not empty
            if (!use_udp_socket){
                prepareMessage(t, message, cam->geoConv, cam->id, cam->dataset, timestamp_acquisition);
            }

            if (!message.objects.empty()){
                communicator.send_message(&message, cam->portCommunicator);
            }
            prof.tock("Prepare message");   

            prof.tock("Total time");   
            if (verbose) 
                prof.printStats();  
        }
        else 
            usleep(500);

        // Cleaning vars to UDP
        box_vector.clear();
        coords.clear();
        coordsGeo.clear();
        boxCoords.clear();
        
        if (n_frame >= data.framesToProcess ) {
            std::cout << " ELABORATION: " << n_frame  << " processed" << std::endl;
            break;
        }

    }
    gRun = false;
    std::cout << " ELABORATION ended !!!!! Releasing boxes video" << std::endl;
    if (recordBoxes) boxes_video.release();
    pthread_join( video_cap, NULL);

    checkCuda( cudaFree(d_input));
    checkCuda( cudaFree(d_output));
    checkCuda( cudaFree(d_map1));
    checkCuda( cudaFree(d_map2));

    if(use_udp_socket) close(sockfd);

    return (void *)0;
}
