#include "camera_elaboration.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

//#include "tkDNN/utils.h"

#include "Profiler.h"

#include "undistort.h"

//Includes for udp sockets
#include "zmq.hpp"
#include <sys/stat.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>



using std::cin;

// void CharToByte(char* char_arr, unsigned char* unsigned_char_arr, unsigned int count){
//     for(unsigned int i = 0; i < count; i++){
//         unsigned_char_arr[i]= static_cast<unsigned char>(char_arr[i]);
//         }
// }



void sendUDPMessage(int sockfd, char* data ){


    char clientname[1024] = "";
    struct sockaddr_in clientaddr = sockaddr_in();
    socklen_t len = sizeof(clientaddr);

    inet_ntop(AF_INET,&clientaddr.sin_addr,clientname,sizeof(clientname));
    std::string client_ip_str(clientname);


    char buffer_recv[1024];
    int recv_error = recvfrom(sockfd, buffer_recv, 1024,
        MSG_WAITALL, ( struct sockaddr *) &clientaddr,
        &len);

    inet_ntop(AF_INET,&clientaddr.sin_addr,clientname,sizeof(clientname));
    client_ip_str = std::string(clientname);

    std::cout << "****************-------------Preparing UDP Meesage const:"  <<  (const char *)data <<  std::endl;
    

    
    size_t data_size = strlen((const char *)data);
    // printBufferHex((const char *)data, data_size);
    if (client_ip_str != "0.0.0.0"){
        sendto(sockfd, (const char *)data, strlen(data),MSG_DONTWAIT, (const struct sockaddr *) &clientaddr,sizeof(clientaddr));
    }
    free(data); 
}


int openUDPsockets(std::string& socketPort){


    zmq::message_t unimportant_message;
    zmq::context_t context(1);

    struct sockaddr_in servaddr;
    int sockfd;
    
    std::cout << "Listdening to tcp://0.0.0.0:" << socketPort << " waiting for COMPSs ack to start" << std::endl;
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


char* prepareMessageUDP2( const std::vector<Box> &boxes, unsigned int n_frame, std::string cam_id, 
                         float scale_x, float scale_y) {
    
    char* buffer= new char[60  * boxes.size()];
    // unsigned char* Bytes = new unsigned char[41  * boxes.size()];
    // We need to keep pointer initial position
    char *buffer_origin = buffer; 
    unsigned long long timestamp = getTimeMs();
    
    for (int i = 0; i < boxes.size(); i++) {

            std::string result;
            for(int i = 0; i < cam_id.size(); i++) {
                char buffer[20];
                sprintf(buffer, "%X ", cam_id[i]);
                result += buffer;
            }
            std::cout << "hexval1: " << result << std::endl;
            
            
            
            // unsigned char* cam_id2 = cam_id.c_str();
            // std::cout << "hexval___: " <<  cam_id2  << std::endl;

        //flag, cam_id , n_frame, timestamp , box_x, box_y, box_w, box_h, score, class
        // size_t size_of_data = snprintf(buffer, 60, "%02x%04x%04x%016lx%04x%08lx%04x%04x%04x%04x",
        size_t size_of_data = snprintf(buffer, 60, "%02x%02x%02x%02x%02x%04x%016llx%04x%04x%04x%04x%08lx%04x",
                true, cam_id[0], cam_id[1], cam_id[2], cam_id[3], n_frame, timestamp, 
                int(boxes[i].x), int(boxes[i].y), 
                int(boxes[i].w), int(boxes[i].h),
                 *(unsigned long*)&boxes[i].prob, boxes[i].cl);
        std::cout << "Preparing UDP Meesage of length "  << size_of_data  <<  std::endl;
        std::cout << "Preparing UDP Meesage Content BUFFER- :"  << buffer <<  std::endl;
        // Set position at the end of the buffer to insert there next iteration
        buffer += size_of_data ;
        std::cout << "Preparing UDP Meesage TOTAL:"  << strlen(buffer_origin) <<  std::endl;

    }



    return buffer_origin;

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
        std::cout<<"Starting camera: "<< cam->id << std::endl;

   
    
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
        // Client handshake and sending cam Info
        std::string camCommunicatorPort = std::to_string(cam->portCommunicator);
        std::cout << "- - - - -> WAITING FOR UDP HANDSHAKE  " << std::endl;
        sockfd = openUDPsockets(camCommunicatorPort);

        // const char *camData[100] = "Send this with UDP";
        char* camData= new char[245];
        std::cout << "- - - - -> SENDING DATA INFO :::: "<< data.camId << std::endl;
        size_t size_of_data = snprintf(camData, 245, "%s|%s|%d|%d|%d|%d|%s",
                                       "Send this with UDP: ",
                                       data.camId.c_str(), data.gstreamer, cam->framesToProcess, 
                                       data.height, data.width, data.input);
        std::cout << "- - - - -> SENDING DATA INFO  " << std::endl;
        sendUDPMessage(sockfd, camData);

    }
    
    
    cv::Mat frame;
    std::vector<cv::Mat> batch_frame;
    // std::vector<cv::Mat> dnn_input;
    cv::Mat distort;
    uint64_t timestamp_acquisition = 0;
    cv::Mat map1, map2;

    // std::vector<tk::dnn::box>       detected;

    cv::VideoWriter boxes_video;
    double north, east;
    bool ce_verbose = false;
    bool first_iteration = true; 
    bool new_frame = false;

    float scale_x   = cam->hasCalib ? (float)cam->calibWidth  / (float)cam->streamWidth : 1;
    float scale_y   = cam->hasCalib ? (float)cam->calibHeight / (float)cam->streamHeight: 1;

    std::cout << std::setprecision (15)  << "ESCALA X: "<<  (float)cam->calibWidth << " / " << (float)cam->streamWidth << " =  " << scale_x << std::endl;
    std::cout << std::setprecision (15)  << "ESCALA Y: "<<  (float)cam->calibHeight << " / " << (float)cam->streamHeight << " =  " << scale_y <<std::endl;
    std::cout << std::setprecision (15)  << "--- > " << cam->hasCalib << std::endl;



    float err_scale_x = !cam->precision.empty() ? (float)cam->precision.cols  / (float)cam->streamWidth: 1;
    float err_scale_y = !cam->precision.empty() ? (float)cam->precision.rows  / (float)cam->streamHeight: 1;

    int pixel_prec_x, pixel_prec_y;
    uint8_t *d_input, *d_output; 
    float *d_map1, *d_map2;

    //profiling
    edge::Profiler prof(cam->id);
    
    if (recordBoxes){
        boxes_video.open("../data/output/boxes_cam" +
                          data.camId  + "_" + 
                          std::to_string(data.width)  + "x" +
                          std::to_string(data.height) + "_" +
                          std::to_string(data.framesToProcess) + "frames.mp4",
                          cv::VideoWriter::fourcc('M','P','4','V'), 30, cv::Size(data.width, data.height));
    }
    
    // Declaring UDP variables
    YoloV8Engine engine("../yolov6.engine", data.width, data.height, 0.7);
    
    unsigned int n_frame=0;
    std::cout << " -> CASO CE - pre while " << gRun << std::endl;
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
            
            if (n_frame % 1 == 0){

            prof.tock("Copy frame");
            std::cout << "A: n_frame = " << n_frame <<std::endl;

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

            engine.preprocess(frame);
            engine.infer();
            const auto boxes = engine.postprocess();

            printf("Number of objects detected: %ld\n", boxes.size());
            
            
            // dnn_input.clear();
            // dnn_input.push_back(frame.clone());
            // cam->detNN->update(dnn_input);
            // detected = cam->detNN->detected;
            prof.tock("Inference");

            // cam->detNN->draw(batch_frame);
            if (recordBoxes) boxes_video << frame;
            std::cout << "B: n_frame = " << n_frame <<std::endl;


            if (use_udp_socket){

                // box_vector: 4 esquinas en pixeles
                // collectBoxInfo(cam->detNN->batchDetected, box_vector, coords, coordsGeo, boxCoords, scale_x, scale_y, *cam);
                unsigned int size;
                std::cout << std::setprecision (15)  << "INIT POINT!: "<< cam->adfGeoTransform[3] << " " << cam->adfGeoTransform[0] << " " <<std::endl;
                // Send data trough UDP: box_vector.size()
                // char data[43 * box_vector.size()];
                char *data = prepareMessageUDP2(boxes, n_frame, cam->id, scale_x, scale_y);
                // std::cout << "Press Enter to continue…" << std::endl;
                // cin.get();
                std::cout << "CCCC: n_frame = " << n_frame <<std::endl;
                sendUDPMessage(sockfd, data);
            }

            prof.tick("Prepare message"); 
            //send the data if the message is not empty

            if (!message.objects.empty()){
                communicator.send_message(&message, cam->portCommunicator);
            }
            prof.tock("Prepare message");   

            prof.tock("Total time");   
            if (verbose) 
                prof.printStats();  
            } // if -> jump with %

        } // if -> image empty
        else 
            usleep(500);

        // Cleaning vars to UDP
        // box_vector.clear();
        
        if (n_frame >= data.framesToProcess ) {
            std::cout << " ELABORATION: " << n_frame  << " processed" << std::endl;
            break;
        }

    } // while grun
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
