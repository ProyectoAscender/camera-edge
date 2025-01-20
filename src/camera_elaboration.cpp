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

void sendUDPMessage2(int sockfd, char* data, unsigned int *data_size, sockaddr_in& clientaddr) {
    // Validar datos de entrada
    if (data == nullptr || data_size == 0) {
        std::cerr << "[ERROR] Datos inválidos para enviar." << std::endl;
        return;
    }

    // Enviar el mensaje usando sendto
    ssize_t bytes_sent = sendto(sockfd, data, *data_size, 0, (struct sockaddr*)&clientaddr, sizeof(clientaddr));
    if (bytes_sent < 0) {
        perror("[ERROR] Error al enviar el mensaje UDP");
    } else {
        std::cout << "[INFO] Mensaje enviado con éxito (" << bytes_sent << " bytes)" << std::endl;
    }
}


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
   // if (client_ip_str != "0.0.0.0"){
        sendto(sockfd, (const char *)data, strlen(data),MSG_DONTWAIT, (const struct sockaddr *) &clientaddr,sizeof(clientaddr));
   // }
    free(data); 
}

int setupUDPConnection(sockaddr_in& servaddr, std::string& socketPort) {

    zmq::message_t unimportant_message;
    zmq::context_t context(1);

    std::cout << "Listdening to tcp://0.0.0.0:" << socketPort << " waiting for COMPSs ack to start" << std::endl;
    zmq::socket_t *app_socket = new zmq::socket_t(context, ZMQ_REP);
    app_socket->bind("tcp://0.0.0.0:" + socketPort);
    app_socket->recv(unimportant_message); // wait for python workflow to ack to start processing frames
    app_socket->close();
    delete app_socket;

    std::cout << "Listening to udp in://0.0.0.0:" << socketPort << std::endl;

    // Crear socket UDP
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("[ERROR] Error al crear el socket");
        return -1;
    }

    // Configurar la dirección del emisor (servidor)
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(atoi(socketPort.c_str())); // Puerto para escuchar solicitudes
    servaddr.sin_addr.s_addr = INADDR_ANY;

    // Asociar el socket a la dirección y puerto
    if (bind(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
        perror("[ERROR] Error al hacer bind");
        close(sockfd);
        return -1;
    }

    std::cout << "[INFO] Servidor UDP configurado y esperando solicitudes..." << std::endl;
    return sockfd;
}

int openUDPsockets(std::string& socketPort){

    zmq::message_t unimportant_message;
    zmq::context_t context(1);

    struct sockaddr_in servaddr;
    int sockfd;

    std::cout << "Listdening to tcp://0.0.0.0:" << socketPort << " waiting for COMPSs ack to start" << std::endl;
    zmq::socket_t *app_socket = new zmq::socket_t(context, ZMQ_REP);
    app_socket->bind("tcp://0.0.0.0:" + socketPort);
    app_socket->recv(unimportant_message); // wait for python workflow to ack to start processing frames
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


char* prepareMessageUDP2( const std::vector<Box> &boxes, unsigned int n_frame, std::string cam_id, unsigned int *message_size) {
    
    char* buffer= new char[CHAR_BOX_SIZE  * boxes.size() + 1];
    // unsigned char* Bytes = new unsigned char[41  * boxes.size()];
    // We need to keep pointer initial position
    char *buffer_origin = buffer;
    unsigned long long timestamp = getTimeMs();

    for (int i = 0; i < boxes.size(); i++) {
    // for (int i = 0; i < 3; i++) {            // for debugging
            std::string result;
            for(int i = 0; i < cam_id.size(); i++) {
                char buffer[20];
                sprintf(buffer, "%X ", cam_id[i]);
                result += buffer;
            }


            // std::cout << "hexval1: " << result << std::endl;
            // unsigned char* cam_id2 = cam_id.c_str();
            // std::cout << "hexval___: " <<  cam_id2  << std::endl;

        //flag, cam_id , n_frame, timestamp , box_x, box_y, box_w, box_h, score, class
        // // size_t size_of_data = snprintf(buffer, 60, "%02x%04x%04x%016lx%04x%08lx%04x%04x%04x%04x",
        // std::cout<<"Box : " << i << " - "<< boxes[i].x << std::endl;
        // std::cout<<"Box : " << i << " - "<< boxes[i].y << std::endl;
        // std::cout<<"Box : " << i << " - "<< boxes[i].w << std::endl;
        // std::cout<<"Box : " << i << " - "<< boxes[i].h << std::endl;


        size_t size_of_data = snprintf(buffer, CHAR_BOX_SIZE + 1,
                                        "%02x%02x%02x%02x%02x%04x%016llx%04x%04x%04x%04x%08x%04x",      // With everything
                                        // "%02x%02x%02x%02x%02x%04x%016llx%04x%04x%04x%04x",                  // For testing
                                        true,                                               // Flag (boolean, 1 byte, 2 hex digits)
                                        cam_id[0], cam_id[1], cam_id[2], cam_id[3],         // Camera ID (4 bytes, 4 x 2 hex digits = 8 hex digits)
                                        n_frame,                                            // Frame number (2 bytes, 4 hex digits)
                                        timestamp,                                          // Timestamp (8 bytes, 16 hex digits)
                                        int(boxes[i].x),                                    // Box x-coordinate (2 bytes, 4 hex digits)
                                        int(boxes[i].y),                                    // Box y-coordinate (2 bytes, 4 hex digits)
                                        int(boxes[i].w),                                    // Box width (2 bytes, 4 hex digits)
                                        int(boxes[i].h),                                    // Box height (2 bytes, 4 hex digits)
                                        *(unsigned int*)&boxes[i].prob,                     // Probability (4 bytes, 8 hex digits)
                                        boxes[i].cl                                         // Class (2 bytes, 4 hex digits)
                                    );



        // std::cout << "Preparing UDP Meesage of length "  << size_of_data  <<  std::endl;
        std::cout << "\tBox "  << i <<  std::endl;
        std::cout << "\tPreparing UDP Meesage Content: "  << buffer <<  std::endl;
        // Set position at the end of the buffer to insert there next iteration
        buffer += size_of_data ;
        std::cout << "\tUDP Meesage of length:"  << strlen(buffer_origin) << std::endl;

    }


    // Store length in message_size for the caller
    *message_size = strlen(buffer_origin);


    return buffer_origin;

}

void *elaborateSingleCamera(void *ptr)
{
    edge::camera* cam = (edge::camera*) ptr;
    std::cout<<"STARTING CAMERA_ELABORATION..." << std::endl;
    std::cout<<"\tStarting camera: "<< cam->id << std::endl;
    pthread_t video_cap;
    edge::video_cap_data data;
    data.input          = (char*)cam->input.c_str();
    std::cout<<"\tCamera input: "<< data.input << std::endl;
    data.camId          = cam->id;
    data.frameConsumed  = true;
    data.framesToProcess = cam->framesToProcess;
    data.gstreamer      = cam->gstreamer;
    data.dataPath = cam->dataPath;

    sockaddr_in servaddr = {};
    sockaddr_in clientaddr = {};
    socklen_t clientaddr_len = sizeof(clientaddr);
    int bufferSize = 70;

    std::string camCommunicatorPort = std::to_string(cam->portCommunicator);

    if (pthread_create(&video_cap, NULL, readVideoCapture, (void *)&data)){
        fprintf(stderr, "Error creating thread\n");
        return (void *)1;
    } else {
        std::cout<<"\nLAUNCHING VIDEO CAPTURE..." << std::endl;
    }



    usleep(3000000);
    int sockfd = setupUDPConnection(servaddr, camCommunicatorPort);
    if (sockfd < 0) {
        return (void *)1; // Terminar si no se pudo configurar
    }

    char buffer_recv[1024];
    ssize_t bytes_received = recvfrom(sockfd, buffer_recv, sizeof(buffer_recv), 0,
                                        (struct sockaddr*)&clientaddr, &clientaddr_len);

    if (bytes_received < 0) {
        perror("[ERROR] Error al recibir datos");
    }

    // Obtener IP del cliente
    char client_ip[INET_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET, &clientaddr.sin_addr, client_ip, sizeof(client_ip));
    std::cout << "\n\n[INFO] Solicitud recibida de " << client_ip << ":" << ntohs(clientaddr.sin_port) << std::endl;

    // Datos a enviar en esta iteración (pueden variar)
    char* camData = new char[bufferSize];

    std::cout << "- - - - -> SENDING CAM INFO :::: "<< data.camId << std::endl;
    size_t size_of_data = snprintf(camData, bufferSize, "%s|%s|%d|%d|%d|%d|%s",
                                    "Sent  UDP: ",
                                    data.camId.c_str(), data.gstreamer, cam->framesToProcess, 
                                    data.frame.rows, data.frame.cols, data.dataPath.c_str());
    std::cout << "- - - - -> SENDING CAM INFO  " << camData << std::endl;
    std::cout << "- - - - -> CAM INFO SIZE  " << strlen(camData) << std::endl;
    // sendUDPMessage2(sockfd, camData, strlen(camData), clientaddr);

    unsigned int length = (unsigned int)strlen(camData);
    sendUDPMessage2(sockfd, camData, &length, clientaddr);


    std::cout << "- - - - -> CAM INFO SENT\n\n" << std::endl;
    
    
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

    int pixel_prec_x, pixel_prec_y;
    uint8_t *d_input, *d_output; 
    float *d_map1, *d_map2;

    //profiling
    edge::Profiler prof(cam->id);
    
    if (recordBoxes){
        boxes_video.open("../data/output/boxes_cam" +
                          data.camId  + "_" + 
                          std::to_string(data.frame.cols)  + "x" +
                          std::to_string(data.frame.rows) + "_" +
                          std::to_string(data.framesToProcess) + "frames.mp4",
                          cv::VideoWriter::fourcc('M','P','4','V'), 30, cv::Size(data.width, data.height));
    }
    
    // Declaring UDP variables
    YoloV6Engine engine("../yolov6_agx13.engine", data.frame.cols, data.frame.rows, 0.5);
    
    unsigned int n_frame=0;



    // Pinned memory allocation
    void* pHost;
    checkCuda(cudaHostAlloc(&pHost,
                            data.frame.cols * data.frame.rows * 3,
                            cudaHostAllocPortable | cudaHostAllocMapped | cudaHostAllocWriteCombined));
    cv::Mat *frame = new cv::Mat(data.frame.rows, data.frame.cols, CV_8UC3, pHost);



    while(gRun){
        prof.tick("Total time");
        batch_frame.clear();
        //copy the frame that the last frame read by the video capture thread
        prof.tick("Copy frame");
        data.mtxF.lock();
        data.frame.copyTo(*frame);

        //std::cout << " Copying in a new iteration" << std::endl;

        timestamp_acquisition = data.tStampMs;
        new_frame = data.frameConsumed;
        data.frameConsumed = true;
        data.mtxF.unlock();
       
        if(!frame->empty() && !new_frame) {
            batch_frame.push_back(*frame);
            prof.tock("Copy frame");
            n_frame++;
            
            std::cout << "\n\nFrame " << n_frame << " being processed..." <<std::endl;

            //inference
            prof.tick("Inference");

            engine.preprocess(*frame);
            engine.infer();
            const auto boxes = engine.postprocess();
            printf("Number of objects detected: %ld\n", boxes.size());

            prof.tock("Inference");

            if (recordBoxes) boxes_video << *frame;


            prof.tick("Prepare message"); 

            if (use_udp_socket){
                unsigned int message_size;
                // Send data trough UDP: box_vector.size()
                // char data[43 * box_vector.size()];
                char *data = prepareMessageUDP2(boxes, n_frame, cam->id, &message_size);
                // std::cout << "Press Enter to continue…" << std::endl;
                // cin.get();

                sendUDPMessage2(sockfd, data, &message_size, clientaddr);
            }

            
            prof.tock("Prepare message");   

            prof.tock("Total time");   
            if (verbose) 
                prof.printStats();  

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
    checkCuda(cudaFreeHost(pHost));
    delete frame;

    if(use_udp_socket) close(sockfd);

    return (void *)0;
}
