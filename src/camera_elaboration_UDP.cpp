#include "camera_elaboration.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include "Profiler.h"
#include "undistort.h"

// Include UDP
#include <sys/socket.h>     // For UDP
#include <netinet/in.h>     // For sockaddr_in
#include <arpa/inet.h>      // For inet_ntop, etc.
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>


using std::cin;


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



int setupUDPHandshake(sockaddr_in &clientAddr, edge::video_cap_data &data, int &udpSock, int handshakePort, int bufferSize)
{
/**
 * @brief Sets up a UDP handshake between the C++ server and a Python client, waits for the camera 
 *        to start streaming frames, and exchanges camera information.
 * 
 * This function performs the following steps:
 * 1. Creates a UDP socket and binds it to the specified handshake port.
 * 2. Waits for a handshake message from the Python client and sends an acknowledgment ("ACK") back.
 * 3. Polls the camera data to ensure that frames are being streamed by the NX11 camera. If this is not available, it waits for NX11
 * 4. Sends camera configuration and information (e.g., resolution, gstreamer flag, and data path) back to the Python client.
 * 
 * @param clientAddr Reference to a sockaddr_in structure to store the Python client's address after the handshake.
 * @param data Reference to a `video_cap_data` structure containing camera configuration and frame data.
 * @param udpSock Reference to an integer where the created UDP socket descriptor will be stored.
 * @param handshakePort The port number on which the handshake will be performed.
 * @param bufferSize The size of the buffer used to construct the camera info message.
 * 
 * @return int Returns 0 on successful handshake and data exchange. Returns -1 on errors such as socket creation,
 *         binding, or data transmission failures.
 * 
 * @note This function blocks until a handshake message is received and the camera starts streaming frames.
 *       It uses a 1-second polling interval to check for frames.
 */

    // Create UDP socket
    udpSock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udpSock < 0) {
        perror("[camera_elaboration_UDP] Error creating UDP socket");
        return -1;
    }

    // Bind on handshakePort
    sockaddr_in servAddr;
    memset(&servAddr, 0, sizeof(servAddr));
    servAddr.sin_family = AF_INET;
    servAddr.sin_port   = htons(handshakePort);
    servAddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(udpSock, (struct sockaddr*)&servAddr, sizeof(servAddr)) < 0) {
        perror("[camera_elaboration_UDP] Error binding UDP socket");
        close(udpSock);
        return -1;
    }

    socklen_t clientLen = sizeof(clientAddr);
    std::cout << "[camera_elaboration_UDP] Waiting for Python handshake on port " << handshakePort << "...\n";

    // Wait for an empty handshake message from Python
    char buffer[128];
    ssize_t rcv = recvfrom(udpSock, buffer, sizeof(buffer), 0, (struct sockaddr*)&clientAddr, &clientLen);
    if (rcv < 0) {
        perror("[camera_elaboration_UDP] Error receiving handshake");
        return -1;
    }
    std::cout << "[camera_elaboration_UDP] Received handshake from Python.\n";


    // Acknowledge handshake
    const char* ack = "ACK";
    ssize_t sent_ack = sendto(udpSock, ack, strlen(ack), 0, (struct sockaddr*)&clientAddr, clientLen);
    if (sent_ack < 0) {
        perror("[camera_elaboration_UDP] Error sending ACK to Python");
        return -1;
    }
    std::cout << "[camera_elaboration_UDP] Sent ACK to Python.\n";



    // Wait until data.frame has something -> AKA waiting for NX11 to be GStreaming frames.
    while (data.frame.empty()) {
        std::cout << "[camera_elaboration_UDP] Waiting for frames from NX11...\n";
        usleep(1000000); // 1 s
    }

    // At this point, camera is streaming frames
    std::cout << "- - - - -> Camera " << data.camId << " is ready. " << std::endl;



    // Data to send in handshake
    char infoBuffer[bufferSize];
    snprintf(infoBuffer, bufferSize, "%s|%s|%d|%d|%d|%d|%s",
                "Sent  UDP: ",
                data.camId.c_str(), data.gstreamer, data.framesToProcess, 
                data.frame.rows, data.frame.cols, data.dataPath.c_str());


    // Send the camera info back to Python
    ssize_t sent = sendto(udpSock, infoBuffer, strlen(infoBuffer), 0,
                          (struct sockaddr*)&clientAddr, clientLen);
    if (sent < 0) {
        perror("[camera_elaboration_UDP] Error sending camera info to Python");
        return -1;
    }
    std::cout << "[camera_elaboration_UDP] Sent camera info: " << infoBuffer << "\n";
    return 0;
}




void sendBoundingBoxes(int udpSock, sockaddr_in &clientAddr, socklen_t clientLen, const std::vector<Box> &boxes, unsigned int frameCounter, const std::string &camId, uint64_t timestampAcquisition, edge::Profiler &prof) {
/**
 * @brief Sends bounding box data over a UDP socket to a specified client.
 * 
 * This function prepares a message containing bounding box information and sends it over
 * the provided UDP socket to the given client address. The message includes metadata such
 * as the frame counter, camera ID, and timestamp. Profiling metrics are recorded for the
 * message preparation step.
 * 
 * @param udpSock The UDP socket used for sending data.
 * @param clientAddr The sockaddr_in structure containing the client's address.
 * @param clientLen The length of the client's address structure.
 * @param boxes A vector of `Box` objects containing the bounding box data to be sent.
 * @param frameCounter The current frame counter for the bounding box data.
 * @param camId The camera ID associated with the bounding box data.
 * @param timestampAcquisition The acquisition timestamp for the bounding box data.
 * @param prof Reference to a `Profiler` object for recording timing metrics.
 * 
 * @note If the message size is zero, no data is sent. The function logs errors if the
 *       `sendto` operation fails.
 */

    prof.tick("Prepare message"); 
    unsigned int message_size;
    char* payload = prepareMessage(boxes, &frameCounter, camId, &message_size, &timestampAcquisition);
    prof.tock("Prepare message");

    // Send the bounding boxes over UDP
    if (message_size > 0) {
        ssize_t sent = sendto(udpSock, payload, message_size, 0,
                                (struct sockaddr*)&clientAddr, clientLen);
        if (sent < 0) {
            perror("[camera_elaboration_UDP] Error sending bounding boxes");
        }
    }
    delete[] payload;
}




char* prepareMessage( const std::vector<Box> &boxes, unsigned int *frameCounter, std::string cam_id, unsigned int *message_size, uint64_t *timestamp_acquisition) {
/**
 * @brief Prepares a serialized message containing bounding box data for transmission.
 * 
 * This function generates a serialized message from a vector of bounding boxes, including
 * metadata such as the frame counter, camera ID, and acquisition timestamp. The resulting
 * message is encoded as a hexadecimal string suitable for UDP transmission.
 * 
 * @param boxes A vector of `Box` objects representing the bounding box data.
 * @param frameCounter A pointer to the frame counter associated with the bounding box data.
 * @param cam_id The camera ID as a string.
 * @param message_size A pointer to store the size of the generated message.
 * @param timestamp_acquisition A pointer to the acquisition timestamp for the bounding box data.
 * 
 * @return A dynamically allocated character buffer containing the serialized message.
 *         The caller is responsible for freeing the memory using `delete[]`.
 * 
 * @note The size of the buffer is calculated based on the number of bounding boxes
 *       and their corresponding serialized data size. The function assumes `CHAR_BOX_SIZE`
 *       defines the per-box serialized size.
 */

    char* buffer = new char[CHAR_BOX_SIZE  * boxes.size() + 1];
    char *buffer_origin = buffer;

    for (int i = 0; i < boxes.size(); i++) {
        size_t size_of_data = snprintf(buffer,
                                        CHAR_BOX_SIZE + 1,
                                        "%02x%02x%02x%02x%02x%04x%016llx%04x%04x%04x%04x%08x%04x",
                                        static_cast<unsigned int>(true),
                                        static_cast<unsigned int>(static_cast<unsigned char>(cam_id[0])),
                                        static_cast<unsigned int>(static_cast<unsigned char>(cam_id[1])),
                                        static_cast<unsigned int>(static_cast<unsigned char>(cam_id[2])),
                                        static_cast<unsigned int>(static_cast<unsigned char>(cam_id[3])),
                                        static_cast<unsigned int>(*frameCounter),
                                        static_cast<unsigned long long>(*timestamp_acquisition),
                                        static_cast<unsigned int>(boxes[i].x),
                                        static_cast<unsigned int>(boxes[i].y),
                                        static_cast<unsigned int>(boxes[i].w),
                                        static_cast<unsigned int>(boxes[i].h),
                                        static_cast<unsigned int>(*(unsigned int*)&boxes[i].prob),
                                        static_cast<unsigned int>(boxes[i].cl)
                                    );


        buffer += size_of_data ;
    }

    *message_size = strlen(buffer_origin);
    

    return buffer_origin;
}





void *elaborateSingleCamera_UDP(void *ptr)
{
    edge::camera* cam = (edge::camera*) ptr;
    std::cout<<"[camera_elaboration_UDP] STARTING camera_elaboration_UDP..." << std::endl;
    std::cout<<"[camera_elaboration_UDP] Starting camera: "<< cam->id << std::endl;

    // Create a thread for video capture
    pthread_t video_cap;
    edge::video_cap_data data;
    data.input              = (char*)cam->input.c_str();
    data.camId              = cam->id;
    data.frameConsumed      = true;
    data.framesToProcess    = cam->framesToProcess;
    data.gstreamer          = cam->gstreamer;
    data.dataPath           = cam->dataPath;




    if (pthread_create(&video_cap, NULL, readVideoCapture, (void*)&data)){
        std::cerr << "[camera_elaboration_UDP] Error creating capture thread\n";
        return (void*)1;
    }
    std::cout << "[camera_elaboration_UDP] Video capture launched.\n";




    //=============================
    // 1) HANDSHAKE (UDP)
    //=============================

    int handshakePort = cam->portCommunicator;
    int udpSock = -1;
    sockaddr_in clientAddr;
    memset(&clientAddr, 0, sizeof(clientAddr));

    // Message variables
    int bufferSize = 70;

    if (setupUDPHandshake(clientAddr, data, udpSock, handshakePort, bufferSize) != 0) {
        std::cerr << "[camera_elaboration_UDP] Handshake failed.\n";
        return (void*)1;
    }
    std::cout << "[camera_elaboration_UDP] Handshake done, will now stream bounding boxes.\n";
    

    //=============================
    // Boxes sending (UDP)
    //=============================
    // We'll reuse clientAddr for sending bounding boxes
    socklen_t clientLen = sizeof(clientAddr);




    // YOLO init
    YoloV6Engine engine("../yolov6_agx13.engine", data.frame.cols, data.frame.rows, 0.5);

    // Initialize stuff for saving video with boxes
    cv::VideoWriter boxes_video;
    if (recordBoxes){
        boxes_video.open("../data/output/boxes_cam" +
                          data.camId  + "_" + 
                          std::to_string(data.frame.cols)  + "x" +
                          std::to_string(data.frame.rows) + "_" +
                          std::to_string(data.framesToProcess) + "frames.mp4",
                          cv::VideoWriter::fourcc('M','P','4','V'), 30,
                          cv::Size(data.frame.cols, data.frame.rows));
    }


    // Pinned memory allocation
    void* pHost;
    checkCuda(cudaHostAlloc(&pHost,
                            data.frame.cols * data.frame.rows * 3,
                            cudaHostAllocPortable | cudaHostAllocMapped | cudaHostAllocWriteCombined));
    cv::Mat *frame = new cv::Mat(data.frame.rows, data.frame.cols, CV_8UC3, pHost);

    // Launch Profiler
    edge::Profiler prof(cam->id);

    // Start frame counter (only for this file, not shared)
    unsigned int n_frame=0;



    // MAIN LOOP
    while(gRun){
        prof.tick("Total time");

        // Copy the last frame from the capture thread
        prof.tick("Copy frame");
        data.mtxF.lock();
        data.frame.copyTo(*frame);
        uint64_t timestamp_acquisition = data.tStampMs;
        unsigned int frameCounter = data.frameCounter;
        bool new_frame = data.frameConsumed;
        data.frameConsumed = true;
        data.mtxF.unlock();
        prof.tock("Copy frame");

       // If there's a new frame, run inference
        if(!frame->empty() && !new_frame) {
            n_frame++;
            
            std::cout << "\n\nVC Frame:  " << frameCounter << "; CE frame: " << n_frame << " with timestamp: " << timestamp_acquisition <<std::endl;

            // YOLO inference
            prof.tick("Inference");
            engine.preprocess(*frame);
            engine.infer();
            const auto boxes = engine.postprocess();
            std::cout << "Number of objects detected: " << boxes.size() << std::endl;
            prof.tock("Inference");


            // Save frame with boxes To Do
            if (recordBoxes) {
                boxes_video << *frame;
            }


            // Debugg stuff
            if(data.frame.rows!= 720 || data.frame.cols != 1280){
                std::cout << "\n\nIMAGE RESOLUTION CHANGED FOR SOME REASON " <<std::endl;
                std::cout << "Frame size mismatch: rows = " << data.frame.rows
                    << ", cols = " << data.frame.cols
                    << " (expected 1280x720)" << std::endl;
                usleep(60000000); // 60 s
            }

            //=============================
            // 2) Send bounding boxes (UDP)
            //=============================

            sendBoundingBoxes(udpSock, clientAddr, clientLen, boxes, frameCounter, cam->id, timestamp_acquisition, prof);



            prof.tock("Total time");

               
            if (verbose) 
                prof.printStats();  



        } // if -> image empty
        else {
            // no new frame
            usleep(500);
        }
        

        if (n_frame >= data.framesToProcess ) {
            std::cout << "[camera_elaboration_UDP] " << data.camId
                      << " done after " << n_frame << " frames.\n";
            break;
        }
    }



    gRun = false;
    std::cout << "[camera_elaboration_UDP] Elaboration ended for cam " << cam->id << ".\n";

    if (recordBoxes) {
        boxes_video.release();
    }

    pthread_join( video_cap, NULL);

    checkCuda( cudaFreeHost(pHost));

    delete frame;


    // close UDP
    if (udpSock >= 0) {
        close(udpSock);
    }


    return (void *)0;
}
