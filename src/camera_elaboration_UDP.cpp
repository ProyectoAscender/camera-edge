#include "camera_elaboration.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include "Profiler.h"

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
 * 2. Waits for a handshake message ("REQ") from the Python client.
 * 3. If the camera is not ready, responds with "NOTREADY" and waits.
 * 4. If the camera is ready:
 *    - Sends "READY" to signal readiness.
 *    - Sends camera configuration (resolution, gstreamer flag, data path).
 *    - Waits for an "ACK" from the Python client before finalizing.
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
    std::cout << "[camera_elaboration_UDP] Waiting for Python handshake on port " << handshakePort << "...\n";



    // Handshake main loop, coordinating camera, camera edge and smart-city
    while(true){

        // Receive a handshake request (REQ)
        char buffer[128];
        socklen_t clientLen = sizeof(clientAddr);
        ssize_t rcv = recvfrom(udpSock, buffer, sizeof(buffer), 0, (struct sockaddr*)&clientAddr, &clientLen);
        if (rcv < 0) {
            perror("[camera_elaboration_UDP] Error receiving handshake");
            close(udpSock);
            return -1;
        }
        buffer[rcv] = '\0';  // Null-terminate the received message
        std::string request(buffer);
        std::cout << "[camera_elaboration_UDP] Received handshake request: " << request << "\n";


        // Ceck if camera (NX11) is ready
        if (data.frame.empty()) {
            // Camera not ready yet: respond "NOTREADY"
            const char* notReadyMsg = "NOTREADY";
            sendto(udpSock, notReadyMsg, strlen(notReadyMsg), 0, (struct sockaddr*)&clientAddr, clientLen);
            std::cout << "[camera_elaboration_UDP] Sent NOTREADY --> Continue Waiting for NX11...\n";
            usleep(1000 * 1000); // 1s delay

        } else {
            // Camera is ready
            
            // send "READY" first
            const char* readyMsg = "READY";
            sendto(udpSock, readyMsg, strlen(readyMsg), 0, (struct sockaddr*)&clientAddr, clientLen);
            std::cout << "[camera_elaboration_UDP] Sent READY to client.\n";

            // Wait for a small delay to ensure the client processes the "READY"
            usleep(500 * 1000); // 500ms delay

            // Send camera information
            char infoBuffer[bufferSize];
            snprintf(infoBuffer, bufferSize, "%s|%s|%d|%d|%d|%d|%s",
                        "Sent  UDP: ",
                        data.camId.c_str(), data.gstreamer, data.framesToProcess, 
                        data.frame.rows, data.frame.cols, data.dataPath.c_str());
            // Send the camera info back to Python
            ssize_t sent = sendto(udpSock, infoBuffer, strlen(infoBuffer), 0, (struct sockaddr*)&clientAddr, clientLen);
            if (sent < 0) {
                perror("[camera_elaboration_UDP] Error sending camera info to Python");
                close(udpSock);
                return -1;
            }
            std::cout << "[camera_elaboration_UDP] Sent camera info: " << infoBuffer << "\n";



            // Wait for an ACK from the client              -->             THIS IS NOT NECESSARY, BUT USEFUL FOR DEBUGGING
            char ackBuffer[128];
            ssize_t ackReceived = recvfrom(udpSock, ackBuffer, sizeof(ackBuffer) - 1, 0, (struct sockaddr*)&clientAddr, &clientLen);
            if (ackReceived < 0) {
                perror("[camera_elaboration_UDP] Error receiving ACK");
                close(udpSock);
                return -1;
            }
            ackBuffer[ackReceived] = '\0';
            std::string ackResponse(ackBuffer);
            
            if (ackResponse == "ACK") {
                std::cout << "[camera_elaboration_UDP] ACK received. Handshake completed successfully.\n";
                return 0; // Handshake successful
            } else {
                std::cout << "[camera_elaboration_UDP] Unexpected response: " << ackResponse << "\n";
            }
        }
    }

    close(udpSock);
    return -1; // Should never reach here
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


    if (message_size > 0 && payload != nullptr) {
        std::cout << "Enviando mensaje de tamaño: " << message_size << " bytes." << std::endl;
        std::cout << "Primeros caracteres del payload: " << std::string(payload, std::min(message_size, 10U)) << std::endl;
    } else {
        std::cerr << "Error: Payload inválido." << std::endl;
    }

    prof.tick("Publish message");
    // Send the bounding boxes over UDP
    if (message_size > 0) {
        ssize_t sent = sendto(udpSock, payload, message_size, 0,
                                (struct sockaddr*)&clientAddr, clientLen);
        if (sent < 0) {
            perror("[camera_elaboration_UDP] Error sending bounding boxes");
        }
    }
    //printHex(payload, message_size);
    delete[] payload;

    prof.tock("Publish message");
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
                                        static_cast<unsigned int>(boxes[i].rect.x),
                                        static_cast<unsigned int>(boxes[i].rect.y),
                                        static_cast<unsigned int>(boxes[i].rect.width),
                                        static_cast<unsigned int>(boxes[i].rect.height),
                                        static_cast<unsigned int>(*(unsigned int*)&boxes[i].prob),
                                        static_cast<unsigned int>(boxes[i].cl)
                                    );


        buffer += size_of_data ;
    }
    // If there are no boxes we send one empty message with frame info
    if (boxes.size() == 0){
        std::cout << "No hay cajas: preparando mensaje vacio " << std::endl;
        size_t size_of_data = snprintf(buffer,
            CHAR_BOX_SIZE + 1,
            "%02x%02x%02x%02x%02x%04x%016llx",
            static_cast<unsigned int>(true),
            static_cast<unsigned int>(static_cast<unsigned char>(cam_id[0])),
            static_cast<unsigned int>(static_cast<unsigned char>(cam_id[1])),
            static_cast<unsigned int>(static_cast<unsigned char>(cam_id[2])),
            static_cast<unsigned int>(static_cast<unsigned char>(cam_id[3])),
            static_cast<unsigned int>(*frameCounter),
            static_cast<unsigned long long>(*timestamp_acquisition)
        );
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
    YoloV6Engine engine(net, data.frame.cols, data.frame.rows, 0.5);

    // Pinned memory allocation
    void* pHost;
    checkCuda(cudaHostAlloc(&pHost,
                            data.frame.cols * data.frame.rows * 3,
                            cudaHostAllocPortable | cudaHostAllocMapped | cudaHostAllocWriteCombined));
    cv::Mat *frame = new cv::Mat(data.frame.rows, data.frame.cols, CV_8UC3, pHost);



    // Gstreamer Pipeline to send processed frames to SmartCity
    cv::VideoWriter gstreamVideoWriter;

    // Gstreamer Pipeline to send processed frames to SmartCity
    cv::VideoWriter gstreamVideoWriter_local;

    // We'll do it after we've locked down 'frame' size
    int w = data.frame.cols;
    int h = data.frame.rows;
    double fps = 30.0; // TO DO this needs to be dynamic

    if (recordBoxes) {
        // For example, if agx12 is at 192.168.1.12
        std::string pipeline =
                "appsrc ! videoconvert ! video/x-raw,format=NV12 ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 "
                " ! nvv4l2h264enc insert-sps-pps=true iframeinterval=5 idrinterval=5 control-rate=1 bitrate=8000000 "
                " ! h264parse ! rtph264pay config-interval=1 "
                " ! udpsink host=239.255.12.42 port=5001 auto-multicast=true";

        // Open the pipeline
        bool opened = gstreamVideoWriter.open(
            pipeline,
            cv::CAP_GSTREAMER,
            0,       // ignored by GStreamer in "appsrc" mode
            fps,
            cv::Size(w, h), 
            true     // isColor
        );
        
        if (!opened) {
            std::cerr << "[camera_elaboration_UDP] Could not open output GStreamer pipeline.\n";
            // set useGstOutput = false or handle error
        }



        // Video storage
        // To save the video in a video file
        std::string local_pipeline = 
		     "appsrc ! videoconvert ! "
		     "x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 ! "
		     "mp4mux ! filesink location=output_gstream.mp4";

        // Open the local_pipeline
        bool local_opened = gstreamVideoWriter_local.open(
            local_pipeline,
            cv::CAP_GSTREAMER,
            0,       // ignored by GStreamer in "appsrc" mode
            fps,
            cv::Size(w, h), 
            true     // isColor
        );
        
        if (!local_opened) {
            std::cerr << "[camera_elaboration_UDP] Could not open output GStreamer local_pipeline.\n";
            // set useGstOutput = false or handle error
        }

    }


    // Launch Profiler
    edge::Profiler prof(cam->id);

    // Start frame counter (only for this file, not shared)
    unsigned int n_frame=0;

    // print frame id and time stamp in the frames
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 1.0;
    int thickness = 2;
    int baseline = 0;
    cv::Scalar textColor(0, 255, 0); // Green text



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
            prof.tick("Inference Pre");
            engine.preprocess(*frame);
            prof.tock("Inference Pre");
            prof.tick("Inference Infer");
            engine.infer();
            prof.tock("Inference Infer");
            prof.tick("Inference Post");
            const auto boxes = engine.postprocess();
            prof.tock("Inference Post");
            std::cout << "Number of objects detected: " << boxes.size() << std::endl;

            //=============================
            // 2) Send bounding boxes (UDP)
            //=============================
            sendBoundingBoxes(udpSock, clientAddr, clientLen, boxes, frameCounter, cam->id, timestamp_acquisition, prof);

            
            prof.tick("Record Boxes");
            if (recordBoxes){
                // **Add frame ID overlay**
                std::string frameText = "Frame: " + std::to_string(frameCounter) + "TS: " + std::to_string(timestamp_acquisition);
                cv::Size textSize = cv::getTextSize(frameText, fontFace, fontScale, thickness, &baseline);
                cv::Point textOrg(frame->cols - textSize.width - 10, frame->rows - 10);
                cv::putText(*frame, frameText, textOrg, fontFace, fontScale, textColor, thickness);
                
                // Gstream the frame to SmartCity
                if (gstreamVideoWriter.isOpened()) {
                    gstreamVideoWriter.write(*frame);
                }

                // Store the video locally
                if (gstreamVideoWriter_local.isOpened()) {
                    gstreamVideoWriter_local.write(*frame);          // NEW
                }

            }
            prof.tock("Record Boxes");
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
        prof.tock("Total time");
    }



    gRun = false;
    std::cout << "[camera_elaboration_UDP] Elaboration ended for cam " << cam->id << ".\n";



    pthread_join( video_cap, NULL);

    checkCuda( cudaFreeHost(pHost));

    delete frame;


    // close UDP
    if (udpSock >= 0) {
        close(udpSock);
    }




    return (void *)0;
}
