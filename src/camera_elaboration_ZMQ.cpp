#include "camera_elaboration.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include "Profiler.h"

// Include ZeroMQ
#include "zmq.hpp"

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



void handshake(zmq::socket_t &rep_socket, edge::video_cap_data &data, int bufferSize) {
    // Wait for 1 request from Python, respond with camera info
    std::cout << "[elaborateSingleCamera] Waiting for handshake request...\n";
    zmq::message_t request;
    rep_socket.recv(request, zmq::recv_flags::none);
    std::cout << "[elaborateSingleCamera] Received handshake request.\n";


    // Wait until data.frame has something -> AKA waiting for NX11 to be GStreaming frames.
    while (data.frame.empty()) {
        std::cout << "[elaborateSingleCamera] Waiting for frames from NX11...\n";
        usleep(1000000); // 1 s
    }

    // At this point, camera is streaming frames
    std::cout << "- - - - -> Camera " << data.camId << " is ready. " << std::endl;

    // Build the camera info in your pipe-delimited style
    // "Sent ZMQ:|cam->id|cam->multicast|framesToProcess|height|width|dataPath"

    // Data to send in handshake
    char infoBuffer[bufferSize];
    snprintf(infoBuffer, bufferSize, "%s|%s|%d|%d|%d|%d|%s",
                "Sent  UDP: ",
                data.camId.c_str(), data.multicast, data.framesToProcess, 
                data.frame.rows, data.frame.cols, data.dataPath.c_str());

    // Send the camera info back
    zmq::message_t reply(strlen(infoBuffer));
    memcpy(reply.data(), infoBuffer, strlen(infoBuffer));
    rep_socket.send(reply, zmq::send_flags::none);
    std::cout << "[elaborateSingleCamera] Replied with camera info: " << infoBuffer << "\n";

}

void publishBoundingBoxes(zmq::socket_t &publisher, const std::vector<Box> &boxes, unsigned int frameCounter, const std::string &camId, uint64_t timestampAcquisition, edge::Profiler &prof) {
    prof.tick("Prepare message"); 
    unsigned int message_size;
    char* payload = prepareMessage(boxes, &frameCounter, camId, &message_size, &timestampAcquisition);
    prof.tock("Prepare message");

    prof.tick("Publish message");
    // Publish over ZeroMQ: topic = cam->id, payload = ASCII-hex
    zmq::message_t topic_msg(camId.begin(), camId.end());
    zmq::message_t data_msg(payload, message_size);

    // We send a two-part message: [topic, payload]
    try {
        publisher.send(topic_msg, zmq::send_flags::sndmore);
        publisher.send(data_msg, zmq::send_flags::none);
    } catch(const zmq::error_t& e) {
        std::cerr << "[elaborateSingleCamera] Publish error: " << e.what() << std::endl;
    }

    // Memory cleaning
    if (payload) delete[] payload;

    prof.tock("Publish message");
}




char* prepareMessage( const std::vector<Box> &boxes, unsigned int *frameCounter, std::string cam_id, unsigned int *message_size, uint64_t *timestamp_acquisition) {
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

void *elaborateSingleCamera_ZMQ(void *ptr)
{
    edge::camera* cam = (edge::camera*) ptr;
    std::cout<<"[elaborateSingleCamera] STARTING CAMERA_ELABORATION_ZMQ..." << std::endl;
    std::cout<<"[elaborateSingleCamera] Starting camera: "<< cam->id << std::endl;

    // Create a thread for video capture
    pthread_t video_cap;
    edge::video_cap_data data;
    data.input              = (char*)cam->input.c_str();
    data.camId              = cam->id;
    data.frameConsumed      = true;
    data.framesToProcess    = cam->framesToProcess;
    data.multicast          = cam->multicast;
    data.dataPath           = cam->dataPath;




    if (pthread_create(&video_cap, NULL, readVideoCapture, (void*)&data)){
        std::cerr << "[elaborateSingleCamera] Error creating capture thread\n";
        return (void*)1;
    }
    std::cout << "[elaborateSingleCamera] Video capture launched.\n";


    // ZeroMQ sockets
    // We'll do the handshake on port X, and bounding boxes on port X+1
    int handshakePort = cam->portCommunicator;
    int pubPort       = cam->portCommunicator + 1;
    std::string handshakeAddr = "tcp://0.0.0.0:" + std::to_string(handshakePort);
    std::string pubBindAddr   = "tcp://0.0.0.0:" + std::to_string(pubPort);

    zmq::context_t context(1);
    zmq::socket_t rep_socket(context, zmq::socket_type::rep);
    zmq::socket_t publisher(context, zmq::socket_type::pub);



    //=============================
    // 1) HANDSHAKE (REP)
    //=============================

    // Message variables
    int bufferSize = 70;

    try {
        rep_socket.bind(handshakeAddr);
        std::cout << "[elaborateSingleCamera] REP handshake bound at " << handshakeAddr << "\n";
    } catch (const zmq::error_t& e) {
        std::cerr << "[elaborateSingleCamera] Error binding REP socket: " << e.what() << std::endl;
        return (void*)1;
    }

    handshake(rep_socket, data, bufferSize);
    
    // One-time handshake done, close the REP socket
    rep_socket.close();
    std::cout << "[elaborateSingleCamera] Handshake done.\n";




    //=============================
    // 2) PUBLISH bounding boxes (PUB)
    //=============================
    try {
        publisher.bind(pubBindAddr);
        std::cout << "[elaborateSingleCamera] PUB bound at: " << pubBindAddr << std::endl;
    } catch (const zmq::error_t& e) {
        std::cerr << "[elaborateSingleCamera] Error binding PUB socket: " << e.what() << std::endl;
        return (void*)1;
    }




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
    while(gRun.load(std::memory_order_acquire)){
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
            // 2) PUBLISH bounding boxes (PUB)
            //=============================

            publishBoundingBoxes(publisher, boxes, frameCounter, cam->id, timestamp_acquisition, prof);



            prof.tock("Total time");

               
            if (verbose) 
                prof.printStats();  



        } // if -> image empty
        else {
            // no new frame
            usleep(500);
        }
        

        if (n_frame >= data.framesToProcess ) {
            std::cout << "[elaborateSingleCamera] " << data.camId
                      << " done after " << n_frame << " frames.\n";
            break;
        }
    }



    gRun.store(false, std::memory_order_release);
    std::cout << "[elaborateSingleCamera] Elaboration ended for cam " << cam->id << ".\n";

    if (recordBoxes) {
        boxes_video.release();
    }

    pthread_join( video_cap, NULL);

    checkCuda( cudaFreeHost(pHost));

    delete frame;

    // Close ZeroMQ
    publisher.close();
    context.close();

    return (void *)0;
}
