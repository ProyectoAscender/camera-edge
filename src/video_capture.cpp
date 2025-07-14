#include "video_capture.h"
#include <sstream>
#include <string>
#include <ctime>
#include <unistd.h>

std::string get_timestamp()
{
    auto now = std::time(nullptr);
    std::ostringstream os;
    os << std::put_time(std::gmtime(&now),"%F_%T");
    return os.str();
}

void *readVideoCapture( void *ptr )
{
    edge::video_cap_data* data = (edge::video_cap_data*) ptr;

    std::cout<<"\tVC-> Thread: "<<data->input<< " started" <<std::endl;
    std::cout<<"\tVC-> Data->multicast: "<<  data->multicast <<std::endl;
    std::cout<<"\n" <<std::endl;
    std::string pipeline;

    std::string input(data->input); 

    if(data->multicast){ // if multicast
        pipeline =
        "udpsrc port=5000 multicast-group=" + input +
        " auto-multicast=true caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! "
        "rtpjitterbuffer latency=15 drop-on-late=false ! "
        "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink";

    }else{
        pipeline =
        "rtspsrc location=rtsp://" + input + ":554 user-id=user user-pw=Mypassword_7 latency=2000 ! "
        "rtph264depay ! h264parse ! nvv4l2decoder ! "
        "nvvidconv ! video/x-raw\\(memory:NVMM\\), format=NV12 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! "
        "appsink drop=true sync=false";

        // "rtspsrc location=rtsp://192.168.88.251:554  user-id=user user-pw=Mypassword_7 latency=100 ! "
        // "rtph264depay ! h264parse ! nvv4l2decoder nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! "
        // "appsink drop=true max-buffers=1 sync=false";
    //     pipeline =
    // "rtspsrc location=rtsp://user:Mypassword_7@192.168.88.251:554 latency=100 ! "
    // "rtph264depay ! h264parse ! nvv4l2decoder ! nvvidconv ! "
    // "video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! "
    // "appsink drop=true max-buffers=1 sync=false";
    
    }

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    if(!cap.isOpened()){
        std::cout << " -> GSTREAM CAN'T OPEN! " << std::endl;
        gRun.store(false, std::memory_order_release);
    }

    // Wait until cap gets a frame -> AKA waiting for NX11 to be GStreaming frames.
    while (cap.read(data->frame) && data->frame.empty()) {
        usleep(100000); // Wait for 100 ms before retrying
    }

    // const int new_width     = data->width;
    // const int new_height    = data->height;
    cv::Mat frame; //, resized_frame;

    cv::VideoWriter result_video;
    std::ofstream video_timestamp;


    if (record){
        auto now = std::chrono::system_clock::now();
        int w = data->frame.cols;
        int h = data->frame.rows;
        
        // Check if output dir exists, and if not, create it
        std::string output_dir = "../data/output/";
        system(("[ -d " + output_dir + " ] || mkdir -p " + output_dir).c_str());

        // Open files to save output
        result_video.open("../data/output/video_cam_" +
                            data->camId+ "_" + 
                            std::to_string(w)+ "_" + 
                            std::to_string(h)+ "_" + 
                            std::to_string(data->framesToProcess) + "_frames_" + 
                            get_timestamp() + ".mp4", 
                           cv::VideoWriter::fourcc('M','P','4','V'), 30, cv::Size(w, h));
        video_timestamp.open ("timestamp_cam_" + data->camId + ".txt");

    }

    uint64_t timestamp_acquisition = 0;
    unsigned int contador = 0;
    edge::Profiler prof("Video capture" + std::string(data->input));
    std::cout << "\n\nSTARTING VIDEO PROCESSING..." << std::endl;

    while(gRun.load(std::memory_order_acquire)) {
        std::cout << "[camera_elaboration_UDP - YYY] While 1.\n";
        if(!data->frameConsumed) {
            std::cout<<" -> Sleeping. Frame consumed = " << data->frameConsumed << std::endl;
            usleep(500);
            continue;
        }
        // std::cout<<"\nVC-> Frame acquisition\n" << std::endl;
        prof.tick("Frame acquisition");
        std::cout << "[camera_elaboration_UDP - YYY] While 2.\n";
        cap >> frame; 
        std::cout << "[camera_elaboration_UDP - YYY] While 3.\n";
        // Timestamp from the Gstreamer
        double pts_msec = cap.get(cv::CAP_PROP_POS_MSEC);
        // Convert milliseconds to microseconds and safely cast to uint64_t
        uint64_t pts_usec = static_cast<uint64_t>(std::round(pts_msec * 1000));
        
        prof.tock("Frame acquisition");

        if(frame.empty()) {
            std::cout << "[camera_elaboration_UDP - YYY] While 4.\n";
            usleep(1000000); // 1s
            cap.open(data->input);
            std::cerr<<"frame is empty"<<std::endl;
            continue;
        }         

        prof.tick("Frame copy");
        data->mtxF.lock();
        std::cout << "[camera_elaboration_UDP - YYY] While 5.\n";
        data->frame         = frame.clone();
        std::cout << "[camera_elaboration_UDP - YYY] While 6.\n";
        data->tStampMs      = pts_usec;
        data->frameCounter++;                           // Increment the frame counter here
        // std::cout << "\tVC --> Frame count: " << data->frameCounter << std::endl;
        data->frameConsumed = false;
        data->mtxF.unlock();
        prof.tock("Frame copy");
        std::cout << "[camera_elaboration_UDP - YYY] While 7.\n";
        if (record){
            //std::cout << " -> Recording result video" << std::endl;
            result_video << frame;
            video_timestamp << timestamp_acquisition << "\n";
        }

        // prof.printStats();
        std::cout << "[camera_elaboration_UDP - YYY] While 8.\n";
    }
    
    std::cout << " -> video capture ended" << std::endl;

    if(record){
        video_timestamp.close();
        std::cout << " -> releasing video" << std::endl;
        result_video.release();
    }
    
    return (void *)0;
}