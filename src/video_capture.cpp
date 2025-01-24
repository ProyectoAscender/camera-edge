#include "video_capture.h"
#include <sstream>
#include <string>
#include <ctime>

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
    auto stream_mode = data->gstreamer ? cv::CAP_GSTREAMER : cv::CAP_FFMPEG;

    std::cout<<"\tVC-> Thread: "<<data->input<< " started" <<std::endl;
    std::cout<<"\tVC-> Data->gstreamer: "<<  data->gstreamer <<std::endl;
    std::cout<<"\tVC-> Stream_mode: "<<stream_mode <<std::endl;
    std::cout<<"\n" <<std::endl;

    cv::VideoCapture cap("udpsrc port=5000 multicast-group=" + std::string(data->input) +
                         " auto-multicast=true caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink",
                         stream_mode);


    if(!cap.isOpened()){
        std::cout << " -> CASO 1 " << std::endl;
        gRun = false; 
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

    while(gRun) {

        if(!data->frameConsumed) {
            // std::cout<<" -> Sleeping. Frame consumed = " << data->frameConsumed << std::endl;
            usleep(500);
            continue;
        }
        // std::cout<<"\nVC-> Frame acquisition\n" << std::endl;
        prof.tick("Frame acquisition");
        cap >> frame; 
        timestamp_acquisition = getTimeMs();
        prof.tock("Frame acquisition");

        if(frame.empty()) {
            usleep(1000000); // 1s
            cap.open(data->input);
            std::cerr<<"frame is empty"<<std::endl;
            continue;
        }         

        prof.tick("Frame copy");
        data->mtxF.lock();
        data->frame         = frame.clone();
        data->tStampMs      = timestamp_acquisition;
        data->frameCounter++;                           // Increment the frame counter here
        // std::cout << "\tVC --> Frame count: " << data->frameCounter << std::endl;
        data->frameConsumed = false;
        data->mtxF.unlock();
        prof.tock("Frame copy");

        if (record){
            //std::cout << " -> Recording result video" << std::endl;
            result_video << frame;
            video_timestamp << timestamp_acquisition << "\n";
        }

        // prof.printStats();
    }
    
    std::cout << " -> video capture ended" << std::endl;

    if(record){
        video_timestamp.close();
        std::cout << " -> releasing video" << std::endl;
        result_video.release();
    }
    
    return (void *)0;
}
