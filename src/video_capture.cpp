#include "video_capture.h"
#include <sstream>
#include <string>
#include <ctime>

std::string get_timestamp()
{
    auto now = std::time(nullptr);
    std::ostringstream os;
    os << std::put_time(std::gmtime(&now),"%F%T");
    return os.str();
}

void *readVideoCapture( void *ptr )
{
    edge::video_cap_data* data = (edge::video_cap_data*) ptr;
    
    std::cout<<"Thread: "<<data->input<< " started" <<std::endl;

    auto stream_mode = data->gstreamer ? cv::CAP_GSTREAMER : cv::CAP_FFMPEG;
    std::cout<<" data->gstreamer: "<<  data->gstreamer <<std::endl;
    std::cout<<"stream_mode: "<<stream_mode <<std::endl;
    cv::VideoCapture cap(data->input, stream_mode);
    std::cout<<"openCV RIGHT!: "<<stream_mode <<std::endl;
    if(!cap.isOpened())
        gRun = false; 

    const int new_width     = data->width;
    const int new_height    = data->height;
    cv::Mat frame, resized_frame;

    cv::VideoWriter result_video;
    std::ofstream video_timestamp;
    if (record){
        auto now = std::chrono::system_clock::now();
        int w = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        int h = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        result_video.open("../data/output/video_cam_" +
                            std::to_string(data->camId)+ "_" + 
                            std::to_string(w)+ "_" + 
                            std::to_string(h)+ "_" + 
                            std::to_string(data->framesToProcess) + "_frames_" + 
                            get_timestamp() + ".mp4", 
                           cv::VideoWriter::fourcc('M','P','4','V'), 30, cv::Size(w, h));
        video_timestamp.open ("timestamp_cam_"+std::to_string(data->camId)+".txt");
    }

    uint64_t timestamp_acquisition = 0;
    unsigned int contador = 0;
    edge::Profiler prof("Video capture" + std::string(data->input));

    while(gRun) {
        if(!data->frameConsumed) {
            //std::cout<<" -> Sleeping. Frame consumed = " << data->frameConsumed << std::endl;
            usleep(500);
            continue;
        }
        prof.tick("Frame acquisition");
        cap >> frame; 
        timestamp_acquisition = getTimeMs();
        prof.tock("Frame acquisition");
        if(frame.empty()){
            std::cerr<<"frame is empty"<<std::endl;
        }
        if(frame.empty()) {
            usleep(1000000); //us
            cap.open(data->input);
            printf("cap reinitialize\n");
            continue;
        } 
        //resizing the image to 960x540 (the DNN takes in input 544x320)
        prof.tick("Frame resize");
        cv::resize (frame, resized_frame, cv::Size(new_width, new_height)); 
         
        prof.tock("Frame resize");

        prof.tick("Frame copy");
        data->mtxF.lock();
        data->frame         = resized_frame.clone();
        data->tStampMs      = timestamp_acquisition;
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
