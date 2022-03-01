#include "video_capture.h"

void *readVideoCapture( void *ptr )
{
    edge::video_cap_data* data = (edge::video_cap_data*) ptr;
    
    std::cout<<"Thread: "<<data->input<< " started" <<std::endl;

    auto stream_mode = data->gstreamer ? cv::CAP_GSTREAMER : cv::CAP_FFMPEG;
    cv::VideoCapture cap(data->input, stream_mode);
    if(!cap.isOpened())
        gRun = false; 
    else
        std::cout<<"Camera correctly started.\n";

    const int new_width     = data->width;
    const int new_height    = data->height;
    cv::Mat frame, resized_frame;

    cv::VideoWriter result_video;
    std::ofstream video_timestamp;
    if (record){
        int w = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        int h = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        result_video.open("video_cam_"+std::to_string(data->camId)+".mp4", cv::VideoWriter::fourcc('M','P','4','V'), 30, cv::Size(w, h));
        video_timestamp.open ("timestamp_cam_"+std::to_string(data->camId)+".txt");
    }

    uint64_t timestamp_acquisition = 0;
    edge::Profiler prof("Video capture" + std::string(data->input));
    while(gRun) {
        if(!stream && !data->frameConsumed) {
            usleep(1000);
            continue;
        }
        prof.tick("Frame acquisition");
        cap >> frame; 
        timestamp_acquisition = getTimeMs();
        prof.tock("Frame acquisition");
        if(!frame.data) {
            usleep(1000000); //us
            cap.open(data->input);
            printf("cap reinitialize\n");
            break;
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
            result_video << frame;
            video_timestamp << timestamp_acquisition << "\n";
        }

        // prof.printStats();
    }

    if(record){
        video_timestamp.close();
        result_video.release();
    }
    
    return (void *)0;
}
