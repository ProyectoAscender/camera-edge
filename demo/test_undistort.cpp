#include "Profiler.h"
//#include "tkDNN/utils.h"
#include "undistort.h"
#include "configuration.h"

int main()
{

    cv::Mat calib_mat, dist_coeff;
    int o_width, o_height;
    readCalibrationMatrix("../data/calib_cameras/20936.params", calib_mat, dist_coeff, o_width, o_height);

    cv::VideoCapture cap("../data/20936.mp4", cv::CAP_FFMPEG);

    const int new_width     = 960;
    const int new_height    = 540;

    cv::Mat frame, resized_frame, undistort_opencv, map1, map2;

    bool gRun = true;
    bool first_iteration = true;

    edge::Profiler prof("opencv");

    cv::namedWindow("undistort", cv::WINDOW_NORMAL);
    cv::namedWindow("distort", cv::WINDOW_NORMAL);

    uint8_t *d_input, *d_output; 
    float *d_map1, *d_map2;

    cv::Mat undistort; 

    while(gRun) {
        prof.tick("Total time");
    
        prof.tick("Frame acquisition");
        cap >> frame;
        if(!frame.data) {
            break;
        }
        prof.tock("Frame acquisition");

        prof.tick("Resize");
        cv::resize (frame, resized_frame, cv::Size(new_width, new_height)); 
        prof.tock("Resize");

        
        if (first_iteration){

            std::cout<< calib_mat<<std::endl;

            calib_mat.at<double>(0,0)*=  double(new_width) / double(o_width);
            calib_mat.at<double>(0,2)*=  double(new_width) / double(o_width);
            calib_mat.at<double>(1,1)*=  double(new_width) / double(o_width);
            calib_mat.at<double>(1,2)*=  double(new_width) / double(o_width);

            
            std::cout<< calib_mat<<std::endl;

            cv::initUndistortRectifyMap(calib_mat, dist_coeff, cv::Mat(), calib_mat, resized_frame.size(), CV_32F, map1, map2);
            first_iteration = false;
            std::cout<<map1.size()<<std::endl;

            checkCuda( cudaMalloc(&d_input, resized_frame.cols*resized_frame.rows*resized_frame.channels()*sizeof(uint8_t)) );
            checkCuda( cudaMalloc(&d_output, resized_frame.cols*resized_frame.rows*resized_frame.channels()*sizeof(uint8_t)) );
            checkCuda( cudaMalloc(&d_map1, map1.cols*map1.rows*map1.channels()*sizeof(float)) );
            checkCuda( cudaMalloc(&d_map2, map2.cols*map2.rows*map2.channels()*sizeof(float)) );

            checkCuda( cudaMemcpy(d_map1, (float*)map1.data,  map1.cols*map1.rows*map1.channels()*sizeof(float), cudaMemcpyHostToDevice));
            checkCuda( cudaMemcpy(d_map2, (float*)map2.data,  map2.cols*map2.rows*map2.channels()*sizeof(float), cudaMemcpyHostToDevice));

            undistort = resized_frame.clone();
            
        }

        prof.tick("Remap Fabio");
        checkCuda( cudaMemcpy(d_input, (uint8_t*)resized_frame.data,  resized_frame.cols*resized_frame.rows*resized_frame.channels()*sizeof(uint8_t), cudaMemcpyHostToDevice));
        remap(d_input, new_width, new_height, 3, d_map1, d_map2, d_output, new_width , new_height, 3);
        checkCuda( cudaMemcpy((uint8_t*)undistort.data , d_output, resized_frame.cols*resized_frame.rows*resized_frame.channels()*sizeof(uint8_t), cudaMemcpyDeviceToHost));
        prof.tock("Remap Fabio");

        prof.tick("Remap opencv");
        cv::remap(resized_frame, undistort_opencv, map1, map2, cv::INTER_CUBIC);
        prof.tock("Remap opencv");


        prof.tock("Total time");
        prof.printStats(5);

        cv::imshow("distort", resized_frame);
        cv::imshow("undistort", undistort);
        
        cv::waitKey(1);
    }

    return 0;
}