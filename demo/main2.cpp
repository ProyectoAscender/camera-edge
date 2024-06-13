#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>

#include "YoloV8Engine.hpp"

const std::string VIDEO_FILE = ".mp4";
const std::string ENGINE_FILE = ".engine";
const float CONF_THRESH = 0.3;

int main(int argc, char *argv[], char *envp[]) {
    cv::VideoCapture cap(VIDEO_FILE);

    if (!cap.isOpened()) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    // Read one frame to set width and height of the engine
    cv::Mat frame1;
    cap >> frame1;

    YoloV8Engine engine(ENGINE_FILE, frame1.cols, frame1.rows, CONF_THRESH);

    while (true) {
        cv::Mat frame;
        // Capture frame-by-frame
        cap >> frame;

        // If the frame is empty, break immediately
        if (frame.empty())
            break;

        engine.preprocess(frame);
        engine.infer();
        const auto boxes = engine.postprocess();

        printf("Number of objects detected: %ld\n", boxes.size());
    }

    return 0;
}