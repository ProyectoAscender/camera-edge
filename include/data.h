#ifndef DATA_H
#define DATA_H

#include <iostream>
#include <cstring>

#include "tkDNN/DetectionNN.h"
#include "EdgeViewer.h"

#define MAX_CAMERAS 10

namespace edge {
struct camera_params
{
    std::string input;
    std::string pmatrixPath;
    std::string maskfilePath;
    std::string cameraCalibPath;
    std::string maskFileOrientPath;
    int         id = 0;
    bool        show = false;
};

struct camera{
    cv::Mat                 prjMat;
    cv::Mat                 calibMat;
    cv::Mat                 distCoeff;
    std::string             input;
    tk::dnn::DetectionNN*   detNN;  
    int                     id = 0;
    bool                    show = false;
};
}

std::ostream& operator<<(std::ostream& os, const edge::camera_params& c);

std::ostream& operator<<(std::ostream& os, const edge::camera& c);

extern EdgeViewer *viewer;
extern bool gRun;
extern double *adfGeoTransform;


#endif /*DATA_H*/