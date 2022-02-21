#ifndef DATA_H
#define DATA_H

#include <iostream>
#include <cstring>
#include <iomanip>
#include <fstream>

#include "tkCommon/common.h"
#include "tkCommon/geodetic_conv.h"
#include "tkDNN/DetectionNN.h"
#include "EdgeViewer.h"

#define MAX_CAMERAS 10
#define VERSION_MAJOR 1
#define VERSION_MINOR 1

namespace edge {
struct camera_params
{
    std::string                     input               = "";
    std::string                     resolution          = "";
    std::string                     pmatrixPath         = "";
    std::string                     maskfilePath        = "";
    std::string                     cameraCalibPath     = "";
    std::string                     maskFileOrientPath  = "";
    int                             id                  = 0;
    int                             portCommunicator    = 8888;
    int                             framesToProcess     = -1;
    int                             streamWidth         = 0;
    int                             streamHeight        = 0;
    int                             filterType          = 0;
    bool                            show                = false;    
    bool                            gstreamer           = false;    
};

enum Dataset_t { BDD, COCO, VOC};

struct camera{
    cv::Mat                         prjMat;
    cv::Mat                         invPrjMat;
    cv::Mat                         calibMat;
    cv::Mat                         distCoeff;
    cv::Mat			                precision;
    std::string                     input               = "";
    std::string                     ipCommunicator      = "127.0.0.1";
    tk::dnn::DetectionNN*           detNN               = nullptr;  
    tk::common::GeodeticConverter   geoConv; 
    double*                         adfGeoTransform     = nullptr;
    int                             id                  = 0;
    int                             framesToProcess     = -1;
    int                             portCommunicator    = 8888;
    int                             calibWidth;
    int                             calibHeight;
    int                             streamWidth;
    int                             streamHeight;
    int                             filterType          = 0;
    Dataset_t                       dataset;
    bool                            show                = false;
    bool                            hasCalib            = false;
    bool                            gstreamer           = false;    
};
}

std::ostream& operator<<(std::ostream& os, const edge::camera_params& c);
std::ostream& operator<<(std::ostream& os, const edge::camera& c);

extern edge::EdgeViewer *viewer;
extern bool gRun;
extern bool show;
extern bool verbose;
extern bool record;
extern bool stream;

#endif /*DATA_H*/
