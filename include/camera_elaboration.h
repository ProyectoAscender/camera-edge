#ifndef CAMERAELABORATION_H
#define CAMERAELABORATION_H

#include <pthread.h>
#include <vector> 
#include <fstream>
#include <iostream>


#include"video_capture.h"
#include"message.h"
#include "YoloV8Engine.hpp"


// #include "tkDNN/DetectionNN.h"


void pixel2GPS(const int x, const int y, double &lat, double &lon, double* adfGeoTransform);
void GPS2pixel(double lat, double lon, int &x, int &y, double* adfGeoTransform);

void printBufferHex(const char* buffer, size_t size);

char* prepareMessageUDP2(const std::vector<Box> &boxes, unsigned int n_frame, std::string cam_id, 
                         float scale_x, float scale_y);
// void collectBoxInfo( std::vector<std::vector<tk::dnn::box>>& batchDetected,
//                 std::vector<tk::dnn::box>& box_vector,std::vector<std::tuple<double, double>>& coords,
//                 std::vector<std::tuple<double, double>>& coordsGeo, 
//                 std::vector<std::tuple<double, double, double, double, double, double, double, double>>& boxCoords,
//                 float& scale_x, float& scale_y, edge::camera& camera);

void *elaborateSingleCamera(void *ptr);

#endif /*CAMERAELABORATION_H*/