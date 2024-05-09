#ifndef CAMERAELABORATION_H
#define CAMERAELABORATION_H

#include <pthread.h>
#include <vector> 
#include <fstream>
#include <iostream>

#include "Tracking.h"

#include"video_capture.h"
#include"message.h"

#include "tkDNN/DetectionNN.h"


void pixel2GPS(const int x, const int y, double &lat, double &lon, double* adfGeoTransform);
void GPS2pixel(double lat, double lon, int &x, int &y, double* adfGeoTransform);

void convertCameraPixelsToGeodetic(const int x, const int y, const int cl, edge::camera& cam, double& lat, double& lon);
void convertCameraPixelsToMapMeters(const int x, const int y, const int cl, edge::camera& cam, double& east, double& north);
std::vector<edge::tracker_line> getTrackingLines(const tracking::Tracking& t, edge::camera& cam,const float scale_x=1, const float scale_y=1, bool verbose=false);

char* prepareMessageUDP(std::vector<tk::dnn::box> &box_vector, std::vector<std::tuple<double, double>> &coords,
                     // std::vector<std::tuple<double, double>> &coordsGeo,
                     std::vector<std::tuple<double, double, double, double, double, double, double, double>> &boxCoords,
                     unsigned int frameAmount, int cam_id, double lat_init, double lon_init, unsigned int *size, float scale_x, float scale_y);
void printBufferHex(const char* buffer, size_t size);

char* prepareMessageUDP2(std::vector<tk::dnn::box> &box_vector, unsigned int n_frame, std::string cam_id, 
                         float scale_x, float scale_y);
void collectBoxInfo( std::vector<std::vector<tk::dnn::box>>& batchDetected,
                std::vector<tk::dnn::box>& box_vector,std::vector<std::tuple<double, double>>& coords,
                std::vector<std::tuple<double, double>>& coordsGeo, 
                std::vector<std::tuple<double, double, double, double, double, double, double, double>>& boxCoords,
                float& scale_x, float& scale_y, edge::camera& camera);

void *elaborateSingleCamera(void *ptr);

#endif /*CAMERAELABORATION_H*/