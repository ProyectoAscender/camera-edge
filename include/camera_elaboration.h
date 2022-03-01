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
void convertCameraPixelsToMapMeters(const int x, const int y, const int cl, edge::camera& cam, double& north, double& east);
std::vector<edge::tracker_line> getTrackingLines(const tracking::Tracking& t, edge::camera& cam,const float scale_x=1, const float scale_y=1, bool verbose=false);

void prepareMessage(const tracking::Tracking& t, MasaMessage& message,tk::common::GeodeticConverter& geoConv, 
                    const int cam_id, edge::Dataset_t dataset, uint64_t t_stamp_acquisition_ms);

char* prepareMessageUDP(std::vector<tk::dnn::box> &box_vector, std::vector<std::tuple<double, double>> &coords,
                     // std::vector<std::tuple<double, double>> &coordsGeo,
                     std::vector<std::tuple<double, double, double, double, double, double, double, double>> &boxCoords,
                     unsigned int frameAmount, int cam_id, double lat_init, double lon_init, unsigned int *size, float scale_x, float scale_y);
void sendUDPMessage(int sockfd,int n_frame, char* data, unsigned int *size );
void collectBoxInfo( std::vector<std::vector<tk::dnn::box>>& batchDetected,
                std::vector<tk::dnn::box>& box_vector,std::vector<std::tuple<double, double>>& coords,
                std::vector<std::tuple<double, double>>& coordsGeo, 
                std::vector<std::tuple<double, double, double, double, double, double, double, double>>& boxCoords,
                float& scale_x, float& scale_y, edge::camera& camera);

void *elaborateSingleCamera(void *ptr);

#endif /*CAMERAELABORATION_H*/