#ifndef CAMERAELABORATION_H
#define CAMERAELABORATION_H

#include <pthread.h>
#include <vector> 
#include <fstream>
#include <iostream>


#include"video_capture.h"
#include"message.h"
#include "YoloV6Engine.hpp"


// #include "tkDNN/DetectionNN.h"


void pixel2GPS(const int x, const int y, double &lat, double &lon, double* adfGeoTransform);
void GPS2pixel(double lat, double lon, int &x, int &y, double* adfGeoTransform);

void printBufferHex(const char* buffer, size_t size);

char* prepareMessageUDP2(const std::vector<Box> &boxes, unsigned int n_frame, std::string cam_id);
void *elaborateSingleCamera(void *ptr);

#endif /*CAMERAELABORATION_H*/