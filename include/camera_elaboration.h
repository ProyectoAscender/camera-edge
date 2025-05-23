#ifndef CAMERAELABORATION_H
#define CAMERAELABORATION_H

#include <pthread.h>
#include <vector> 
#include <fstream>
#include <iostream>


#include"video_capture.h"
#include "YoloV6Engine.hpp"


// #define USE_ZMQ // Comment this line to use Version with UDP



// #define CHAR_BOX_SIZE 14
// #define CHAR_BOX_SIZE 46
// #define CHAR_BOX_SIZE 58
#define CHAR_BOX_SIZE 64


void printHex(const char* data, size_t length);
void pixel2GPS(const int x, const int y, double &lat, double &lon, double* adfGeoTransform);
void GPS2pixel(double lat, double lon, int &x, int &y, double* adfGeoTransform);

void printBufferHex(const char* buffer, size_t size);

char* prepareMessage(const std::vector<Box> &boxes, unsigned int *frameCounter, std::string cam_id, unsigned int *message_size, uint64_t *timestamp_acquisition);


// Conditional declaration of `elaborateSingleCamera`
#ifdef USE_ZMQ
    void *elaborateSingleCamera_ZMQ(void *ptr);
    #define elaborateSingleCamera elaborateSingleCamera_ZMQ
#else
    void *elaborateSingleCamera_UDP(void *ptr);
    #define elaborateSingleCamera elaborateSingleCamera_UDP
    
#endif



#endif /*CAMERAELABORATION_H*/