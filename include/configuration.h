#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <iostream>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>

#include <yaml-cpp/yaml.h>

#include <gdal/gdal.h>
#include <gdal_priv.h>

#include "tkDNN/utils.h"
#include "tkDNN/CenternetDetection.h"
#include "tkDNN/MobilenetDetection.h"
#include "tkDNN/Yolo3Detection.h"

#include "data.h"

std::string executeCommandAndGetOutput(const char * command);
std::string decryptString(std::string encrypted, const std::string& password);
std::string encryptString(std::string to_encrypt, const std::string& password);

void readParamsFromYaml(const std::string& params_path, const std::vector<std::string>& cameras_ids,std::vector<edge::camera_params>& cameras_par,std::string& net, char& type, int& n_classes, std::string& tif_map_path);
bool readParameters(int argc, char **argv,std:: vector<edge::camera_params>& cameras_par,std::string& net, char& type, int& n_classes,std::string& tif_map_path);

// void initializeCamerasNetworks(std:: vector<edge::camera>& cameras, const std::string& net, const char type, int& n_classes);

void readProjectionMatrix(const std::string& path, cv::Mat& prj_mat);
void readCalibrationMatrix(const std::string& path, cv::Mat& calib_mat, cv::Mat& dist_coeff, int& image_width, int& image_height);
void readTiff(const std::string& path, double *adfGeoTransform);

std::vector<edge::camera> configure(int argc, char **argv);

#endif /*CONFIGURATION_H*/