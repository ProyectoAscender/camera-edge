#include "message.h"
#include "utils.h"


uint8_t orientationToUint8(const float yaw){
    uint8_t orientation = uint8_t((int((yaw * 57.29 + 360)) % 360) * 17 / 24);
    return orientation;
}

uint8_t velocityToUint8(const float vel){
    uint8_t velocity = uint8_t(std::abs(vel * 3.6 * 2));
    return velocity;
} 

bool checkClass(const int cl, const edge::Dataset_t dataset){
    
    switch(dataset){
        case edge::Dataset_t::BDD:
            if(cl == 0 || cl == 1 || cl == 2 || cl == 3 || cl == 4 || cl == 5)
                return true;
            break;
        case edge::Dataset_t::COCO:
            if(cl == 0 || cl == 1 || cl == 2 || cl == 3 || cl == 5 || cl == 7)
                return true;
            break;
        default: 
        std::cout<<dataset<<std::endl;
            FatalError("Dataset not supported yet.");
    }
    return false;
}

Categories classToCategory(const int cl, const edge::Dataset_t dataset){
    Categories cat;
    if(dataset == edge::Dataset_t::BDD)
        switch (cl){
        case 0: cat = Categories::C_person;     break;
        case 1: cat = Categories::C_car;        break;
        case 2: cat = Categories::C_car;        break;
        case 3: cat = Categories::C_bus;        break;
        case 4: cat = Categories::C_motorbike;  break;
        case 5: cat = Categories::C_bycicle;    break;
        }
    else if (dataset == edge::Dataset_t::COCO)
        switch (cl){
        case 0: cat = Categories::C_person;     break;
        case 1: cat = Categories::C_bycicle;    break;
        case 2: cat = Categories::C_car;        break;
        case 3: cat = Categories::C_motorbike;  break;
        case 5: cat = Categories::C_bus;        break;
        case 7: cat = Categories::C_car;        break;
        }
    else
        FatalError("Dataset not supported yet.");
    return cat;
}
std::ostream& operator<<(std::ostream& os, const RoadUser& o){
    os<<std::setprecision(20);
    os<<"----------------------------------------------------\n";
    os<< "camera ids\t";
    for(int id : o.camera_id)
        os<< id << " ";
    os << std::endl;
    os<< "latitude\t"       << o.latitude           << std::endl;
    os<< "longitude\t"      << o.longitude          << std::endl;
    os<< "speed\t\t"        << int(o.speed)         << std::endl;
    os<< "precision\t"      << o.precision          << std::endl;
    os<< "orientation\t"    << int(o.orientation)   << std::endl;
    os<< "category\t"       << o.category           << std::endl;
    os<<"----------------------------------------------------\n";
    return os;
}

RoadUser getRoadUser(const std::vector<int> camera_id, const double latitude, const double longitude, const std::vector<int> object_id, const float velocity, const float orientation, const float precision, const int cl , edge::Dataset_t dataset){
    RoadUser r;
    r.camera_id = camera_id;
    r.latitude = static_cast<float>(latitude);
    r.longitude = static_cast<float>(longitude);
    r.object_id = object_id;
    r.speed = velocityToUint8(velocity);
    r.orientation = orientationToUint8(orientation);
    r.precision = precision;
    r.category = classToCategory(cl, dataset);
    return r;
}

unsigned long long getTimeMs() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    unsigned long long t_stamp_ms = (unsigned long long)(tv.tv_sec) * 1000 + (unsigned long long)(tv.tv_usec) / 1000;
    return t_stamp_ms;
}
