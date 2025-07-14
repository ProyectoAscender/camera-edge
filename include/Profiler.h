#ifndef PROFILER_H
#define PROFILER_H

#include <iostream>
#include <algorithm>
#include <vector>
#include <map>
#include <iomanip>
#include <chrono>
#include "utils.h"


//#include "tkDNN/utils.h"

#define MAX_MIN 9999999

namespace edge{

struct stats{
    std::chrono::time_point<std::chrono::system_clock> start;
    std::vector<double> diff;
    unsigned long int count = 0; //total count
    double oMin = MAX_MIN;       //overall max
    double oMax = 0;             //overall min
    double oAvg = 0;             //overall average
};

class Profiler{
    std::string name;
    std::map<std::string,stats> timers;
    
public:
    Profiler(std::string profiler_name)  {
        name = profiler_name;
    }
    
    ~Profiler() {}
    void tick(std::string timer_name){
        std::cout << "[camera_elaboration_UDP - ZZZ] Ticking...\n";
        timers[timer_name].start = std::chrono::high_resolution_clock::now();
        std::cout << "[camera_elaboration_UDP - ZZZ] Ticking 2...\n";
    }

    void tock(std::string timer_name){
        if ( timers.find(timer_name) == timers.end() ) 
            FatalError("Timer never started (no tick associated)");
        auto end = std::chrono::high_resolution_clock::now();
        double diff = std::chrono::duration_cast<std::chrono::microseconds>(end-timers[timer_name].start).count();
        timers[timer_name].diff.push_back(diff);
    }

    void printStats(int interval = 20){
        bool print = false;
        for (auto& t : timers)
            if (t.second.diff.size() == interval) {
                print = true;
                break;
            }

        if(print){
            int max_lenght = 0;
            for (const auto& t : timers)
                if(t.first.size() > max_lenght)
                    max_lenght = t.first.size();
            max_lenght += 10;
            
            std::cout<<"######################### Profiler "<< name << " [ "<< interval << " iterations ] #########################"<<std::endl;
            
            double cur_sum, cur_min, cur_max;
            for (auto& t : timers)
            {
                cur_sum = 0;
                cur_max = 0;
                cur_min = MAX_MIN;

                for(const auto& d: t.second.diff){
                    t.second.oMin = (d < t.second.oMin) ? d : t.second.oMin;
                    t.second.oMax = (d > t.second.oMax) ? d : t.second.oMax;
                    t.second.count++;

                    cur_min  = (d < cur_min)  ? d : cur_min;
                    cur_max  = (d > cur_max)  ? d : cur_max;
                    cur_sum  += d;
                }
                t.second.oAvg = t.second.oAvg * double((t.second.count - t.second.diff.size()))/t.second.count + cur_sum / t.second.count;

                std::cout << t.first << std::fixed  << std::setprecision(2)
                        << std::setfill(' ')        << std::setw (max_lenght - t.first.size())
                        << "\t\tavg(ms): "          << cur_sum / double(t.second.diff.size()) / 1000
                        << "\tmin(ms): "            << cur_min / 1000
                        << "\tmax(ms): "            << cur_max / 1000
                        << "\toverall avg(ms): "    << t.second.oAvg / 1000
                        << "\toverall min(ms): "    << t.second.oMin / 1000
                        << "\toverall max(ms): "    << t.second.oMax / 1000
                        << std::endl ;
                t.second.diff.clear();
            }
        }
    }
};

}
#endif /*PROFILER_H*/
