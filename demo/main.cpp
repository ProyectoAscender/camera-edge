#include "configuration.h"
#include <yaml-cpp/yaml.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <csignal>
#include <iostream>
#include "camera_elaboration.h"
#include "data.h"

void sig_handler(int signo) {
    std::cout<<"request stop\n";
    gRun = false;
}

int main(int argc, char **argv)
{
    // tk::exceptions::handleSegfault();
    std::signal(SIGINT, sig_handler);    
    std::signal(SIGTERM, sig_handler);

    std::vector<edge::camera> cameras = configure(argc, argv);
    if(cameras.empty()){
        std::cout<<"No (correct) camera IDS specified as input. Closing."<<std::endl;
        return EXIT_SUCCESS;
    }

    if(show){
        //TODO allow to switch on and off viewer with signals
        // viewer = new edge::EdgeViewer(cameras.size());
        // viewer->setWindowName("Cameras");
        // viewer->setBackground(tk::gui::color::DARK_GRAY);
        // // viewer->setClassesNames(cameras[0].detNN->classesNames);
        // // viewer->setColors(cameras[0].detNN->classes);
        // viewer->initOnThread();    
    }

    pthread_t threads[MAX_CAMERAS];
    int iret[MAX_CAMERAS];
    for(size_t i=0; i<cameras.size(); ++i)
        iret[i] = pthread_create( &threads[i], NULL, elaborateSingleCamera, (void*) &cameras[i]);


    for(size_t i=0; i<cameras.size(); ++i)
        pthread_join( threads[i], NULL);

    for(size_t i=0; i<cameras.size(); ++i)
        std::cout << "Thread " << i << " returns: " << iret[i] << std::endl;

    return EXIT_SUCCESS;
}
