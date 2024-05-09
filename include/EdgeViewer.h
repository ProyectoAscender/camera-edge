#ifndef EDGEVIEWER_H
#define EDGEVIEWER_H

#include "tkCommon/gui/Viewer.h"
#include <map>

namespace edge{

struct tracker_line{
    std::vector<tk::common::Vector3<float>> points;
    tk::gui::Color_t                        color;
};

struct camera_data{
    std::vector<tk::dnn::box> detected;        
    std::vector<tracker_line> lines;
    cv::Mat frame;
    std::string id = 0;
    int showId = -1;
    bool *showCamera;
    bool newFrame = false;
    std::mutex *mtxNewFrame;

    GLuint frameTexture;
        
    int frame_width, frame_height;
    float xScale, yScale;
    float aspectRatio;

    tk::common::Vector3<float> pose;
    tk::common::Vector3<float> size;
};

#define MAX_CAMERA_VIZ 6

class EdgeViewer : public tk::gui::Viewer {
    private:
        
        std::vector<std::string> classesNames;
        std::vector<tk::gui::Color_t> colors;

        int shCameras   = 0; //showing cameras
        int avCameras   = 1; //avaiable cameras
        int cameraIndex = 0;
        std::mutex mtxIndex;

        std::map<int,int> idIndexBind;

        std::vector<camera_data> cameraData;

        bool open = true; //for imgui

        void settkViewport2D(const int index){
            int viewport_h = height;
            int viewport_w = width;
            int viewport_x = 0;
            int viewport_y = 0;
            if(shCameras == 2)
            {
                viewport_h /= 2;
                viewport_y = viewport_h*index;
            }
            else if(shCameras == 3 || shCameras == 4){
                viewport_w /= 2;
                viewport_h /= 2;
                viewport_x = viewport_w*(index/2);
                viewport_y = viewport_h*(index%2);
            }
            else if(shCameras == 5 || shCameras == 6){
                viewport_w /= 2;
                viewport_h /= 3;
                viewport_x = viewport_w*(index/3);
                viewport_y = viewport_h*(index%3);
            }

            tkViewport2D(viewport_w, viewport_h, viewport_x , viewport_y);
        }



        
    public:
        EdgeViewer(int n_cameras) {
            if(n_cameras < MAX_CAMERA_VIZ)
                avCameras = n_cameras;
            else
                std::cout<<"Maximum "<<MAX_CAMERA_VIZ<<" cameras can be shown"<<std::endl;
            cameraData.resize(avCameras);
            for(auto& cd: cameraData)
                cd.mtxNewFrame  = new std::mutex;

            fontPath = "../data/monofonto.ttf";
        }
        ~EdgeViewer() {
            // for(auto& cd: cameraData)
            //     delete cd.mtxNewFrame;
        }

        void init() {
            tk::gui::Viewer::init();
            for(auto& cd: cameraData)
                glGenTextures(1, &cd.frameTexture);
        }

        void draw() {
            tk::gui::Viewer::draw();

            ImGui::Begin("Camera visualization",&open, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove);
            ImGui::SetWindowSize(ImVec2(70*avCameras,40));
            ImGui::SetWindowPos(ImVec2(0,0));            
            for(auto& cd:cameraData){
                ImGui::Checkbox(cd.id.c_str(), cd.showCamera);
                ImGui::SameLine();
            }
            ImGui::End();
            resetShowIds();

            for(auto& cd:cameraData){
                if(!(*cd.showCamera)) continue;
            
                //when receiving a new frame, convert cv::Mat into GLuint texture
                if(cd.newFrame){
                    glBindTexture(GL_TEXTURE_2D, cd.frameTexture);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
                    cd.mtxNewFrame->lock();
                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,cd.frame.cols, cd.frame.rows, 0, GL_BGR,GL_UNSIGNED_BYTE, cd.frame.data);
                    cd.frame_width  = cd.frame.cols;
                    cd.frame_height = cd.frame.rows;
                    cd.newFrame     = false;
                    cd.mtxNewFrame->unlock();
                    cd.aspectRatio  = float(cd.frame_width)/float(cd.frame_height); 
                }

                //set 2D view 
                settkViewport2D(cd.showId);

                //draw frame
                glPushMatrix(); {
                    glTranslatef(0, 0, 0.001);
                    glColor4f(1,1,1,1);
                    cd.xScale = xLim;
                    cd.yScale = cd.xScale*cd.aspectRatio;
                    tkDrawTexture(cd.frameTexture, cd.yScale, cd.xScale);
                } glPopMatrix();

                //draw detections
                cd.mtxNewFrame->lock();
                for(const auto& d: cd.detected){
                    tk::gui::Color_t col = tk::gui::color::RED;
                    if(colors.size() > 0) col = colors[d.cl];                    
                    tkSetColor(col);

                    cd.pose = convertPosition((d.x+d.w/2), (d.y+d.h/2), -0.002, cd.id);
                    cd.size = convertSize(d.w, d.h, cd.id);
                    tkDrawRectangle(cd.pose, cd.size, false);

                    tkDrawRectangle(tk::common::Vector3<float>{cd.pose.x, cd.pose.y+ d.h/2.0/cd.frame_height*cd.xScale + 0.008*cd.yScale, cd.pose.z+0.001},
                                    tk::common::Vector3<float>{classesNames[d.cl].size()/float(cd.frame_width)*12*cd.xScale, 0.015*cd.yScale, 0}, 
                                    true);
                    tkSetColor(tk::gui::color::WHITE);
                    

                    tkDrawText(classesNames[d.cl],tk::common::Vector3<float>{cd.pose.x - (classesNames[d.cl].size()/float(cd.frame_width)*12*cd.xScale)/2, 
                                                cd.pose.y+ d.h/2.0/cd.frame_height*cd.xScale + 0.002*cd.yScale, cd.pose.z},
                                    tk::common::Vector3<float>{0, 0, 0},
                                    tk::common::Vector3<float>{0.025*cd.xScale, 0.015*cd.yScale, 0});
                }
                cd.mtxNewFrame->unlock();

                //draw lines
                cd.mtxNewFrame->lock();
                for(const auto& l:cd.lines){
                    glLineWidth(3);
                    tkSetColor(l.color);
                    tkDrawLine(l.points);
                }
                cd.mtxNewFrame->unlock();
            
            }
            
        }
        void setFrameData(const cv::Mat &new_frame, const std::vector<tk::dnn::box> &new_detected, const std::vector<tracker_line>& new_lines, std::string id){ 
            int cur_index = idIndexBind[std::stoi(id)];
            
            cameraData[cur_index].mtxNewFrame->lock();
            cameraData[cur_index].newFrame  = true;
            cameraData[cur_index].frame     = new_frame.clone();
            cameraData[cur_index].detected  = new_detected;
            cameraData[cur_index].lines     = new_lines;
            cameraData[cur_index].mtxNewFrame->unlock();
        }

        void setClassesNames(const std::vector<std::string>& classes_names){
            classesNames = classes_names;
        }

        tk::common::Vector3<float> convertPosition(int x, int y, float z, std::string id){
            float new_x = ((float)x/(float)cameraData[idIndexBind[std::stoi(id)]].frame_width - 0.5)*cameraData[idIndexBind[std::stoi(id)]].yScale;
            float new_y = -((float)y/(float)cameraData[idIndexBind[std::stoi(id)]].frame_height -0.5)*cameraData[idIndexBind[std::stoi(id)]].xScale;
            return tk::common::Vector3<float>{new_x, new_y, z};
        }

        tk::common::Vector3<float> convertSize(int w, int h, std::string id){
            float new_w = (float)w/(float)cameraData[idIndexBind[std::stoi(id)]].frame_width * cameraData[idIndexBind[std::stoi(id)]].yScale;
            float new_h = (float)h/(float)cameraData[idIndexBind[std::stoi(id)]].frame_height * cameraData[idIndexBind[std::stoi(id)]].xScale;
            return tk::common::Vector3<float>{new_w, new_h, 0};
        }

        void setColors(const int n_classes){
            for(int c=0; c<n_classes; ++c) {
                int offset = c*123457 % n_classes;
                float r = getColor(2, offset, n_classes);
                float g = getColor(1, offset, n_classes);
                float b = getColor(0, offset, n_classes);
                colors.push_back(tk::gui::Color_t {int(255.0*b), int(255.0*g), int(255.0*r), 255});
            }
        }

        void bindCamera(std::string id, bool *show_camera)
        {         
            mtxIndex.lock();
            idIndexBind[std::stoi(id)] = cameraIndex++;
            mtxIndex.unlock();
            cameraData[idIndexBind[std::stoi(id)]].id = id;
            cameraData[idIndexBind[std::stoi(id)]].showCamera = show_camera;
        }

        void resetShowIds(){
            int show_id = 0;
            for(auto& cd:cameraData)
                if(*cd.showCamera)
                    cd.showId = show_id++;
                else
                    cd.showId = -1;
            shCameras = show_id;
        }

        void tkViewport2D(int width, int height, int x, int y) {
            float ar = (float)width / (float)height;

            glViewport(x, y, width, height);
            glOrtho(0, width, 0, height, -1, 1);
            glLoadIdentity();
            
            glMatrixMode( GL_PROJECTION );
            glLoadIdentity();

            glOrtho(-ar, ar, -1, 1, 1, -1);

            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
        }
};

}

#endif /*EDGEVIEWER_H*/