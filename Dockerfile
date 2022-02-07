FROM nvcr.io/nvidia/tensorrt:20.03-py3

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y git build-essential rsync libssl-dev libeigen3-dev libglew-dev libglfw3-dev freeglut3-dev libfreetype6-dev libyaml-cpp-dev libpcap-dev libmatio-dev
RUN cmake --version
RUN wget https://github.com/Kitware/CMake/releases/download/v3.22.1/cmake-3.22.1.tar.gz && \
    tar xf cmake-3.22.1.tar.gz && \ 
    cd cmake-3.22.1 && \
    ./bootstrap && make -j4 && make install

RUN cmake --version
# Update cmake for tkdnn compiling
RUN apt install -y gpg wget && \
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null && \
    echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ bionic main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null && \
    apt update -y && rm /usr/share/keyrings/kitware-archive-keyring.gpg && \
    apt install -y kitware-archive-keyring && \
    apt update -y && apt install -y cmake && cmake --version

RUN apt-get install -y python3-matplotlib python-dev libgdal-dev libcereal-dev python-numpy
RUN apt-get install -y libgles2-mesa-dev xorg-dev libglu1-mesa-dev
RUN apt-get install -y libopencv-dev libopencv-contrib-dev

WORKDIR /root

RUN mkdir repos

WORKDIR /root/repos

RUN apt-get install -y build-essential \
    unzip \
    pkg-config \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libgtk-3-dev \
    libatlas-base-dev \
    gfortran \
    python3-dev \
    python3-venv \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libdc1394-22-dev \
    libavresample-dev

RUN git clone https://github.com/opencv/opencv.git && git clone https://github.com/opencv/opencv_contrib.git

RUN mkdir -p opencv/build

WORKDIR /root/repos/opencv/build

RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D INSTALL_C_EXAMPLES=OFF \
    -D OPENCV_EXTRA_MODULES_PATH='/root/repos/opencv_contrib/modules' \
    -D BUILD_EXAMPLES=ON \
    -D WITH_CUDA=ON \
    -D CUDA_ARCH_BIN=7.0 \
    -D CUDA_ARCH_PTX="" \
    -D ENABLE_FAST_MATH=ON \
    -D CUDA_FAST_MATH=ON \
    -D WITH_CUBLAS=ON \
    -D WITH_LIBV4L=ON \
    -D WITH_GSTREAMER=ON \
    -D WITH_GSTREAMER_0_10=OFF \
    -D WITH_TBB=ON \
    ../

RUN make -j4
RUN make install
RUN ldconfig

#WORKDIR /root/repos/
#RUN git clone https://git.hipert.unimore.it/mverucchi/class-edge.git
#ENV AAA=BBB
#COPY . /root/repos/class-edge
#WORKDIR /root/repos/class-edge
#RUN wget https://github.com/glfw/glfw/releases/download/3.3/glfw-3.3.zip && unzip glfw-3.3.zip && mkdir -p glfw-3.3/build && cd glfw-3.3/build && cmake .. && make -j4 && make install

RUN apt install -y libglm-dev

# Class edge compiling requirements
RUN  wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz && \
     tar xf eigen-3.4.0.tar.gz && cd eigen-3.4.0 && \
     mkdir build_dir && cd build_dir && cmake .. && make install -j4
#RUN apt install -y geographiclib-tools
RUN   wget https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.52.tar.gz && \
tar xfpz GeographicLib-1.52.tar.gz && \
cd GeographicLib-1.52  && \
mkdir BUILD && \
cd BUILD && \
cmake .. && \
make install -j4

RUN python3 -m pip install --upgrade pip && \
    apt install -y libzmq3-dev && \
    pip install pytest

RUN git clone https://github.com/pybind/pybind11.git && \
    cd pybind11 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make check -j8 && \
    make install

ENV AAA=BBB
COPY . /root/repos/class-edge
WORKDIR /root/repos/class-edge
RUN wget https://github.com/glfw/glfw/releases/download/3.3/glfw-3.3.zip && unzip glfw-3.3.zip && mkdir -p glfw-3.3/build && cd glfw-3.3/build && cmake .. && make -j4 && make install

RUN git submodule add https://gitlab.bsc.es/ppc-bsc/masa_protocol ./masa_protocol
RUN git submodule update --init --recursive 

# Install matplotlib
#RUN cd /root/repos/ && git clone https://github.com/Microsoft/vcpkg.git && \
#apt-get install -y curl zip unzip tar && \
#cd vcpkg && \
#./bootstrap-vcpkg.sh && \
#./vcpkg integrate install && \
#./vcpkg install matplotlib-cpp


# Instal tracker_class
#RUN git clone https://github.com/mive93/tracker_CLASS  && \
#cd tracker_CLASS  && \
#git submodule update --init --recursive  && \
#mkdir build  && \
#cd build  && \
#cmake -DCMAKE_BUILD_TYPE=Release ..  && \
#make -j4


#Install catkin
##RUN apt-get update -y && apt-get install -y lsb-release && \
##    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
##        > /etc/apt/sources.list.d/ros-latest.list' && \
##wget http://packages.ros.org/ros.key -O - | apt-key add - && \
##apt-get update -y && \
##apt-get install -y python3-catkin-tools python3-empy


# Install catkin_simple
##RUN CATKIN_WS=~/catkin_ws && \
##mkdir -p $CATKIN_WS/src && \
##cd $CATKIN_WS/src && \
##git clone https://github.com/catkin/catkin_simple.git && \
##git clone https://github.com/ros/catkin.git && \ 
##cd $CATKIN_WS && \
##catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m

#RUN mkdir -p ~/catkin_ws/src && \
#cd ~/catkin_ws/src 
#catkin_init_workspace && \

#RUN cd /root/repos && wget https://github.com/ethz-asl/geodetic_utils/archive/refs/heads/master.zip && \
#unzip master.zip && cd geodetic_utils-master/geodetic_utils && \
#mkdir build && cd build 
#&& cmake .. && make install -j4




#WORKDIR /root/repos/class-edge/glfw-3.3
#RUN git submodule update --init --recursive && git submodule update --remote --recursive && \
#RUN cd class-tracker/ && \
#git submodule update --init --recursive && git submodule update --remote --recursive && \
#cd - && \

RUN mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. && \
    make
# Creating .rt file of DNN used
RUN cd build/tkDNN && ./test_yolo4_berkeley


# COMPSS added in this dockerfile for PAI Project. 
RUN python3 -m pip install dataclay pymap3d zmq requests pygeohash geolib 
RUN apt install -y libeigen3-dev libflann-dev python3-matplotlib python-dev python-dev libflann-dev
RUN ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
WORKDIR /root/repos/
RUN git clone --depth 1 --branch udpsockets https://github.com/class-euproject/COMPSs-obstacle-detection.git &&  \
    cd COMPSs-obstacle-detection && \
    git submodule init && \
    git submodule update && \
    cd ./tracker_CLASS/ && \
    mkdir build && \
    cd build && \
    cmake ../ && \
    make -j && \
    mv track*.so ../../track.so
# cd class-edge/build/tkDNN/ && ./demo 1 2404 5559 -1 0 0 0.5 &
COPY tracker.py ./COMPSs-obstacle-detection/