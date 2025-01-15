ARG ROOT_CONTAINER
FROM ${ROOT_CONTAINER}

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y git build-essential rsync libssl-dev libeigen3-dev libglew-dev libglfw3-dev freeglut3-dev libfreetype6-dev libyaml-cpp-dev libpcap-dev libmatio-dev
RUN apt install -y apt-utils wget && apt install -y --reinstall ca-certificates && wget https://github.com/Kitware/CMake/releases/download/v3.22.1/cmake-3.22.1.tar.gz && \
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

RUN mkdir -p /root/repos
WORKDIR /root/repos

RUN apt-get update -y && apt install -y build-essential \
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
    libavresample-dev \
    ffmpeg \
    libjpeg8-dev \
    libjpeg-turbo8-dev \
    liblapack-dev \
    liblapacke-dev \
    libopenblas-dev \
    libpostproc-dev \
    libtbb-dev \
    libtbb2 \
    libtesseract-dev \
    qv4l2 \
    v4l-utils \
    libglm-dev \
    zlib1g-dev

ARG OPENCV_VERSION="4.7.0"
COPY docker/l4t-trt/build_opencv.sh build_opencv.sh 
RUN bash build_opencv.sh  ${OPENCV_VERSION} 

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

RUN wget https://bootstrap.pypa.io/pip/3.6/get-pip.py && python3 get-pip.py && \
    python3 -m pip install --upgrade pip && \
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
RUN echo "BBBB"
# RUN cd /root/repos && git clone https://pat:ZBDrnj-dzZEUdBAFz6KF@gitlab.bsc.es/ppc/benchmarks/smart-city/camera-edge.git -b master
RUN mkdir -p /root/repos/camera-edge
RUN mkdir -p /root/b2drop
COPY . /root/repos/camera-edge
WORKDIR /root/repos/camera-edge
RUN wget https://github.com/glfw/glfw/releases/download/3.3/glfw-3.3.zip && unzip glfw-3.3.zip && mkdir -p glfw-3.3/build && cd glfw-3.3/build && cmake .. && make -j4 && make install
RUN git submodule add https://gitlab.bsc.es/ppc-bsc/software/masa_protocol.git ./masa_protocol
RUN git submodule update --init --recursive 
# RUN git submodule add https://gitlab.bsc.es/ppc/benchmarks/ai/trt-detect.git ./trt-detect
# RUN git submodule update --init --recursive 

# RUN cd class-edge && \
#     git submodule add https://gitlab.bsc.es/ppc-bsc/masa_protocol ./masa_protocol && \
#     git submodule update --init --recursive 

RUN mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -Wno-dev .. && \
    make
# # Creating .rt file of DNN used
# RUN apt install -y curl
# RUN cd build/tkDNN && \
#     mkdir yolo4_berkeley && \
#     curl -tlsv1 -C - https://cloud.hipert.unimore.it/s/nkWFa5fgb4NTdnB/download --output yolo4_berkeley/weights.zip --user user:pass -O --retry 999 --retry-max-time 0 && \
#     unzip yolo4_berkeley/weights.zip -d yolo4_berkeley && \ 
#     rm yolo4_berkeley/weights.zip && \
#     ./test_yolo4_berkeley && \
#     mv yolo4_berkeley ../

RUN apt-get update -y && apt install -y gdb

# # COMPSS added in this dockerfile for PAI Project. 
# RUN python3 -m pip install dataclay pymap3d zmq requests pygeohash geolib 
# RUN apt install -y libeigen3-dev libflann-dev python3-matplotlib python-dev python-dev libflann-dev
# RUN ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
# WORKDIR /root/repos/
# RUN git clone --depth 1 --branch udpsockets https://github.com/class-euproject/COMPSs-obstacle-detection.git &&  \
#     cd COMPSs-obstacle-detection && \
#     git submodule init && \
#     git submodule update && \
#     cd ./tracker_CLASS/ && \
#     mkdir build && \
#     cd build && \
#     cmake ../ && \
#     make -j && \
#     mv track*.so ../../track.so
# # cd class-edge/build/tkDNN/ && ./demo 1 2404 5559 -1 0 0 0.5 &
# COPY tracker.py ./COMPSs-obstacle-detection/