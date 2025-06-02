ARG ROOT_CONTAINER
FROM ${ROOT_CONTAINER}
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y git build-essential rsync libssl-dev libeigen3-dev libglew-dev libglfw3-dev freeglut3-dev libfreetype6-dev libyaml-cpp-dev libpcap-dev libmatio-dev
RUN apt install -y apt-utils wget && apt install -y --reinstall ca-certificates && wget https://github.com/Kitware/CMake/releases/download/v3.22.1/cmake-3.22.1.tar.gz && \
    tar xf cmake-3.22.1.tar.gz && \ 
    cd cmake-3.22.1 && \
    ./bootstrap && make -j4 && make install
RUN cmake --version
# RUN apt-get install -y python3-matplotlib python-dev libgdal-dev libcereal-dev python-numpy
# RUN apt-get install -y libgles2-mesa-dev xorg-dev libglu1-mesa-dev
RUN mkdir -p /root/repos
WORKDIR /root/repos

# install development packages (from dustynv jetson-inference repository)
#
RUN apt-get update -y && apt-get install -y --no-install-recommends lsb-release
RUN add-apt-repository --remove "deb https://apt.kitware.com/ubuntu/ $(lsb_release --codename --short) main" && \
    apt-get update && \
    apt-get purge -y '*opencv*' || echo "existing OpenCV installation not found" && \
    apt-get install -y --no-install-recommends \
          mesa-utils \
          lsb-release \
          gstreamer1.0-tools \
          gstreamer1.0-libav \
          gstreamer1.0-rtsp \
          gstreamer1.0-plugins-good \
          gstreamer1.0-plugins-bad \
          gstreamer1.0-plugins-ugly \
          libgstreamer-plugins-base1.0-dev \
          libgstreamer-plugins-good1.0-dev \
          libgstreamer-plugins-bad1.0-dev && \
    if [ `lsb_release --codename --short` != 'bionic' ]; then \
    apt-get install -y --no-install-recommends \
          gstreamer1.0-plugins-rtp; \
    else echo "skipping packages unavailable for Ubuntu 18.04"; fi \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean
# make a copy of this cause it gets purged...
RUN mkdir -p /usr/local/include/gstreamer-1.0/gst && \
    cp -r /usr/include/gstreamer-1.0/gst/webrtc /usr/local/include/gstreamer-1.0/gst && \
    ls -ll /usr/local/include/ && \
    ls -ll /usr/local/include/gstreamer-1.0/gst/webrtc
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
RUN apt update && apt install -y libzmq3-dev x11-apps
ENV AAA=BBB

RUN mkdir -p /root/b2drop
RUN echo "Downloading camera-edge"
RUN cd /root/repos && git clone https://pat:ZBDrnj-dzZEUdBAFz6KF@gitlab.bsc.es/ppc/benchmarks/smart-city/camera-edge.git -b master
WORKDIR /root/repos/camera-edge
RUN mkdir models
COPY models/yolov6nAGXOrin.engine models/
COPY models/yolov6mAGXOrin.engine models/
RUN echo "Downloading trt-detect"
RUN git clone https://pat:4dvKs5a1DtGsoeVfFxci@gitlab.bsc.es/ppc/benchmarks/ai/trt-detect.git -b Vicente-Masip-CU-86bz8v5k1
RUN apt-get update -y && apt install -y gdb
RUN mkdir build && cd build && cmake .. && make



WORKDIR /root/repos/camera-edge/build
ENTRYPOINT ["./edge", "-i/root/repos/camera-edge/data/all_cameras_en.yaml", "-s0", "-v 1", "-u 1"]
CMD ["0001"]
# ENTRYPOINT ["ls","-laR", "/root/repos/camera-edge/data"] 
# RUN echo hola
# RUN ls -laRt .
# COPY ./CMakeLists.txt /root/repos/camera-edge/CMakeLists.txt
# RUN rm -rf src include
# COPY ./src /root/repos/camera-edge/src
# COPY ./include /root/repos/camera-edge/include
# COPY .gitmodules /root/repos/camera-edge/
# RUN cat CMakeLists.txt
# RUN ls /usr/lib/aarch64-linux-gnu/libnv*
# RUN apt-get update -y && apt install -y gdb
# RUN mkdir ./build && cd build && \
#     cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -Wno-dev .. && \
#     make
