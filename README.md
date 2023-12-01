# class-edge

class-edge provides the software for the edges of a smart city, i.e. smart cameras, in the context of the European Project CLASS (H2020, G.A. 780622)

## Dependencies

This projects depends on: 

  * CUDA 10.0
  * CUDNN 7.603
  * TENSORRT 6.01
  * OPENCV 3.4
  * yaml-cpp 0.5.2 
  * Eigen
  * GDal
  * cmake v3.15

```
sudo apt-get install -y libeigen3-dev \
                        python3-matplotlib \
                        python3.6-dev \
                        libgdal-dev \
                        libcereal-dev \
                        libyaml-cpp-dev \
                        python-numpy
```

required for tkCommon
```
sudo apt-get install -y libgles2-mesa-dev libglew-dev libmatio-dev libpcap-dev
bash scripts/install-glfw-3.3.sh
```

## How to build the project

```
https://git.hipert.unimore.it/mverucchi/class-edge.git
cd class-edge
git submodule update --init --recursive 
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -j4
```
## How to launch container
docker run --name edge01 -p 8886:8886 -v ~/camera-edge/:/root/repos/camera-edge/ -v /root/repos/camera-edge/build -v /root/repos/camera-edge/tkDNN -v /
root/repos/camera-edge/tkCommon -v /root/repos/camera-edge/tracker -v /root/repos/camera-edge/masa_protocol -v /media/b2drop/smartCity:/root/ -it bscppc/camera-edge:r32.5.0 /bin/bash

## How to launch the edge

In general (use ./edge -h for help)
```
./edge -i <parameter-file> <cam-id-1> <cam-id-2> ... <cam-id-8> 
```
Example:
```
./edge -i ../data/all_cameras_en.yaml -s0 -v 1 -u 1 0001  
```

To better understand the parameter file, refer to [config doc](data/README.md).

## How to initialize or update submodule
```
git submodule update --init --recursive #initialize
git submodule update --remote --recursive  #update all
```

## How to encrypt 

This is how you encrypt a string (omit -iter 100000 with Ubuntu 16.04):
```
echo -n "yourAwesomeString" | openssl enc -e -aes-256-cbc -a -salt -iter 100000
```
In you want to encrypt the input of a parameters file, be sure that the field ```encrypted``` is set to 0.
Then just run 
```
./encrypt <params-no-enc> <params-enc>
```
where
  * ```<params-no-enc>``` is the input parameters file (yaml)
  * ```<params-enc>``` is the output parameters file (yaml) with all the input encrypted with the password the program will ask for.
