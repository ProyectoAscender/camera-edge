IMAGE=registry.gitlab.bsc.es/ppc/benchmarks/smart-city/camera-edge:trt-r8.5.2.2-devel
docker run --runtime nvidia --name edge02 -p 8080:8080 -p 5000:5000 -p 5000:5000/udp -p 8884:8884 -p 8884:8884/udp -p 5001:5001 -p 5001:5001/udp -p 8884:8884 -p 8884:8884/udp  --network host -v ~/camera-edge/:/root/repos/camera-edge/ -v /root/repos/camera-edge/build -v /root/repos/camera-edge/masa_protocol -v /root/repos/camera-edge/tracker  -v /mnt/b2drop/smartCity:/root/b2drop -it $IMAGE /bin/bash

#-v /root/repos/camera-edge/trt-detect
# IMAGE=nvcr.io/nvidia/l4t-tensorrt:r8.5.2.2-devel
# docker run --runtime nvidia --name edge02 -p 8884:8884 -p 8884:8884/udp -v ~/camera-edge/:/root/repos/camera-edge/ -v /root/repos/camera-edge/build -v /root/repos/camera-edge/masa_protocol -v /root/repos/camera-edge/tracker  -v /mnt/b2drop/smartCity:/root/ -it $IMAGE /bin/bash
# #-v /root/repos/camera-edge/trt-detect