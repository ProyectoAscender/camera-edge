# class-edge

Class edge is part from smart city pipeline. It executes YOLOv6 with tensorRT API to an ethernet camera streaming. This connection is configured with `data/all_cameras.yaml`. From this streaming frame by frame object detection boxes are obtained and send trough stablished port (`port communicator`) by udp connection. 

At this moment, for Ascender project camera-edge is executed at agx12 machine. This is a Jetson Orin 
First this repository must be cloned at your home folder.


Once there, execute the container.

`bash docker/l4t-trt/runDocker.sh`

It will execute a docker run which, inside the executed container, compiles the project first and executes it after. At this very moment, the code which will be compiled and executed, is linked to your host with: `-v ~/camera-edge/:/root/repos/camera-edge/`.  It's not inside docker container in order to be able to execute fast your code modifications with the complete pipeline. 

This runDocker executes what you have in `scripts/runEdge.sh` which contains:
`./edge -i ../data/all_cameras_en.yaml -s0 -v 1 -u 1 0001`

Important here is `0001`, which is the name of the camera the streraming reading is going to get. It's configured at `../data/all_cameras_en.yaml`. There you can change the ip is going to be reading streaming from (`ìnput`). And control de length of the project with `framesToProcess` and `neverend`. Other important configuration is the port (`portCommunicator`) which is used to send the boxes info with udp. smart-city-compss will target this port.

The docker will run in background. You can get the stdout of the process invoking `docker logs` of the container created. Once the docker is stopped, it will be erased.


