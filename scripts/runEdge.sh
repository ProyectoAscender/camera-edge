#!/bin/bash
cd ../build && \
SECONDS=0 && \
nohup ./edge -i ../data/all_cameras_en.yaml -s0 -v 1 -u 1 20936 20937 > ./program.log 2>&1 & \
bg_pid=$! && \
echo "one $!" && \
wait $bg_pid && echo "waiting job seconds: $SECONDS" && \
mv ../data/output/* ~/data/florencia/resistenza/videos/ && \
cd -