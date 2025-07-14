#!/bin/bash

# cd ../build && \
# SECONDS=0 && \
# nohup ./edge -i ../data/all_cameras_en.yaml -s0 -v 1 -u 1 20936 20937 > ./program.log 2>&1 & \
# bg_pid=$! && \
# echo "one $!" && \
# wait $bg_pid && echo "waiting job seconds: $SECONDS" && \
# mv ../data/output/* ~/data/florencia/resistenza/videos/ && \
# cd -

export GST_DEBUG=0
export ASAN_OPTIONS=detect_leaks=1:strict_string_checks=1:verbosity=1:log_path=asan.log

# configure (default generator is "Unix Makefiles")
cmake -B build -DCMAKE_BUILD_TYPE= \
      -DENABLE_SANITIZER_ADDRESS=ON \
      ..
make && \
nohup ./edge -i ../data/all_cameras_en.yaml -s0 -v 1 -u 1 0003 > ../out.log 2>&1 &
