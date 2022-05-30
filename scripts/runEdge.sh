#!/bin/bash
cd ../build && \
./edge -i ../data/all_cameras_en.yaml -s0 -v 1 -u 1 20936 20937 && \ 
rsync -uazPt ../data/output/ ~/data/florencia/resistenza/videos && \
cd -