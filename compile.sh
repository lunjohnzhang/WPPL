#!/bin/bash

mkdir build

# build exec for cpp

cd build
cmake ../ -DEIGEN3_INCLUDE_DIR=/media/project0/hongzhi/TrafficFlowMAPF/third_party/eigen \
    -DMINIDNN_DIR=/media/project0/hongzhi/TrafficFlowMAPF/third_party/MiniDNN/include
make -j


# build exec for python

# cd build
# cmake ../ -DPYTHON=true
# make -j
# cp ./*.so ../  #make sue the library is in the working directory