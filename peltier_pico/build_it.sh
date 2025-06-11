#! /bin/bash

mkdir build
cd build
cmake -DPICO_PLATFORM=rp2350 .. && make
