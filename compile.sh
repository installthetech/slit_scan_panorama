#!/bin/bash -e
reset
### OpenCV warning 
echo "Make sure you have installed OpenCV on the system, and it is added to PKG_CONFIG variable."
### Compile files in 'src' and generate executable in 'bin' folder
g++ src/UTILITY.cpp src/PRIMEPANORAMA.cpp src/DRAWIMAGE.cpp src/main.cpp -o bin/panorama `pkg-config --cflags --libs opencv` -std=c++11
### Executable done
reset
echo "Done compiling."
