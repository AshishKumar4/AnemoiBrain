#! /bin/bash

if [ ! -d "build" ]; then
    if [[ $2 == "airsim" ]]; then
        cp cmake_templates/CMakeLists_AirSim.txt CMakeLists.txt
    else
        cp cmake_templates/CMakeLists_Real.txt CMakeLists.txt
    fi
    mkdir build 
    cd build 
    if [[ $1 == "gcc" ]]; then
        cmake -D CMAKE_BUILD_TYPE=Debug -D CMAKE_C_COMPILER=gcc -D CMAKE_CXX_COMPILER=g++ --build ..
    elif [[ $1 == "clang" ]]; then
        cmake -D CMAKE_BUILD_TYPE=Debug -D CMAKE_C_COMPILER=clang-5.0 -D CMAKE_CXX_COMPILER=clang++-5.0 --build ..
    else
        cmake ..
    fi
    cd ..
fi
cd build
make
cd ..
cp build/ControlServer ControlServer

echo "Compilation Completed!"

