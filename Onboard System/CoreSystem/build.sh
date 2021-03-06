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
        cmake -DCOMPILE_MODE=$2 -D CMAKE_BUILD_TYPE=Debug -D CMAKE_C_COMPILER=gcc -D CMAKE_CXX_COMPILER=g++ --build ..
    elif [[ $1 == "clang-5" ]]; then
        cmake -DCOMPILE_MODE=$2 -D CMAKE_BUILD_TYPE=Debug -D CMAKE_C_COMPILER=clang-5.0 -D CMAKE_CXX_COMPILER=clang++-5.0 --build ..
    elif [[ $1 == "clang-7" ]]; then
        cmake -DCOMPILE_MODE=$2 -D CMAKE_BUILD_TYPE=Debug -D CMAKE_C_COMPILER=clang-7 -D CMAKE_CXX_COMPILER=clang++-7 --build ..
    elif [[ $1 == "clang-8" ]]; then
        cmake -DCOMPILE_MODE=$2 -D CMAKE_BUILD_TYPE=Debug -D CMAKE_C_COMPILER=clang-8 -D CMAKE_CXX_COMPILER=clang++-8 --build ..
    elif [[ $1 == "clang" ]]; then
        cmake -DCOMPILE_MODE=$2 -D CMAKE_BUILD_TYPE=Debug -D CMAKE_C_COMPILER=clang -D CMAKE_CXX_COMPILER=clang++ --build ..
    else
        cmake ..
    fi
    cd ..
fi
cd build
make -j4
cd ..
if [ ! -d "lib" ]; then
    mkdir lib 
fi
cp build/libOnboardCore.* ./lib/

echo "Compilation Completed!"

