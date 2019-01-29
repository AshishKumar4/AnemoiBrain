#! /bin/bash

if [ ! -d "build" ]; then
    mkdir build 
    cd build 
    if [[ $1 == "gcc" ]]; then
        cmake -D CMAKE_BUILD_TYPE=Debug -D CMAKE_C_COMPILER=gcc -D CMAKE_CXX_COMPILER=g++ --build ..
    elif [[ $1 == "clang-4" ]]; then
        cmake -D CMAKE_BUILD_TYPE=Debug -D CMAKE_C_COMPILER=clang-4.0 -D CMAKE_CXX_COMPILER=clang++-4.0 --build ..
    elif [[ $1 == "clang-5" ]]; then
        cmake -D CMAKE_BUILD_TYPE=Debug -D CMAKE_C_COMPILER=clang-5.0 -D CMAKE_CXX_COMPILER=clang++-5.0 --build ..
    elif [[ $1 == "clang-7" ]]; then
        cmake -D CMAKE_BUILD_TYPE=Debug -D CMAKE_C_COMPILER=clang-7.0 -D CMAKE_CXX_COMPILER=clang++-7.0 --build ..
    elif [[ $1 == "clang" ]]; then
        cmake -D CMAKE_BUILD_TYPE=Debug -D CMAKE_C_COMPILER=clang -D CMAKE_CXX_COMPILER=clang++ --build ..
    else
        cmake ..
    fi
    cd ..
fi
cd build
make

cd ..
cp build/libDrone* lib/

echo "Compilation Completed!"
