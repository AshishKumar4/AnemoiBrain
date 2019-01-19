#! /bin/bash

cd Utils
./build.sh $1 $2
cd ../ControlAbstraction
./build.sh $1
cd ../SensorAbstraction
./build.sh $1
cd ..

if [ ! -d "build" ]; then
    mkdir build 
    cd build 
    if [[ $1 == "gcc" ]]; then
        cmake -D CMAKE_BUILD_TYPE=Debug -D CMAKE_C_COMPILER=gcc -D CMAKE_CXX_COMPILER=g++ --build ..
    elif [[ $1 == "clang-5" ]]; then
        cmake -D CMAKE_BUILD_TYPE=Debug -D CMAKE_C_COMPILER=clang-5.0 -D CMAKE_CXX_COMPILER=clang++-5.0 --build ..
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
cp build/GardienOnboard GardienOnboard

echo "Compilation Completed!"
