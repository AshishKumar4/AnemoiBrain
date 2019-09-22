#! /bin/bash

cd CoreSystem
./build.sh $1 $2
cd ../ControlServer
./build.sh $1
#cd ../SensorServer
#./build.sh $1
cd ..

ExtraArgs="-D COMPILE_MODE:STRING="$2

echo $ExtraArgs

if [ ! -d "build" ]; then
    mkdir build 
    cd build 
    if [[ $1 == "gcc" ]]; then
        cmake -D CMAKE_BUILD_TYPE=Debug -D CMAKE_C_COMPILER=gcc -D CMAKE_CXX_COMPILER=g++ $ExtraArgs --build ..
    elif [[ $1 == "clang-5" ]]; then
        cmake -D CMAKE_BUILD_TYPE=Debug -D CMAKE_C_COMPILER=clang-5.0 -D CMAKE_CXX_COMPILER=clang++-5.0 $ExtraArgs --build ..
    elif [[ $1 == "clang-7" ]]; then
        cmake -D CMAKE_BUILD_TYPE=Debug -D CMAKE_C_COMPILER=clang-7 -D CMAKE_CXX_COMPILER=clang++-7 $ExtraArgs --build ..
    elif [[ $1 == "clang-8" ]]; then
        cmake -D CMAKE_BUILD_TYPE=Debug -D CMAKE_C_COMPILER=clang-8 -D CMAKE_CXX_COMPILER=clang++-8 $ExtraArgs --build ..
    elif [[ $1 == "clang" ]]; then
        cmake -D CMAKE_BUILD_TYPE=Debug -D CMAKE_C_COMPILER=clang -D CMAKE_CXX_COMPILER=clang++ $ExtraArgs --build ..
    else
        cmake ..
    fi
    cd ..
fi
cd build
make -j4
cd ..
cp build/GardienOnboard GardienOnboard

echo "Compilation Completed!"
