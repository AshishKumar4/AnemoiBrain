#! /bin/bash

cd CoreSystem/; ./clean.sh
cd ../ControlServer; ./clean.sh 
cd ../SensorServer; ./clean.sh
cd ..
rm -rf build
rm -rf GardienOnboard