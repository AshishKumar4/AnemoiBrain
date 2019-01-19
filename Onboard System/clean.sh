#! /bin/bash

cd Utils/; ./clean.sh
cd ../ControlAbstraction; ./clean.sh 
cd ../SensorAbstraction; ./clean.sh
cd ..
rm -rf build
rm -rf GardienOnboard