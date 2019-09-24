#pragma once 

#include "common.hpp"
#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>
#include <thread> // std::thread
#include <mutex>
#include <queue>
#include <atomic>

class InertialEstimator_t 
{
public:
    InertialEstimator_t()
    {
        //bufferWriterThread = new std::thread(bufferWriter, this);
    }

    ~InertialEstimator_t()
    {
        //bufferWriterThread->join();
    }

    virtual quaternion_t& getOrientation() = 0;
    virtual vector3D_t getEulerOrientation();
	virtual vector3D_t getAcceleration();
};

#if defined MODE_REALDRONE

class Real_IMU_t : public InertialEstimator_t
{
	public:
	Real_IMU_t()
	{
		
	}
    quaternion_t& getOrientation();
    vector3D_t getEulerOrientation();
};

#endif