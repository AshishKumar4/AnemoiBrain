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

class InertialMeasurement_t 
{
    std::thread*        bufferWriterThread;
    std::atomic<float>   rollBuffer;      
    std::atomic<float>   pitchBuffer;    
    std::atomic<float>   yawBuffer;    

    std::mutex      rollLock;
    std::mutex      pitchLock;
    std::mutex      yawLock;

    static void bufferWriter(InertialMeasurement_t* imu);

public:
    InertialMeasurement_t()
    {
        //bufferWriterThread = new std::thread(bufferWriter, this);
    }

    ~InertialMeasurement_t()
    {
        //bufferWriterThread->join();
    }

    virtual quaternion_t& getOrientation() = 0;

    virtual vector3D_t getEulerOrientation();
	
    float getYaw();
    float getRoll();
    float getPitch();

    float getYawDegrees();
    float getRollDegrees();
    float getPitchDegrees();
};

#if defined MODE_AIRSIM

// #include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
// #include "rpc/server.h"

class AirSim_IMU_t : public InertialMeasurement_t
{
public: 
    AirSim_IMU_t() : InertialMeasurement_t()
    {
    }

	quaternion_t& getOrientation();
    vector3D_t getEulerOrientation();
};

#elif defined MODE_REALDRONE

class Real_IMU_t : public InertialMeasurement_t
{
	public:
	Real_IMU_t()
	{
		
	}
    quaternion_t& getOrientation();
    vector3D_t getEulerOrientation();
};

#endif