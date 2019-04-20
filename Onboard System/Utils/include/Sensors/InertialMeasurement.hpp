#pragma once 

#include "../common.hpp"

class InertialMeasurement_t 
{
public:
    InertialMeasurement_t()
    {

    }

    virtual quaternion_t getOrientation() = 0;
    vector3D_t getEulerOrientation();
    float getYaw();
    float getRoll();
    float getPitch();

    float getYawDegrees();
    float getRollDegrees();
    float getPitchDegrees();
};

class Real_IMU_t : public InertialMeasurement_t
{

};

#if defined MODE_AIRSIM

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "rpc/server.h"

class AirSim_IMU_t : public InertialMeasurement_t
{
    msr::airlib::MultirotorRpcLibClient* client;
public: 
    AirSim_IMU_t(msr::airlib::MultirotorRpcLibClient *client) : InertialMeasurement_t()
    {
        this->client = client;
    }

    quaternion_t getOrientation();
};

#endif