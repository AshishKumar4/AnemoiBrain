#pragma once

#include "../common.hpp"

class GlobalLocator_t
{
public:
    GlobalLocator_t()
    {

    }
    float get_X_Coordinate();
    float get_Y_Coordinate();
    
    float get_X_VelocityRel();
    float get_Y_VelocityRel();
    float get_Z_VelocityRel();
    
    float get_X_VelocityAbs();
    float get_Y_VelocityAbs();
    float get_Z_VelocityAbs();

    vector3D_t getVelocityAbs();
    vector3D_t getVelocityRel();
    virtual vector3D_t getVelocity() = 0;

    virtual vector3D_t getPosition() = 0;
    virtual GeoPoint_t getLocation() = 0;
};

class GNSS_Locator_t : public GlobalLocator_t
{

};

#if defined MODE_AIRSIM

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "rpc/server.h"

class AirSim_Locator_t : public GlobalLocator_t
{
    msr::airlib::MultirotorRpcLibClient* client;
public:
    AirSim_Locator_t(msr::airlib::MultirotorRpcLibClient *client)
    {
        this->client = client;
    }

    vector3D_t getVelocity();

    vector3D_t getPosition();
    GeoPoint_t getLocation();
};

#endif

class AdvancedGNSS_Locator_t : public GlobalLocator_t
{

};

