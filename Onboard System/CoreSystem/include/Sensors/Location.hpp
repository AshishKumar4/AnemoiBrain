#pragma once

#include "../common.hpp"
#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>
#include <thread> // std::thread
#include <mutex>
#include <queue>
#include <atomic>

class GlobalLocator_t
{
    static void bufferWriter(GlobalLocator_t* locator);

    std::thread*        bufferWriterThread;
    std::atomic<float> Xcoord;
    std::atomic<float> Ycoord;
    std::atomic<float> Zcoord;

    std::mutex      XLock;
    std::mutex      YLock;
    std::mutex      ZLock;

public:
    GlobalLocator_t()
    {
        bufferWriterThread = new std::thread(bufferWriter, this);
    }

    ~GlobalLocator_t()
    {
        bufferWriterThread->join();
    }

    float get_X_Coordinate();
    float get_Y_Coordinate();
    float get_Z_Coordinate();
    
    float get_X_VelocityRel();
    float get_Y_VelocityRel();
    float get_Z_VelocityRel();
    
    float get_X_VelocityAbs();
    float get_Y_VelocityAbs();
    float get_Z_VelocityAbs();

    vector3D_t getVelocityAbs();
    vector3D_t getVelocityRel();
    virtual vector3D_t getVelocity() = 0;
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
    GeoPoint_t getLocation();
};

#endif

class AdvancedGNSS_Locator_t : public GlobalLocator_t
{

};

