#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>
#include <thread> // std::thread
#include <mutex>
#include <stdexcept>
#include <future>

//#include "common.hpp"
#include "Sensors/Location.hpp"

void GlobalLocator_t::bufferWriter(GlobalLocator_t *locator)
{
    while (1)
    {
        try
        {
            /*locator->XLock.lock();
            locator->YLock.lock();
            locator->ZLock.lock();*/

            GeoPoint_t location = locator->getLocation();
            locator->Xcoord = location.x;
            locator->Ycoord = location.y;
            locator->Zcoord = location.z;

            /*locator->XLock.unlock();
            locator->YLock.unlock();
            locator->ZLock.unlock();*/
        }
        catch (const std::future_error &e)
        {
            std::cout << "<GlobalLocator_t::bufferWriter>Caught a future_error with code \"" << e.code()
                      << "\"\nMessage: \"" << e.what() << "\"\n";
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }
}

float GlobalLocator_t::get_X_Coordinate()
{
    float h = 0;
    //this->XLock.lock();
    try
    {
        h = Xcoord;
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalLocator_t::get_X_Coordinate>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    //this->XLock.unlock();
    return h;
}

float GlobalLocator_t::get_Y_Coordinate()
{
    float h = 0;
    //this->XLock.lock();
    try
    {
        h = Ycoord;
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalLocator_t::get_Y_Coordinate>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    //this->XLock.unlock();
    return h;
}

float GlobalLocator_t::get_Z_Coordinate()
{
    float h = 0;
    //this->XLock.lock();
    try
    {
        h = Zcoord;
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalLocator_t::get_Z_Coordinate>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    //this->XLock.unlock();
    return h;
}

float GlobalLocator_t::get_X_VelocityRel()
{

    try
    {
        return this->getVelocityRel().x;
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalState_t::getVelocityRel>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}

float GlobalLocator_t::get_Y_VelocityRel()
{
    try
    {
        return this->getVelocityRel().y;
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalState_t::getVelocityRel>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}

float GlobalLocator_t::get_Z_VelocityRel()
{
    try
    {
        return this->getVelocityRel().z;
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalState_t::getVelocityRel>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}

float GlobalLocator_t::get_X_VelocityAbs()
{
    return this->getVelocityAbs().x;
}

float GlobalLocator_t::get_Y_VelocityAbs()
{
    return this->getVelocityAbs().y;
}

float GlobalLocator_t::get_Z_VelocityAbs()
{
    return this->getVelocityAbs().z;
}

/***********************************************************************************************/
/******************************* A Little Higher Level Get APIs ********************************/
/***********************************************************************************************/

vector3D_t GlobalLocator_t::getVelocityAbs()
{
    return this->getVelocity(); // CHANGE THIS
}

vector3D_t GlobalLocator_t::getVelocityRel()
{
    try
    {
        return this->getVelocity(); // CHANGE THIS
    }

    catch (const std::future_error &e)
    {
        std::cout << "<GlobalState_t::getVelocityRel>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return vector3D_t(0,0,0);
}

/* ------------------------------------------------------------------------------------------------------------------------ */
/*----------------------------------------------------- AirSim_Locator -----------------------------------------------------*/
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined MODE_AIRSIM

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "rpc/server.h"

namespace
{
vector3D_t tmpVelocity;
GeoPoint_t tmpLocation;
}

vector3D_t AirSim_Locator_t::getVelocity() // CHANGE THIS
{
    auto velocity = client->getMultirotorState().kinematics_estimated.twist.linear;
    tmpVelocity = vector3D_t(velocity.x(), velocity.y(), velocity.z());
    return tmpVelocity;
}

GeoPoint_t AirSim_Locator_t::getLocation() // CHANGE THIS
{
    try
    {
        auto location = client->getMultirotorState().getPosition();
        tmpLocation = GeoPoint_t(location.y(), location.x(), -location.z());
        // Airsim gives us relative position, with start position as 0,0,0, and altitude is negative above us
        return tmpLocation;
    }
    catch (const std::future_error &e)
    {
        std::cout << "<AirSim_Locator_t::getLocation>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
        return tmpLocation; //getLocation();
    }
    catch (const std::exception &e)
    {
        std::cout << e.what() << '\n';
    }
}

#endif