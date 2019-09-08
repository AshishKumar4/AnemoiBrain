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
        h = getLocation().x;//Xcoord;
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
        h = getLocation().y;//Ycoord;
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
        h = getLocation().z;//Zcoord;
        if( h < 0 ) h = 0;
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

extern GeoPoint_t AIRSIM_location;
extern vector3D_t AIRSIM_velocity;

vector3D_t AirSim_Locator_t::getVelocity() // CHANGE THIS
{
    //tmpVelocity = AIRSIM_velocity;
    return AIRSIM_velocity;
}

GeoPoint_t AirSim_Locator_t::getLocation() // CHANGE THIS
{
    try
    {
    	return AIRSIM_location;
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

#elif defined(MODE_REALDRONE)

extern GeoPoint_t REAL_location;
extern vector3D_t REAL_velocity;

vector3D_t Real_Locator_t::getVelocity() // CHANGE THIS
{
    return REAL_velocity;
}

GeoPoint_t Real_Locator_t::getLocation() // CHANGE THIS
{
	return REAL_location;
}

#endif