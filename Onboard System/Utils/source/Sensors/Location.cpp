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

#include "common.hpp"
#include "Sensors/Location.hpp"


float GlobalLocator_t::get_X_Coordinate()
{
    return 0;
}

float GlobalLocator_t::get_Y_Coordinate()
{
    return 0;
}

float GlobalLocator_t::get_X_VelocityRel()
{
    return this->getVelocityRel().x;
}

float GlobalLocator_t::get_Y_VelocityRel()
{
    return this->getVelocityRel().y;
}

float GlobalLocator_t::get_Z_VelocityRel()
{
    return this->getVelocityRel().z;
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
    return this->getVelocity();   // CHANGE THIS
}

vector3D_t GlobalLocator_t::getVelocityRel()
{
    return this->getVelocity();   // CHANGE THIS
}


/* ------------------------------------------------------------------------------------------------------------------------ */
/*----------------------------------------------------- AirSim_Locator -----------------------------------------------------*/
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined MODE_AIRSIM

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "rpc/server.h"

vector3D_t AirSim_Locator_t::getVelocity()   // CHANGE THIS
{
    vector3D_t vel;
    return vel;
}

vector3D_t AirSim_Locator_t::getPosition()   // CHANGE THIS
{
    vector3D_t pos;
    return pos;
}

GeoPoint_t AirSim_Locator_t::getLocation()   // CHANGE THIS
{
    GeoPoint_t loc;
    return loc;
}

#endif