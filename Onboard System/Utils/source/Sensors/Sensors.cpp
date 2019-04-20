
#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>
#include <thread> // std::thread
#include <mutex>
#include <stdexcept>

#include "Sensors/Sensors.hpp"
#include "Sensors/Location.hpp"
#include "Sensors/InertialMeasurement.hpp"
#include "common.hpp"


float GlobalState_t::get_X_Coordinate()
{
    return locator->get_X_Coordinate();
}

float GlobalState_t::get_Y_Coordinate()
{
    return locator->get_Y_Coordinate();
}

float GlobalState_t::get_X_VelocityRel()
{
    return locator->get_X_VelocityRel();
}

float GlobalState_t::get_Y_VelocityRel()
{
    return locator->get_Y_VelocityRel();
}

float GlobalState_t::get_Z_VelocityRel()
{
    return locator->get_Z_VelocityRel();
}

float GlobalState_t::get_X_VelocityAbs()
{
    return locator->get_X_VelocityAbs();
}

float GlobalState_t::get_Y_VelocityAbs()
{
    return locator->get_Y_VelocityAbs();
}

float GlobalState_t::get_Z_VelocityAbs()
{
    return locator->get_Z_VelocityAbs();
}

float GlobalState_t::getAltitude()
{
    return 0;
}

float GlobalState_t::getHeadingDegrees() // Gives in Degrees
{
    return this->getYawDegrees();
}

float GlobalState_t::getHeading()
{
    return this->getHeadingDegrees();
}

/* ------------------------------------------------------------------------------------------------------------------------ */
/*------------------------------------------------- Rotation related stuff -------------------------------------------------*/
/* ------------------------------------------------------------------------------------------------------------------------ */

float GlobalState_t::getYaw()
{
    return imu->getYaw();
}

float GlobalState_t::getRoll()
{
    return imu->getRoll();
}

float GlobalState_t::getPitch()
{
    return imu->getPitch();
}

float GlobalState_t::getYawDegrees()
{
    return imu->getYawDegrees();
}

float GlobalState_t::getRollDegrees()
{
    return imu->getYawDegrees();
}

float GlobalState_t::getPitchDegrees()
{
    return imu->getPitchDegrees();
}


/***********************************************************************************************/
/******************************* A Little Higher Level Get APIs ********************************/
/***********************************************************************************************/

vector3D_t GlobalState_t::getVelocityAbs()
{
    return this->getVelocity();   // CHANGE THIS
}

vector3D_t GlobalState_t::getVelocityRel()
{
    return this->getVelocity();   // CHANGE THIS
}

vector3D_t GlobalState_t::getVelocity()   // CHANGE THIS
{
    return this->locator->getVelocity();
}

vector3D_t GlobalState_t::getPosition()   // CHANGE THIS
{
    vector3D_t pos;
    return pos;
}

GeoPoint_t GlobalState_t::getLocation()   // CHANGE THIS
{
    return this->locator->getLocation();
}
