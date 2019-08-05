
#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>
#include <thread> // std::thread
#include <mutex>
#include <stdexcept>
#include <future>

#include "Sensors/Sensors.hpp"
#include "Sensors/Location.hpp"
#include "Sensors/InertialMeasurement.hpp"
//#include "common.hpp"

float GlobalState_t::get_X_Coordinate()
{
    try
    {
        return locator->get_X_Coordinate();
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalState_t::get_X_Coordinate>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}

float GlobalState_t::get_Y_Coordinate()
{
    try
    {
        return locator->get_Y_Coordinate();
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalState_t::get_Y_Coordinate>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}

float GlobalState_t::get_Z_Coordinate()
{
    try
    {
        return locator->get_Z_Coordinate();
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalState_t::get_Z_Coordinate>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}

float GlobalState_t::get_X_VelocityRel()
{
    try
    {
        return locator->get_X_VelocityRel();
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalState_t::get_X_VelocityRel>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}

float GlobalState_t::get_Y_VelocityRel()
{
    try
    {
        return locator->get_Y_VelocityRel();
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalState_t::get_X_VelocityRel>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
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
    try
    {
        return locator->get_Z_Coordinate();
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalState_t::getAltitude>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}

float GlobalState_t::getHeadingDegrees() // Gives in Degrees
{
    try
    {
        return this->getYawDegrees();
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalState_t::getHeadingDegrees>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}

float GlobalState_t::getHeading()
{
    try
    {
        return this->getYaw();
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalState_t::getHeading>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}

/* ------------------------------------------------------------------------------------------------------------------------ */
/*------------------------------------------------- Rotation related stuff -------------------------------------------------*/
/* ------------------------------------------------------------------------------------------------------------------------ */

float GlobalState_t::getYaw()
{
    try
    {
        return imu->getYaw();
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalState_t::getYaw>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}

float GlobalState_t::getRoll()
{
    try
    {
        return imu->getRoll();
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalState_t::getRoll>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}

float GlobalState_t::getPitch()
{
    try
    {
        return imu->getPitch();
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalState_t::getPitch>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}

float GlobalState_t::getYawDegrees()
{
    try
    {
        return imu->getYawDegrees();
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalState_t::getYawDegrees>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}

float GlobalState_t::getRollDegrees()
{
    try
    {
        return imu->getRollDegrees();
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalState_t::getRollDegrees>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}

float GlobalState_t::getPitchDegrees()
{
    try
    {
        return imu->getPitchDegrees();
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalState_t::getPitchDegrees>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}

/***********************************************************************************************/
/******************************* A Little Higher Level Get APIs ********************************/
/***********************************************************************************************/

vector3D_t GlobalState_t::getVelocityAbs()
{
    return this->getVelocity(); // CHANGE THIS
}

vector3D_t GlobalState_t::getVelocityRel()
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

vector3D_t GlobalState_t::getVelocity() // CHANGE THIS
{
    try
    {
        return this->locator->getVelocity();
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalState_t::getVelocity>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return vector3D_t(0,0,0);
}

GeoPoint_t GlobalState_t::getLocation() // CHANGE THIS
{
    try
    {
        return this->locator->getLocation();
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalState_t::getLocation>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return GeoPoint_t(0, 0, 0);
}
