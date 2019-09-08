
#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>
#include <thread> // std::thread
#include <mutex>
#include <stdexcept>
#include <math.h>
#include <future>

//#include "common.hpp"
#include "Sensors/InertialMeasurement.hpp"

vector3D_t InertialMeasurement_t::getEulerOrientation() // Returns Euler angle orientation
{
    try
    {
        return eulerFromQuaternion(this->getOrientation());
    }

    catch (const std::future_error &e)
    {
        std::cout << "<InertialMeasurement_t::getEulerOrientation>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return vector3D_t(0, 0, 0);
}


void InertialMeasurement_t::bufferWriter(InertialMeasurement_t *imu)
{
    while (1)
    {
        try
        {
            /* code */

            /*imu->rollLock.lock();
            imu->pitchLock.lock();
            imu->yawLock.lock();*/

            vector3D_t eulerOrient = eulerFromQuaternion(imu->getOrientation());
            imu->yawBuffer = (eulerOrient.z);
            imu->rollBuffer = (eulerOrient.y);
            imu->pitchBuffer = (eulerOrient.x);
            //std::cout << "<" << eulerOrient.z << ">" ;

            /*imu->rollLock.unlock();
            imu->pitchLock.unlock();
            imu->yawLock.unlock();*/
            //std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
        catch (const std::future_error &e)
        {
            std::cout << "<InertialMeasurement_t::bufferWriter>Caught a future_error with code \"" << e.code()
                      << "\"\nMessage: \"" << e.what() << "\"\n";
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }
}

float InertialMeasurement_t::getYaw() // Gives in Radians
{
    float h = 0; // = this->getEulerOrientation().z;
    //this->yawLock.lock();
    try
    {
		h = this->getEulerOrientation().z;
    }
    catch (const std::future_error &e)
    {
        std::cout << "<InertialMeasurement_t::getYaw>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    //this->yawLock.unlock();
    //std::cout << "---" << h << "---" ;
    return h;
}

float InertialMeasurement_t::getRoll() // Gives in Radians
{
    float h = 0; // = this->getEulerOrientation().y;
    //this->rollLock.lock();
    try
    {
        h = this->getEulerOrientation().y;//= rollBuffer;
    }
    catch (const std::future_error &e)
    {
        std::cout << "<InertialMeasurement_t::getRoll>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    //this->rollLock.unlock();
    return h;
}

float InertialMeasurement_t::getPitch() // Gives in Radians
{
    float h = 0; // = this->getEulerOrientation().x;
    //this->pitchLock.lock();
    try
    {
        h = this->getEulerOrientation().x;//pitchBuffer;
    }
    catch (const std::future_error &e)
    {
        std::cout << "<InertialMeasurement_t::getPitch>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    //this->pitchLock.unlock();
    return h;
}

float InertialMeasurement_t::getYawDegrees()
{
    try
    {
        return getConventionalDegrees(this->getYaw());
    }
    catch (const std::future_error &e)
    {
        std::cout << "<InertialMeasurement_t::getYawDegrees>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}

float InertialMeasurement_t::getRollDegrees()
{
    try
    {
        return getConventionalDegrees(this->getRoll());
    }

    catch (const std::future_error &e)
    {
        std::cout << "<InertialMeasurement_t::getRollDegrees>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}

float InertialMeasurement_t::getPitchDegrees()
{
    try
    {
        return getConventionalDegrees(this->getPitch());
    }
    catch (const std::future_error &e)
    {
        std::cout << "<InertialMeasurement_t::getPitchDegrees>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}

/* ------------------------------------------------------------------------------------------------------------------------ */
/*------------------------------------------------------- AirSim_IMU -------------------------------------------------------*/
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined MODE_AIRSIM

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "rpc/server.h"

namespace
{
quaternion_t tmporien;
}

extern quaternion_t AIRSIM_oritentation;
extern vector3D_t AIRSIM_euleroritentation;

quaternion_t AirSim_IMU_t::getOrientation()
{
    try
    {
        return AIRSIM_oritentation;
    }
    catch (const std::future_error &e)
    {
        std::cout << "<AirSim_IMU_t::getOrientation>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
        return tmporien; //getOrientation();
    }
    catch (const std::exception &e)
    {
        std::cout << e.what() << '\n';
    }
}

vector3D_t AirSim_IMU_t::getEulerOrientation()
{
    try
    {
        //tmporien = AIRSIM_euleroritentation;
        return AIRSIM_euleroritentation;
    }
    catch (const std::future_error &e)
    {
        std::cout << "<AirSim_IMU_t::getOrientation>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
        return AIRSIM_euleroritentation; //getOrientation();
    }
    catch (const std::exception &e)
    {
        std::cout << e.what() << '\n';
    }
}

#elif defined(MODE_REALDRONE)

extern quaternion_t Real_IMU_oritentation;
extern vector3D_t Real_IMU_euleroritentation;

quaternion_t Real_IMU_t::getOrientation()
{
    return Real_IMU_oritentation;
}

vector3D_t Real_IMU_t::getEulerOrientation()
{
    return Real_IMU_euleroritentation;
}
#endif