
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

namespace // Anonymous Namespace
{

inline vector3D_t eulerFromQuaternion(quaternion_t orien)
{
    try
    {
        vector3D_t oo;
        /*
        Quaternion to Euler
    */
        //std::cout << ">>>" << orien << "<<<" << std::endl;
        double heading = 0, roll, pitch, yaw;
        double ysqr = orien.y() * orien.y();

        // roll (x-axis rotation)
        double t0 = +2.0f * (orien.w() * orien.x() + orien.y() * orien.z());
        double t1 = +1.0f - 2.0f * (orien.x() * orien.x() + ysqr);
        roll = std::atan2(t0, t1);

        // pitch (y-axis rotation)
        double t2 = +2.0f * (orien.w() * orien.y() - orien.z() * orien.x());
        t2 = ((t2 > 1.0f) ? 1.0f : t2);
        t2 = ((t2 < -1.0f) ? -1.0f : t2);
        pitch = std::asin(t2);

        // yaw (z-axis rotation)
        double t3 = +2.0f * (orien.w() * orien.z() + orien.x() * orien.y());
        double t4 = +1.0f - 2.0f * (ysqr + orien.z() * orien.z());
        yaw = std::atan2(t3, t4);
        //heading = yaw;
        //printf("->Roll %f, Pitch %f, Yaw %f", roll, pitch, yaw);
        //printf("->Heading: { %f %f}", orien.z(), orien.w());
        oo.x = pitch;
        oo.y = roll;
        oo.z = yaw; // // 0, 360);
        return oo;
    }
    catch (const std::future_error &e)
    {
        std::cout << "<GlobalState_t::eulerFromQuaternion>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return vector3D_t(0,0,0);
}
}; // namespace

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
        h = yawBuffer;
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
        h = rollBuffer;
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
        h = pitchBuffer;
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

quaternion_t AirSim_IMU_t::getOrientation()
{
    try
    {
        auto orien = client->getMultirotorState().getOrientation();
        tmporien = quaternion_t(orien.w(), orien.x(), orien.y(), orien.z());
        return tmporien;
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

#endif