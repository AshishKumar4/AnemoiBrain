
#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>
#include <thread> // std::thread
#include <mutex>
#include <stdexcept>
#include <math.h>

#include "common.hpp"
#include "Sensors/InertialMeasurement.hpp"

namespace // Anonymous Namespace
{

vector3D_t eulerFromQuaternion(quaternion_t orien)
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
};

vector3D_t InertialMeasurement_t::getEulerOrientation() // Returns Euler angle orientation
{
    return eulerFromQuaternion(this->getOrientation());
}

float InertialMeasurement_t::getYaw() // Gives in Radians
{
    float h = this->getEulerOrientation().z;
    return h;
}

float InertialMeasurement_t::getRoll() // Gives in Radians
{
    float h = this->getEulerOrientation().y;
    return h;
}

float InertialMeasurement_t::getPitch() // Gives in Radians
{
    float h = this->getEulerOrientation().x;
    return h;
}

/*
    This is our convention, counter clockwise, 0 to 360 in every direction
*/

float clamp(float val, float imin, float imax, float omin, float omax)
{
    float aa = omin + (((omax - omin) / (imax - imin)) * (val - imin));
    return aa;
}

float getConventionalDegrees(float rads)
{
    // 0 -> North
    // 90 -> east
    // 180 -> south
    // 270 ->west
    float h = clamp(rads, -3.14159265358979323846, 3.14159265358979323846, 180, -180);
    if (h < 0)
    {
        h = 360 + h;
    }
    return h;
}

float InertialMeasurement_t::getYawDegrees()
{
    return getConventionalDegrees(this->getYaw());
}

float InertialMeasurement_t::getRollDegrees()
{
    return getConventionalDegrees(this->getRoll());
}

float InertialMeasurement_t::getPitchDegrees()
{
    return getConventionalDegrees(this->getPitch());
}


/* ------------------------------------------------------------------------------------------------------------------------ */
/*------------------------------------------------------- AirSim_IMU -------------------------------------------------------*/
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined MODE_AIRSIM

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "rpc/server.h"

quaternion_t AirSim_IMU_t::getOrientation()
{
    auto orien = client->getMultirotorState().getOrientation();
    return quaternion_t(orien.w(), orien.x(), orien.y(), orien.z());
}

#endif