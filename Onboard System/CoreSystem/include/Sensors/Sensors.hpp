#pragma once 

#include "../common.hpp"

#include "Sensors/Location.hpp"
#include "Sensors/InertialMeasurement.hpp"

class GlobalState_t
{
    GlobalLocator_t* locator;
    InertialMeasurement_t* imu;

public: 
    GlobalState_t(GlobalLocator_t *locator, InertialMeasurement_t* imu)
    {
        this->locator = locator;
        this->imu = imu;
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

    float getAltitude();
    float getHeadingDegrees();
    float getHeading();

    float getYaw();
    float getRoll();
    float getPitch();

    float getYawDegrees();
    float getRollDegrees();
    float getPitchDegrees();

    vector3D_t getVelocityAbs();
    vector3D_t getVelocityRel();
    vector3D_t getVelocity();
    GeoPoint_t getLocation();
};
