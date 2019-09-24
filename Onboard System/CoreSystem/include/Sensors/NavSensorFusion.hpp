#pragma once 

#include "common.hpp"

#if defined (MODE_REALDRONE)

namespace NavSensorFusion
{
vector3D_t getVelocityAbs();
vector3D_t getVelocityRel();
GeoPoint_t getLocalCoordinates();
}

#endif 