#pragma once 

#include "common.hpp"

#if defined (MODE_REALDRONE)

namespace InertialSensorFusion
{
quaternion_t 	getOrientation();
vector3D_t 		getOrientationEuler();
vector3D_t 		getAcceleration();
float 			getAltitude();

int InertialSensor_Fusion_init(int argc, char **argv);
}

#endif 