
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
// #include "Sensors/Location.hpp"
// #include "Sensors/InertialMeasurement.hpp"

#include "common.hpp"

#if defined(MODE_AIRSIM)

namespace
{
quaternion_t tmporien;
vector3D_t tmpVelocity;
GeoPoint_t tmpLocation;
} // namespace

extern GeoPoint_t AIRSIM_location;
extern vector3D_t AIRSIM_velocity;
extern vector3D_t AIRSIM_velocityAbs;
extern vector3D_t AIRSIM_velocityRel;
extern vector3D_t AIRSIM_euleroritentation;

extern quaternion_t AIRSIM_oritentation;
extern vector3D_t AIRSIM_euleroritentation;

GeoPoint_t AirSim_StateEstimator_t::getLocalCoordinates()
{
	return AIRSIM_location;
}

vector3D_t AirSim_StateEstimator_t::getVelocityRel()
{
	return AIRSIM_velocityRel;
}

vector3D_t AirSim_StateEstimator_t::getVelocityAbs()
{
	return AIRSIM_velocityAbs;
}

vector3D_t AirSim_StateEstimator_t::getVelocity()
{
	return AIRSIM_velocity;
}

vector3D_t AirSim_StateEstimator_t::getOrientationEuler()
{
	return AIRSIM_euleroritentation;
}

quaternion_t AirSim_StateEstimator_t::getOrientation()
{
	return AIRSIM_oritentation;
}

vector3D_t AirSim_StateEstimator_t::getAcceleration()
{
	return vector3D_t(0, 0, 0);
}

#elif defined(MODE_REALDRONE)

#include "Sensors/InertialSensorFusion.hpp"
#include "Sensors/NavSensorFusion.hpp"

GeoPoint_t Real_StateEstimator_t::getLocalCoordinates()
{
	return NavSensorFusion::getLocalCoordinates();
}

vector3D_t Real_StateEstimator_t::getVelocityRel()
{
	return NavSensorFusion::getVelocityRel();
}

vector3D_t Real_StateEstimator_t::getVelocityAbs()
{
	return NavSensorFusion::getVelocityAbs();
}

vector3D_t Real_StateEstimator_t::getVelocity()	// By default, GetVelocity gives relative velocity
{
	return NavSensorFusion::getVelocityRel();
}

// vector3D_t Real_StateEstimator_t::getOrientationEuler()
// {

// }

quaternion_t Real_StateEstimator_t::getOrientation()
{
	return InertialSensorFusion::getOrientation();
}

vector3D_t Real_StateEstimator_t::getAcceleration()
{
	return InertialSensorFusion::getAcceleration();
}

#endif