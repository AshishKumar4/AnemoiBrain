#pragma once

#include "../common.hpp"

// #include "Sensors/Location.hpp"
// #include "Sensors/InertialMeasurement.hpp"

class StateEstimator_t
{

protected: 
	// virtual quaternion_t 	getOrientationUnbuffered();
	// virtual vector3D_t 		getOrientationEulerUnbuffered();
	// virtual vector3D_t 		getAccelerationUnbuffered();
	// virtual vector3D_t 		getVelocityRelUnbuffered();
	// virtual vector3D_t 		getVelocityAbsUnbuffered();
	// virtual GeoPoint_t 		getLocalCoordinatesUnbuffered();

public:
	StateEstimator_t()
	{
	}

	virtual float get_X_Coordinate()	{return getLocalCoordinates().x;}
	virtual float get_Y_Coordinate()	{return getLocalCoordinates().y;}
	virtual float get_Z_Coordinate()	{return getLocalCoordinates().z;}
	virtual GeoPoint_t getLocalCoordinates() = 0;

	virtual float get_X_VelocityRel() 	{return getVelocityRel().x;}
	virtual float get_Y_VelocityRel()	{return getVelocityRel().y;}
	virtual float get_Z_VelocityRel()	{return getVelocityRel().z;}
	virtual vector3D_t getVelocityRel() = 0;

	virtual float get_X_VelocityAbs() 	{return getVelocityAbs().x;}
	virtual float get_Y_VelocityAbs() 	{return getVelocityAbs().y;}
	virtual float get_Z_VelocityAbs() 	{return getVelocityAbs().z;}
	virtual vector3D_t getVelocityAbs() = 0;

	virtual vector3D_t getVelocity() = 0;

	virtual float getAltitude()			{return getLocalCoordinates().z;}
	virtual float getHeadingDegrees()	{return getConventionalDegrees(getHeading());}
	virtual float getHeading()			{return getOrientationEuler().z;}

	virtual float getYaw() 				{return getOrientationEuler().z;}
	virtual float getRoll() 			{return getOrientationEuler().y;}
	virtual float getPitch() 			{return getOrientationEuler().z;}
	virtual vector3D_t getOrientationEuler() {return eulerFromQuaternion(getOrientation());}
	virtual quaternion_t getOrientation() = 0;

	virtual float getYawDegrees()			{return getConventionalDegrees(getYaw());}
	virtual float getRollDegrees()			{return getConventionalDegrees(getRoll());}
	virtual float getPitchDegrees()			{return getConventionalDegrees(getPitch());}

	virtual vector3D_t getAcceleration() = 0;
};

#if defined(MODE_AIRSIM)

class AirSim_StateEstimator_t : public StateEstimator_t
{
public: 
	AirSim_StateEstimator_t()
	{

	}

	GeoPoint_t getLocalCoordinates();
	vector3D_t getVelocityRel();
	vector3D_t getVelocityAbs();
	vector3D_t getVelocity();
	vector3D_t getOrientationEuler();
	quaternion_t getOrientation();
	vector3D_t getAcceleration();
};


#elif defined(MODE_REALDRONE)

class Real_StateEstimator_t : public StateEstimator_t
{
public: 
	Real_StateEstimator_t()
	{

	}

	GeoPoint_t getLocalCoordinates();
	vector3D_t getVelocityRel();
	vector3D_t getVelocityAbs();
	vector3D_t getVelocity();
	// vector3D_t getOrientationEuler();
	quaternion_t getOrientation();
	vector3D_t getAcceleration();
};


int Sensor_Fusion_init(int argc, char **argv);

#endif
