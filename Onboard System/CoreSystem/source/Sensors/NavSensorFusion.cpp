#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>
#include <thread> // std::thread
#include <mutex>
#include <stdexcept>
#include <future>

#include "common.hpp"

#include "Sensors/Sensors.hpp"
#include "Sensors/drivers/GeoEstimator.hpp"
#include "Sensors/drivers/InertialEstimator.hpp"

#include "Sensors/InertialSensorFusion.hpp"

/***********************************************************************************************/
/*************************** Systems to generate Sensor Fusion Data ****************************/
/***********************************************************************************************/

#if defined(MODE_REALDRONE)

namespace NavSensorFusion 
{

vector3D_t getVelocityAbs()
{
	return vector3D_t(0,0,0);
}

vector3D_t getVelocityRel()
{
	return vector3D_t(0,0,0);
}

GeoPoint_t getLocalCoordinates()
{
	return GeoPoint_t(0,0,InertialSensorFusion::getAltitude());
}
}

#endif 