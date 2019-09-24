#pragma once

#include "common.hpp"
#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>
#include <thread> // std::thread
#include <mutex>
#include <queue>
#include <atomic>

class GeoEstimator_t
{

public:
    GeoEstimator_t()
    {
    }

    ~GeoEstimator_t()
    {
    }

	virtual GeoPoint_t getLocalCoordinates() = 0;
	virtual vector3D_t getVelocity() = 0; // ==> Returns absolute world velocity
	virtual GeoCoords_t getRawCoordinates() = 0;
};

#if defined(MODE_REALDRONE)

class Real_GeoEstimator_t : public GeoEstimator_t
{
	public:
	Real_GeoEstimator_t()
	{

	}

	GeoPoint_t getLocalCoordinates();
	vector3D_t getVelocity(); // ==> Returns absolute world velocity
	GeoCoords_t getRawCoordinates();
};

#endif

