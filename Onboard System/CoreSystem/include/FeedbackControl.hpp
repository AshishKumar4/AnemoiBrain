#pragma once

#include "common.hpp"

namespace ControllerInterface
{

/*
    GPS/Barometer/Compass assisted
    All SI Units, Meters, Degrees
    Forward pitch direction in Y Axis, Left Right is X axis, up is Z
*/

float getPathLength();
float getPathDeviation();

namespace FeedbackControl
{

}

int setGazeOn(GeoPoint_t destination);
int setLinearPath(GeoPoint_t start, GeoPoint_t destination);
int moveSavedPath();

int setFeedbackAltitude(float altitude);
int setFeedbackYaw(float heading);

}