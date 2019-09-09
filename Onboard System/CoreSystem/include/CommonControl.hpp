#pragma once

#include "common.hpp"

namespace ControllerInterface
{
int setGazeOn(GeoPoint_t destination);
int setLinearPath(GeoPoint_t start, GeoPoint_t destination);
int moveSavedPath();

int setFeedbackAltitude(float altitude);
int setFeedbackYaw(float heading);

float getForwardVelocity();
float getDesiredVelocity();
void HeadlessMoveTowardsTarget(float val);
}