#pragma once

#include "common.hpp"

namespace ControllerInterface
{
int setGazeOn(GeoPoint_t destination);
int setLinearPath(GeoPoint_t start, GeoPoint_t destination, float cruise_velocity = 15, float final_velocity = 0, float clamp_factor = 40);
int moveSavedPath();

int setFeedbackAltitude(float altitude);
int setFeedbackYaw(float heading);

float getForwardVelocity();
float getDesiredVelocity();
void HeadlessHover(float val);
}