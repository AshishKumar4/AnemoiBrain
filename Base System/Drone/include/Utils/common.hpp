#pragma once

#include "iostream"
#include "vector"

using namespace std;

class DronePosition_t
{
  public:
    DronePosition_t()
    {
    }
};

class DroneState_t
{
  public:
    DroneState_t()
    {
    }
};

class DroneSensors_t
{
  public:
};

class DroneCamera_t : public DroneSensors_t
{
  public:
    DroneCamera_t()
    {
    }

    int *getDisparity(int cam1, int cam2)
    {
        return NULL;
    }
};

class DroneIMU_t : public DroneSensors_t
{
  public:
    DroneIMU_t()
    {
    }
};
