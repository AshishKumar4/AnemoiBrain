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

class Controller // An Abstract Class to provide APIs for Controls
{
  protected:
  public:
    Controller()
    {
    }
    ~Controller()
    {
    }

    /*      APIs for channel Controls       */
    virtual void setThrottle(int val) = 0;
    virtual void setPitch(int val) = 0;
    virtual void setYaw(int val) = 0;
    virtual void setRoll(int val) = 0;
    virtual void setAux(int channel, int val) = 0;
    virtual void callRAPI(int code, int val) = 0;

    virtual void printChannels() = 0;
    /* APIs to get Data */
    virtual int startSensorsServer() = 0;
    virtual DroneState_t *getState() = 0;
    virtual DronePosition_t *getPosition() = 0;
    virtual int startCameraServer() = 0;
    virtual int *getCameraView(int id) = 0;
    virtual int *getCameraView(DroneCamera_t *camera) = 0;
};