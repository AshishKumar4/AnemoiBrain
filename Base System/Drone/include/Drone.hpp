#pragma once

#include <iostream>

#include <rpc/client.h>

#include "Controls/DirectControls.hpp"
#include "Sensors/Sensors.hpp"

class Drone : public DirectController, public Sensors
{
    rpc::client* rpcStub;
  protected:
  public:
    Drone(std::string ip = "0.0.0.0", int portBase = 8400);

    /*
        Provide High level Control APIs
    */
    int setHeading(float heading);
    int setRollAngle(float angle);
    int setPitchAngle(float angle);
    int setAltitude(float altitude);
    int toggleAutoActuator(char type);
    int gotoLocation(float x, float y, float z);
    /*
        Provide High level Sensor Telemetry
   */
};