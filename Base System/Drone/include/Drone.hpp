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
    /*
        Provide High level Sensor Telemetry
   */
};