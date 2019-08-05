#include <iostream> 
#include <bits/stdc++.h>

#include <rpc/client.h>

#include "Drone.hpp"

#include "Controls/DirectControls.hpp"
#include "Sensors/Sensors.hpp"

Drone::Drone(std::string ip, int portBase): DirectController(ip, portBase), Sensors(ip, portBase+100)
{
    std::cout<<"Initiating Drone Connection on IP "<<ip<<", May the force be with you\n";
    // PortBase - 1 port would be for RPC channel
    rpcStub = new rpc::client(ip, portBase-1);
}

int Drone::setHeading(float heading)
{
    printf("%f", heading);
    return rpcStub->call("setHeading", heading).as<int>();
}

int Drone::setRollAngle(float angle)
{
    printf("%f", angle);
    return rpcStub->call("setRollAngle", angle).as<int>();
}

int Drone::setPitchAngle(float angle)
{
    printf("%f", angle);
    return rpcStub->call("setPitchAngle", angle).as<int>();
}

int Drone::setAltitude(float altitude)
{
    printf("%f", altitude);
    return rpcStub->call("setAltitude", altitude).as<int>();
}

int Drone::toggleAutoActuator(char type)
{
    return rpcStub->call("toggleAutoActuator", type).as<int>();
}

int Drone::gotoLocation(float x, float y, float z)
{
    return rpcStub->call("gotoLocation", x, y, z).as<int>();
}