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
    return rpcStub->call("setHeading", heading).as<int>();
}