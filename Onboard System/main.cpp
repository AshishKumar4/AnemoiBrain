// Our Main File
// GardienOnboard Executable

#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>

#include <rpc/server.h>

#include "AbstractServer.hpp"
#include "ControllerInterface.hpp"

#include "ControlServer.hpp"
#include "SensorsServer.hpp"

rpc::server* rpcStub;

int main(int argc, char *argv[])
{
#ifndef DRONELESS_LOCAL_TEST
    /*
        New Feature, RPC Communication!!! 
    */
    ControllerInterface::ControllerInterface_init(argc, (char **)argv); // Maybe lower levels can make use of command line args

    int portBase = (argc > 1) ? stoi(std::string(argv[3])) : 8400;
    rpcStub = new rpc::server(portBase - 1);
    rpcStub->bind("setHeading", &(ControllerInterface::setHeading));
    rpcStub->async_run(1);
#endif
    Onboard::ControlServer_init(argc, (char **)argv);
    //Onboard::SensorsServer_init(argc, (char **)argv);
    while(1);
    return 0;
}