// Our Main File
// GardienOnboard Executable

#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
#include <future>

#include <rpc/server.h>

#include "AbstractServer.hpp"
#include "ControllerInterface.hpp"

#include "ControlServer.hpp"
#include "SensorsServer.hpp"

rpc::server *rpcStub;

int main(int argc, char *argv[])
{
    try
    {
#ifndef DRONELESS_LOCAL_TEST
        /*
        New Feature, RPC Communication!!! 
    */
        ControllerInterface::ControllerInterface_init(argc, (char **)argv); // Maybe lower levels can make use of command line args

        int portBase = (argc >= 3) ? stoi(std::string(argv[3])) : 8400;
        rpcStub = new rpc::server(portBase - 1);
        rpcStub->bind("setHeading", &(ControllerInterface::setHeading));
        rpcStub->bind("setRollAngle", &(ControllerInterface::setAutoRoll));
        rpcStub->bind("setPitchAngle", &(ControllerInterface::setAutoPitch));

        //rpcStub->bind("setDestinationX", &(ControllerInterface::setDestinationX));
        //rpcStub->bind("setDestinationY", &(ControllerInterface::setDestinationY));
        rpcStub->bind("setAltitude", &(ControllerInterface::setAltitude));
        //rpcStub->bind("setDestination", &(ControllerInterface::setDestination));
        rpcStub->bind("toggleAutoActuator", &(ControllerInterface::toggleFeedbackControllers));
        rpcStub->bind("gotoLocation", &(ControllerInterface::gotoLocation));
        rpcStub->async_run(1);  
#endif
        //std::thread *launch_ActuationControllersThread = new std::thread(ControllerInterface::launch_ActuationControllers);
        Onboard::ControlServer_init(argc, (char **)argv);
        ControllerInterface::launch_ActuationControllers();
        ControlServer->JoinThreads();
        while(1);
        //Onboard::SensorsServer_init(argc, (char **)argv); //*/
    }
    catch (const std::future_error &e)
    {
        std::cout << "<main>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (std::exception &e)
    {
        std::cout << "Some Error in toggleAutoActuators!" << e.what();
        fflush(stdout);
        while(1);
    }
    while (1)
        ;
    return 0;
}
