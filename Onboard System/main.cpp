// Our Main File
// GardienOnboard Executable

#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>

#include "AbstractServer.hpp"
#include "ControllerInterface.hpp"

#include "ControlServer.hpp"
#include "SensorsServer.hpp"

int main(int argc, char *argv[])
{
#ifndef DRONELESS_LOCAL_TEST
    ControllerInterface::ControllerInterface_init(argc, (char **)argv); // Maybe lower levels can make use of command line args
#endif
    Onboard::ControlServer_init(argc, (char **)argv);
    //Onboard::SensorsServer_init(argc, (char **)argv);
    while(1);
    return 0;
}