// Our Main File
// GardienOnboard Executable

#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>

#include "AbstractServer.hpp"
#include "ControllerInterface.hpp"

#include "ControlServer.hpp"

int main(int argc, char *argv[])
{
    Onboard::ControlServer_init(argc, (char **)argv);
    return 0;
}