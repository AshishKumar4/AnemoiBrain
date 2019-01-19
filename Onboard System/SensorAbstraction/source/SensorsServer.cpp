#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>

#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>
#include <thread> // std::thread
#include <mutex>

#include "AbstractServer.hpp"
#include "Sensors.h"

namespace Onboard
{
namespace Sensors
{

} // namespace Sensors
} // namespace Onboard
  /*
int main(int argc, char const *argv[])
{
    char *buff = new char[1024];
#ifndef DRONELESS_LOCAL_TEST
    BasicControls_init(argc, (char **)argv); // Maybe lower levels can make use of command line args
#endif
    Onboard::AbstractServer SensorsServer(8400);
    for (int i = 0; i < 6; i++)
    {
        Onboard::Sensors::SensorsChannelBuffer[i] = new char[4096];
        SensorsServer.AddChannels(Onboard::Sensors::SensorsListeners);
    }
    ControlServer.LaunchThreads();
    while (1)
        std::cout << "Some Error!";
    ;
    return 0;
}
*/