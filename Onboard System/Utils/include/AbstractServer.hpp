#pragma once

#include <string.h>

#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>
#include <thread> // std::thread
#include <mutex>

namespace Onboard
{

/* ------------------------------------------------------------------------------------------------------------------------ */
/* --------------------------------------------------Some Configurations--------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

/*
    STREAM_PROTOCOL_1 Refers to conventional failproof streaming of values as packets, encapsulated and later stripped off on the other hand 
    STREAM_PROTOCOL_2 Refers to newer, faster and lightweight but simplest streaming, stream of simple bytes. This isn't failproof and errors 
                        may be entroduced.
    */
//#define STREAM_PROTOCOL_1 // FailProof, Encapsulate
//#define STREAM_PROTOCOL_2 // Simple Byte Stream, Faster and lightweight

/* ------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------Permanent Configurations------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------ */
/*
#if defined(STREAM_PROTOCOL_1)
#undef STREAM_PROTOCOL_2
#elif defined(STREAM_PROTOCOL_2)
#undef STREAM_PROTOCOL_1
#endif
*/
/* ------------------------------------------------------------------------------------------------------------------------ */
/* ---------------------------------------------------Main Definitions----------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */
    
typedef int (*func_iii_t)(int, int); //void function pointer
typedef void (*func_vi_t)(int); //void function pointer

class AbstractServer
{
    std::vector<std::thread *> ListenerThreads;

  protected:
    std::vector<int> server_fd;
    std::vector<struct sockaddr_in*> addresses;
    //struct sockaddr_in **addresses;
    std::mutex smtx;

    int SetupChannel(int port, int channel); // This would create a port for a particular channel
    static void ChannelListeners(int i, AbstractServer* thisObj);
  public:
    int PORT_BASE;
    std::vector<func_iii_t> ChannelOperators;

    AbstractServer(int portBase = 8400);
    ~AbstractServer();

    void AddChannels(func_iii_t function);      // Create now and launch later
    void CreateChannels(func_iii_t function);   // Create and launch simultaneously
    int LaunchThreads();
};
} // namespace Onboard