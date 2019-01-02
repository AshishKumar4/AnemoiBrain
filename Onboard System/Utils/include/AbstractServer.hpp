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
    static void ChannelListener(int i, AbstractServer* thisObj);
    static void ChannelLogic(int i, int j, int fd, AbstractServer* thisObj);
  public:
    int PORT_BASE;
    std::vector<std::vector<func_iii_t>> ChannelOperators;
    std::vector<func_iii_t> ChannelInitializers;
    std::vector<func_vi_t> ChannelListeners;

    AbstractServer(int portBase = 8400);
    AbstractServer(AbstractServer* obj);
    ~AbstractServer();

    void AddChannels(int index, func_iii_t function, func_iii_t initializer, int subchannel = 0);      // Create now and launch later
    void CreateChannels(int index, func_iii_t function, func_iii_t initializer, int subchannel = 0, bool initPort = true);   // Create and launch simultaneously
    
    int LaunchThreads(bool initPort = true);
    int JoinThreads();
};
} // namespace Onboard