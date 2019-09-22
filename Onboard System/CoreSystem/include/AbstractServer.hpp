#pragma once

#include <string.h>

#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>
#include <thread> // std::thread
#include <mutex>
#include <functional>
#include <atomic>

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
// typedef void (*func_vi_t)(int); //void function pointer

enum SERVER_STATUS
{
	NOT_READY,
	READY,
	WORKING,
	EXITED
};

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
    static void ChannelLogic(int i, int fd, AbstractServer* thisObj);
  public:
	bool connectionBroken;
    int (*ExceptionHandler)();
    int (*ResumeHandler)();
    int PORT_BASE;

    std::vector<func_iii_t> ChannelOperators;
    std::vector<func_iii_t> ChannelInitializers;

	std::vector<int> 		logicStatus;
    //std::vector<std::function<void(int)>> ChannelListeners;

    explicit AbstractServer(int portBase = 8400);
    explicit AbstractServer(AbstractServer* obj);
    ~AbstractServer();

	void triggerFault();
	void manageFault();

    void AddChannels(int index, func_iii_t maincode, func_iii_t initializer);      // Create now and launch later
    void CreateChannels(int index, func_iii_t maincode, func_iii_t initializer, bool initPort = true);   // Create and launch simultaneously
    
    int LaunchThreads(bool initPort = true);
    int JoinThreads();

	void logicSynchronize();
};
} // namespace Onboard