#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>

#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>
#include <thread> // std::thread

#include "DirectControls.hpp"

/* ------------------------------------------------------------------------------------------------------------------------ */
/* --------------------------------------------------Some Configurations--------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

/*
  STREAM_PROTOCOL_1 Refers to conventional failproof streaming of values as packets, encapsulated and later stripped off on the other hand 
  STREAM_PROTOCOL_2 Refers to newer, faster and lightweight but simplest streaming, stream of simple bytes. This isn't failproof and errors 
                    may be entroduced.
*/
//#define STREAM_PROTOCOL_1 // FailProof, Encapsulate
#define STREAM_PROTOCOL_2 // Simple Byte Stream, Faster and lightweight

//#define DRONELESS_LOCAL_TEST

/* ------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------Permanent Configurations------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined(STREAM_PROTOCOL_1)
#undef STREAM_PROTOCOL_2
#elif defined(STREAM_PROTOCOL_2)
#undef STREAM_PROTOCOL_1
#endif

/* ------------------------------------------------------------------------------------------------------------------------ */
/* -----------------------------------------------------Main Program------------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

using namespace std;

const char HANDSHAKE_IN_MSG[] = "Hello Gardien!";
const char HANDSHAKE_OUT_MSG[] = "Hello Overloard!";
int opt = 1;

void DirectController::InitSequence()
{
  for (int i = 0; i < CHANNEL_COUNT; i++)
  {
    send(server_fd[i], HANDSHAKE_IN_MSG, strlen(HANDSHAKE_IN_MSG), 0);
    // TODO: Place mechanism to recieve back handshake and if not matching, Panic!
    char *buff = (char *)malloc(1024);
    int valread = read(server_fd[i], buff, 1024);
    if (strncmp(buff, HANDSHAKE_OUT_MSG, strlen(HANDSHAKE_OUT_MSG)))
    {
      std::cout << "Gardien Could not establish Connection / Handshake Failure...\n";
      throw "Handshake Failed!";
    }
    printf("Handshake Successfull, Connection Established!\n");
  }
  disarm();
  balance();
  disarm();
  cout << "Initialization Sequence Completed...\n";
}

DirectController::DirectController(std::string ip, int portBase)
{
  ConnectChannel(ip, portBase + 0, 0); //PORT_THROTTLE
  ConnectChannel(ip, portBase + 1, 1); //PORT_PITCH
  ConnectChannel(ip, portBase + 2, 2); //PORT_ROLL
  ConnectChannel(ip, portBase + 3, 3); //PORT_YAW
  ConnectChannel(ip, portBase + 4, 4); //PORT_AUX1
  ConnectChannel(ip, portBase + 5, 5); //PORT_AUX2
  ConnectChannel(ip, portBase + 6, 6); //PORT_AUX3
  ConnectChannel(ip, portBase + 7, 7); //PORT_AUX4
  ConnectChannel(ip, portBase + 8, 8); //RAPI_INVOKER

  InitSequence();
}

DirectController::~DirectController()
{
}

int DirectController::ConnectChannel(std::string ip, int port, int channel) // This would create a port for a particular channel
{
  int sfd;
  struct sockaddr_in *address = new struct sockaddr_in;
  // Creating socket file descriptor
  if ((sfd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
  {
    perror("socket failed");
    exit(EXIT_FAILURE);
  }

  address->sin_family = AF_INET;
  address->sin_port = htons(port);

  // Convert IPv4 and IPv6 addresses from text to binary form
  if (inet_pton(AF_INET, ip.c_str(), &(address->sin_addr)) <= 0)
  {
    printf("\nInvalid address/ Address not supported \n");
    return -1;
  }

  if (connect(sfd, (struct sockaddr *)address, sizeof(struct sockaddr_in)) < 0)
  {
    printf("\nConnection Failed \n");
    return -1;
  }
  try
  {
    if (server_fd.size() <= channel)
    {
      server_fd.push_back(sfd);
      addresses.push_back(address);
    }
    else
    {
      // TODO: Free up the existing sockets
      server_fd[channel] = sfd;
      addresses[channel] = address;
    }
  }
  catch (...)
  {
    perror("Error while Saving the Socket Memory Structures");
    exit(EXIT_FAILURE);
  }
  cout << "Socket for port " << port << " Connected Successfully!" << endl;
  return 0;
}

void DirectController::arm()
{
  // TODO: Instead of manipulating via RC, make and use APIs directly to FC
  cmd(0, 255, 127, 127);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  cmd(0, 127, 127, 127);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  cout << "ARMed Successfully...\n";
}

void DirectController::disarm()
{
  // TODO: Instead of manipulating via RC, make and use APIs directly to FC
  cmd(0, 0, 127, 127);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  cmd(0, 127, 127, 127);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  cout << "Disarmed Successfully...\n";
}

void DirectController::balance()
{
  // TODO: Instead of manipulating via RC, make and use APIs directly to FC
  cmd(-1, -1, 255, 255);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  cmd(-1, -1, 127, 127);
}

void DirectController::altitudeHold()
{
  //cmd(-1, -1, 127, 127);
}

/*      APIs for channel Controls       */
void DirectController::cmd(int throttle, int yaw, int roll, int pitch, int aux1, int aux2, int aux3, int aux4)
{
  setThrottle(throttle);
  setYaw(yaw);
  setRoll(roll);
  setPitch(pitch);
  setAux(1, aux1);
  setAux(2, aux2);
  setAux(3, aux3);
  setAux(4, aux4);
}

void DirectController::sendCommand(int val, int channel)
{
  try
  {
    if (val == -1) // If the value recieved is nonsence, send over the last sensible data
    {
      val = channelBuffs[channel];
    }
    else if (val >= 255)
    {
      val = 255;
      channelBuffs[channel] = 255;
    }
    else if (channelBuffs[channel] == val) // Why send the same data again? waste of time
    {
      return;
    }
    else
    {
      channelBuffs[channel] = val;
    }
    char *bmsg;
    int blen = 0;
#if defined(STREAM_PROTOCOL_1)
    stringstream ss; // = new stringstream;
    ss << ".[:" << val << ":]";
    string msg = ss.str(); // = new string(ss->str());
    bmsg = (char *)msg.c_str();
    blen = msg.size();
#elif defined(STREAM_PROTOCOL_2)
    uint8_t gm[1] = {uint8_t(val)};
    bmsg = (char*)gm;
    blen = 1;
#endif
    send(server_fd[channel], bmsg, blen, 0);
    /*delete msg;
    delete ss;*/
  }
  catch (exception &e)
  {
    printf("\n{ERROR: %s", e.what());
  }
}

void DirectController::printChannels()
{
  printf("\nData: ");
  for(int i = 0; i < 9; i++)
    printf("[%d]--", channelBuffs[i]);
  fflush(stdout);
}

void DirectController::setThrottle(int val)
{
  sendCommand(val, 0);
}

void DirectController::setPitch(int val)
{
  sendCommand(val, 1);
}

void DirectController::setYaw(int val)
{
  sendCommand(val, 3);
}

void DirectController::setRoll(int val)
{
  sendCommand(val, 2);
}

void DirectController::setAux(int channel, int val)
{
  sendCommand(val, channel + 3);
}

void DirectController::callRAPI(int code, int val)
{
    char *bmsg;
    int blen = 0;
    uint8_t gm[1] = {uint8_t(code)};
    bmsg = (char*)gm;
    blen = 1;
    send(server_fd[8], bmsg, blen, 0);
}
/*
  Sensors APIs 
*/

int DirectController::startSensorsServer()
{
  return 0;
}

DroneState_t *DirectController::getState()
{
  return NULL;
}

DronePosition_t *DirectController::getPosition()
{
  return NULL;
}

int DirectController::startCameraServer()
{
  return 0;
}
int *DirectController::getCameraView(int id)
{
  return NULL;
}

int *DirectController::getCameraView(DroneCamera_t *camera)
{
  return NULL;
}