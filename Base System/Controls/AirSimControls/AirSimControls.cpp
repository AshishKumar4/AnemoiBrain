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

#include "AirSimControls.h"

using namespace std;

const char HANDSHAKE_IN_MSG[] = "Hello Gardien!";
const char HANDSHAKE_OUT_MSG[] = "Hello Overloard!";
int opt = 1;

#define PORT_BASE 8300

#define PORT_THROTTLE PORT_BASE + 0
#define PORT_PITCH PORT_BASE + 1
#define PORT_ROLL PORT_BASE + 2
#define PORT_YAW PORT_BASE + 3
#define PORT_AUX1 PORT_BASE + 4
#define PORT_AUX2 PORT_BASE + 5

void AirSimController::InitSequence()
{
  for(int i = 0; i < 6; i++)
  {
    send(server_fd[i], HANDSHAKE_IN_MSG, strlen(HANDSHAKE_IN_MSG), 0);
    // TODO: Place mechanism to recieve back handshake and if not matching, Panic!
  }
  disarm();
  balance();
  disarm();
  cout << "Initialization Sequence Completed...\n";
}

AirSimController::AirSimController(char *ip)
{
  ConnectChannel(ip, PORT_THROTTLE, 0);
  ConnectChannel(ip, PORT_PITCH, 1);
  ConnectChannel(ip, PORT_ROLL, 2);
  ConnectChannel(ip, PORT_YAW, 3);
  ConnectChannel(ip, PORT_AUX1, 4);
  ConnectChannel(ip, PORT_AUX2, 5);

  InitSequence();
}

AirSimController::~AirSimController()
{
}

int AirSimController::ConnectChannel(char *ip, int port, int channel) // This would create a port for a particular channel
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
  if (inet_pton(AF_INET, ip, &(address->sin_addr)) <= 0)
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

void AirSimController::arm()
{
  cmd(0, 255, 127, 127);
  sleep(1);
  cmd(0, 127, 127, 127);
  cout << "ARMed Successfully...\n";
}

void AirSimController::disarm()
{
  cmd(0, 0, 127, 127);
  sleep(1);
  cmd(0, 0, 127, 127);
  cout << "Disarmed Successfully...\n";
}

void AirSimController::balance()
{
  cmd(-1, -1, 255, 255);
  sleep(0.01);
  cmd(-1, -1, 127, 127);
}

void AirSimController::altitudeHold()
{
  //cmd(-1, -1, 127, 127);
}

/*      APIs for channel Controls       */
void AirSimController::cmd(int throttle, int yaw, int roll, int pitch, int aux1, int aux2)
{
  setThrottle(throttle);
  setYaw(yaw);
  setRoll(roll);
  setPitch(pitch);
  setAux1(aux1);
  setAux2(aux2);
}

void AirSimController::setThrottle(int val)
{
  if (val == -1)
  {
    val = channelBuffs[0];
  }
  else
  {
    channelBuffs[0] = val;
  }
  stringstream ss;
  ss << ".[:" << val << ":]";
  string msg = ss.str();
  send(server_fd[0], msg.c_str(), msg.size(), 0);
}

void AirSimController::setPitch(int val)
{
  if (val == -1)
  {
    val = channelBuffs[1];
  }
  else
  {
    channelBuffs[1] = val;
  }
  stringstream ss;
  ss << ".[:" << val << ":]";
  string msg = ss.str();
  send(server_fd[1], msg.c_str(), msg.size(), 0);
}

void AirSimController::setYaw(int val)
{
  if (val == -1)
  {
    val = channelBuffs[2];
  }
  else
  {
    channelBuffs[2] = val;
  }
  stringstream ss;
  ss << ".[:" << val << ":]";
  string msg = ss.str();
  send(server_fd[2], msg.c_str(), msg.size(), 0);
}

void AirSimController::setRoll(int val)
{
  if (val == -1)
  {
    val = channelBuffs[3];
  }
  else
  {
    channelBuffs[3] = val;
  }
  stringstream ss;
  ss << ".[:" << val << ":]";
  string msg = ss.str();
  send(server_fd[3], msg.c_str(), msg.size(), 0);
}

void AirSimController::setAux1(int val)
{
  if (val == -1)
  {
    val = channelBuffs[4];
  }
  else
  {
    channelBuffs[4] = val;
  }
  stringstream ss;
  ss << ".[:" << val << ":]";
  string msg = ss.str();
  send(server_fd[4], msg.c_str(), msg.size(), 0);
}

void AirSimController::setAux2(int val)
{
  if (val == -1)
  {
    val = channelBuffs[5];
  }
  else
  {
    channelBuffs[5] = val;
  }
  stringstream ss;
  ss << ".[:" << val << ":]";
  string msg = ss.str();
  send(server_fd[5], msg.c_str(), msg.size(), 0);
}