#ifndef DIRECT_CONTROLS_H
#define DIRECT_CONTROLS_H

#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>
#include <thread> // std::thread

#include "../Utils/common.hpp"

#define CHANNEL_COUNT   9

class DirectController
{
  vector<int> server_fd;
  vector<int> socket_num;
  vector<struct sockaddr_in *> addresses;
  int channelBuffs[CHANNEL_COUNT] =     {0, 0, 0, 0, 0, 0, 0, 0, 0};
  
protected:
  DronePosition_t*          currentPosition;
  DroneState_t*             currentState;
  vector<DroneCamera_t>     cameras;
  DroneIMU_t*               imu;

  void sendCommand(int val, int channel);
public:
  void InitSequence();
  int ConnectChannel(std::string ip, int port, int channel);
  DirectController(std::string ip = "0.0.0.0", int portBase = 8400);
  ~DirectController();
  void arm();
  void disarm();
  void balance();
  void altitudeHold();
  /*      APIs for channel Controls       */
  void cmd(int throttle, int yaw, int roll, int pitch, int aux1 = 0, int aux2 = 0, int aux3 = 0, int aux4 = 0);

  void setThrottle(int val);
  void setPitch(int val);
  void setYaw(int val);
  void setRoll(int val);
  void setAux(int channel, int val);
  void callRAPI(int code, int val);

  void printChannels();
  /* APIs to get Data */
  int startSensorsServer();
  DroneState_t* getState();
  DronePosition_t* getPosition();
  int startCameraServer();
  int* getCameraView(int id);
  int* getCameraView(DroneCamera_t* camera);
};

#endif