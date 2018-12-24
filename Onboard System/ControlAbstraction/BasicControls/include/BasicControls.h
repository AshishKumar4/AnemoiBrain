#pragma once 

#include "stdint.h"

#define CP_MAGIC 110

uint8_t checksum(uint8_t *buf, int len);
#define CPACKET_MAGIC 110
#define REQ_SIGNAL     251
#define REQ2_SIGNAL     101
#define ACCEPT_SIGNAL    252
#define RPACKET_MAGIC  120
#define FALSE_PACKET   145
#define ACK_GOT_PACKET  250

void setThrottle(int throttle);
void setPitch(int pitch);
void setRoll(int roll);
void setYaw(int yaw);
void setAux1(int val);
void setAux2(int val);

void setThrottle(int throttle);
void setPitch(int pitch);
void setRoll(int roll);
void setYaw(int yaw);
void setAux1(int val);
void setAux2(int val);
int BasicControls_init(int argc, char *argv[]);
