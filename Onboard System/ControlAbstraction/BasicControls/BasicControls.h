#ifndef BASICCONTROLS_H
#define BASICCONTROLS_H
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

struct ControlPackets
{
    unsigned char magic;
    unsigned char throttle;
    unsigned char pitch;
    unsigned char roll;
    unsigned char yaw;
    unsigned char aux1;
    unsigned char aux2;
    unsigned char switches;
    //unsigned char random[9];
    unsigned char checksum;
};

struct ResponsePackets
{
    unsigned char magic;
    unsigned char alt;
    unsigned char pitch;
    unsigned char roll;
    unsigned char yaw;
    unsigned char lat;
    unsigned char lon;
    unsigned char heading;
    //unsigned char random[9];
    unsigned char checksum;
};

struct CommandPackets
{
    uint8_t magic;
    uint8_t value;
    uint8_t channel;
    uint8_t checksum;
};


void setThrottle(int throttle);
void setPitch(int pitch);
void setRoll(int roll);
void setYaw(int yaw);
void setAux1(int val);
void setAux2(int val);

int BasicControls_init();

ResponsePackets* getResponse();

#endif
