#pragma once
#include "stdint.h"

#define CP_MAGIC 110

int checksum(char *buf, int len);

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
    unsigned char random[9];
    unsigned char checksum;
};

struct ResponsePackets
{
    unsigned char magic;
    unsigned char altitude;
    unsigned char pitch;
    unsigned char roll;
    unsigned char yaw;
    unsigned char aux1;
    unsigned char aux2;
    unsigned char switches;
    unsigned char random[9];
    unsigned char checksum;
};

void setThrottle(int throttle);
void setPitch(int pitch);
void setRoll(int roll);
void setYaw(int yaw);
ResponsePackets* getResponse();