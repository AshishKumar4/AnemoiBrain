#pragma once

#ifndef SERIALPORT_H
#define SERIALPORT_H

#define ARDUINO_WAIT_TIME 2000
#define MAX_DATA_LENGTH 255

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

using namespace std;

class SerialRX
{
    FILE* serialFile;
    FILE* serialWriteFile;
    char* buffer;
    char  pname[100];
  private:
  public:
    SerialRX(char *portName);
    ~SerialRX();

    FILE* openSerial();
    void closeSerial();
    string getBuff(int size);
    string getBuff_synced(int size);
    string getLines_synced(int num);
};

#endif // SERIALPORT_H