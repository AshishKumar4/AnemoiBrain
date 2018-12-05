#include "stdio.h"
#include "iostream"
#include "vector"
#include "string"
#include <algorithm>
#include "stdio.h"
#include "string.h"
#include "cstring"

using namespace std;

#include "SerialRX.h"

SerialRX::SerialRX(char* portName)
{
    buffer = new char[81920];
    strcpy(pname, portName);
    //fread(buffer, 1000, 1, file);
}

FILE* SerialRX::openSerial()
{
    return serialFile = fopen(pname, "r");
}

void SerialRX::closeSerial()
{
    fclose(serialFile);
}

string SerialRX::getBuff(int size)
{
    //serialFile = fopen(pname, "r");
    fread(buffer, size, 1, serialFile);
    //fclose(serialFile);
    //strcpy(buffer, "1242 1438 1236 1485 1240 1240\n1242 1438 1236 1485 1240 1240\n1242 1438 1236 1485 1240 1240\n1242 1438 1236 1485 1240 1240\n1242 1438 1236 1485 1240 1240\n1242 1438 1236 1485 1240 1240\n");
    /*int i;
    for(i = 0; buffer[i] != '\n' && buffer[i] != '\0'; i++);
    buffer += i + 1;*/
    return string(buffer);
}

string SerialRX::getBuff_synced(int size)
{
    serialFile = fopen(pname, "r");
    fread(buffer, size, 1, serialFile);
    fclose(serialFile);
    //strcpy(buffer, "1242 1438 1236 1485 1240 1240\n1242 1438 1236 1485 1240 1240\n1242 1438 1236 1485 1240 1240\n1242 1438 1236 1485 1240 1240\n1242 1438 1236 1485 1240 1240\n1242 1438 1236 1485 1240 1240\n");
    /*int i;
    for(i = 0; buffer[i] != '\n' && buffer[i] != '\0'; i++);
    buffer += i + 1;*/
    return string(buffer);
}

string SerialRX::getLines_synced(int num = 3)
{
    serialFile = fopen(pname, "r");
    serialWriteFile = fopen(pname, "w");
    char* buff = new char[10];
    buff[0] = num;
    fread(buff, 1, 1, serialWriteFile);
    fclose(serialWriteFile);

    fread(buffer, 30*num, 1, serialFile);
    fclose(serialFile);

    return string(buffer);
}