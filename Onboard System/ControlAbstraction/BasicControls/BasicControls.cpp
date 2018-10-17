#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "SPIdrivers.c"
#include <fcntl.h>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <wiringPiSPI.h>
#include <string>

using namespace std;

int checksum(char* buf, int len)
{
    char tt  = 0;
    for(int i = 0; i < len - 1; i++)
    {
	tt ^= buf[i];
    }
    return tt;
}

struct ControlPackets
{
    unsigned char  magic;
    unsigned char throttle;
    unsigned char pitch;
    unsigned char roll;
    unsigned char yaw;
    unsigned char aux1;
    unsigned char aux2;
    unsigned char switches;
    unsigned char random[9];
    unsigned char  checksum;
};

int main()
{
    ControlPackets* pp = new ControlPackets;
    //int fd = wiringPiSPISetup(0, SPI_IOC_WR_MAX_SPEED_HZ);//SPI_init("/dev/spidev0.0");
    int fd = SPI_init("/dev/spidev0.0");
    while(1)
    {
	pp->magic = 110;
	pp->throttle = 255;
	pp->pitch = 230;
	pp->roll = 20;
	pp->yaw = 40;
	//*/
	/*int t;
	cout<<"Throttle: ";
	cin>>t;
	pp->throttle = (unsigned char)t;
	cout<<"Pitch: ";
	cin>>t;
	pp->pitch = (unsigned char)t;//*/
	//pp->checksum = checksum((char*)pp, sizeof(ControlPackets));
        SPI_ReadWrite(fd, (uintptr_t)pp, sizeof(ControlPackets));
        //wiringPiSPIDataRW(0, (unsigned char*)pp, sizeof(ControlPackets));
	//cout<<sizeof(ControlPackets);//pp->random<<endl;//sizeof(ControlPackets);
        //strcpy(msg, "Hello Arduino!!!");
	
        usleep(200);
    }
}
