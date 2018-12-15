#include <iostream>
#include <string>
#include "../SPI/SPIdrivers.h"
#include <unistd.h>

using namespace std;

int main()
{
	cout<<"SPI Testing Program...";
	int fd = SPI_init("/dev/spidev0.0");
	while(1)
	{
		uint8_t a;
		cout<<"\nEnter Val: ";
		int aa;
		cin>>aa;
		a = (uint8_t)aa;
		uint8_t b = a;
		SPI_ReadWrite((int)fd, (uintptr_t)&a, (uintptr_t)&a, (size_t)1);
		while(1)
		{
			usleep(1000);
			if(a != b)
				break;
			cout<<".";
		}
		printf("<%d>", a);
		cout<<"[a : "<<int(a)<<"\n";
	}
	return 0;
}

