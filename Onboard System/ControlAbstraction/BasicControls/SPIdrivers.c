#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <unistd.h>
#include "stdint.h"
#include "string.h"
#include "stdio.h"

/*
    SPI Drivers, to send and recieve data packets over SPI
*/

int SPI_init(char* file)
{
    int fd = (int)open(file, O_RDWR);
    unsigned int speed = 1000000;
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    return fd;
}

int SPI_ReadWrite(int fd, uintptr_t buffer, size_t len)
{
    uintptr_t tmp = (uintptr_t)malloc(len);
    struct spi_ioc_transfer spi;
    memset(&spi, 0, sizeof(spi));
    spi.tx_buf = (unsigned long)buffer;
    spi.rx_buf = (unsigned long)tmp;
    spi.len    = len;
    ioctl(fd, SPI_IOC_MESSAGE(1), &spi);
    memcpy((void*)buffer, (void*)tmp, len);
    return buffer;
}

int SPI_send(int fd, uintptr_t buffer, size_t len)
{
    struct spi_ioc_transfer spi;
    memset(&spi, 0, sizeof(spi));
    spi.tx_buf = (unsigned long)buffer;
    spi.rx_buf = (unsigned long)buffer;
    spi.len    = len;
    
    ioctl(fd, SPI_IOC_MESSAGE(1), &spi);

    return 1;
}

int SPI_read(int fd, uintptr_t buffer, size_t len)
{
    struct spi_ioc_transfer spi;
    memset(&spi, 0, sizeof(spi));
    spi.tx_buf = (unsigned long)buffer;
    spi.rx_buf = (unsigned long)buffer;
    spi.len    = len;
    
    ioctl(fd, SPI_IOC_MESSAGE(1), &spi);

    return 1;
}