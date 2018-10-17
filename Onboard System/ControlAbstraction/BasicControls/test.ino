/*************************************************************
 SPI_Hello_Raspi
   Configures an ATMEGA as an SPI slave and demonstrates
   bidirectional communication with an Raspberry Pi SPI master
   by repeatedly sending the text "Hello Raspi"
****************************************************************/

/***************************************************************
 Global Variables
  -hello[] is an array to hold the data to be transmitted
  -marker is used as a pointer in traversing data arrays
/***************************************************************/

struct ControlPackets
{
    char throttle;
    char pitch;
    char roll;
    char yaw;
    char aux1;
    char aux2;
    char switches;
    char random[9];
};

int marker = 0;

/***************************************************************  
 Setup SPI in slave mode (1) define MISO pin as output (2) set
 enable bit of the SPI configuration register 
****************************************************************/

void setup(void)
{

    pinMode(MISO, OUTPUT);
    SPCR |= _BV(SPE);
    Serial.begin(9600);
}

/***************************************************************  
 Loop until the SPI End of Transmission Flag (SPIF) is set
 indicating a byte has been received.  When a byte is
 received, load the next byte in the Hello[] array into SPDR
 to be transmitted to the Raspberry Pi, and increment the marker.
 If the end of the Hell0[] array has been reached, reset
 marker to 0.
****************************************************************/

void loop(void)
{
    if ((SPSR & (1 << SPIF)) != 0)
    {
        SPDR = hello[marker];
        marker++;

        if (marker > sizeof(hello))
        {
            marker = 0;
        }
    }
}

char* SPI_ReadWrite(char *buf, int len)
{
    int i = 0;
    char* tmp = new char[len];
    while (((SPSR & (1 << SPIF)) != 0)) // If SPI End of Transmission not reached
    {
        tmp[i] = SPDR;
        SPDR = buf[i];
        ++i;

        if (i > len)
        {
            SPDR = 0;
        }
    }
    memcpy(buf, tmp, len);
    delete tmp;
    return buf;
}

int SPI_Read(char *buf, int len)
{
    int i = 0;
    while (((SPSR & (1 << SPIF)) != 0)) // If SPI End of Transmission not reached
    {
        buf[i] = SPDR;
        ++i;

        /*if (i > len)
        {
            buf[i] = 0;
        }*/
    }
}
int SPI_Write(char *buf, int len)
{
    int i = 0;
    while (((SPSR & (1 << SPIF)) != 0)) // If SPI End of Transmission not reached
    {
        SPDR = buf[i];
        ++i;

        if (i > len)
        {
            SPDR = 0;
        }
    }
}