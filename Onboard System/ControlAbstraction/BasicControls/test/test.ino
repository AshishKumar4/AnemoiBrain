#include <SPI.h>
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
    unsigned char alt;
    unsigned char pitch;
    unsigned char roll;
    unsigned char yaw;
    unsigned char lat;
    unsigned char lon;
    unsigned char heading;
    unsigned char random[9];
    unsigned char checksum;
};

ControlPackets rfCusData;
ResponsePackets rfResData;
char *buff = (char *)&rfCusData;
char *rbuff = (char *)&rfResData;

volatile int index = 0;
volatile boolean process = false;

void setup(void)
{
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  // SPI.begin();
  index = 0;
  SPI.attachInterrupt();
  Serial.begin(19200);
}

ISR(SPI_STC_vect)
{
  if (!process) // && (SPSR & (1<<SPIF)) != 0)
  {
    Serial.println((SPSR & (1<<SPIF)));
    if (index < sizeof(ControlPackets))
    {
      buff[index++] = SPDR;
    }
    else
    {
      //if(buff[0] == 110)
      process = true;
      
          /*
      rfResData.lat = 35.62;
      rfResData.lon = 139.68;
      rfResData.heading = 0;//att.heading;
      rfResData.pitch = 0;//att.angle[PITCH];
      rfResData.roll = 0;//att.angle[ROLL];
      rfResData.alt = 0;//alt.EstAlt;
          
      for (int i = 0; i < index; i++)
      {
        SPDR = rbuff[i];
      }*/
      
     /* if (rfCusData.magic == 110)
      {
        /*nrf24_rcData[THROTTLE] = map(rfCusData.throttle, 0, 255, 1000, 2000);
        nrf24_rcData[YAW] = map(rfCusData.yaw, 0, 255, 1000, 2000);
        nrf24_rcData[PITCH] = map(rfCusData.pitch, 0, 255, 1000, 2000);
        nrf24_rcData[ROLL] = map(rfCusData.roll, 0, 255, 1000, 2000);
        nrf24_rcData[AUX1] = map(rfCusData.aux1, 0, 255, 1000, 2000);
        nrf24_rcData[AUX2] = map(rfCusData.aux2, 0, 255, 1000, 2000);
        Serial.print((int)rfCusData.magic);
        Serial.print(" ");
        Serial.print((int)rfCusData.throttle);
        Serial.print(" ");
        Serial.print((int)rfCusData.pitch);
        Serial.print(" ");
        Serial.print((int)rfCusData.roll);
        Serial.print(" ");
        Serial.print((int)rfCusData.yaw);
        Serial.print("\n");
      }*/
      process = false;
      index = 0;
      // index = 0;
    }
  }
}

void loop()
{
  analogRead(A2);
  /*memcpy(&cp.random, "hello\0", 9);
   char* tt = (char*)&cp;
   SPI_transfer(tt, sizeof(ControlPackets));*/
 /* if(process)
  {
    index = 0;
    //if(cp.magic == 110)// && cp.checksum == checksum(buff, sizeof(ControlPackets)))
    {
      //Serial.print((int)sizeof(ControlPackets));
      //Serial.print("->");
      Serial.print((int)cp.magic);
      Serial.print(" ");
      Serial.print((int)cp.throttle);
      Serial.print(" ");
      Serial.print((int)cp.pitch);
      Serial.print(" ");
      Serial.print((int)cp.roll);
      Serial.print(" ");
      Serial.print((int)cp.yaw);
      Serial.print("\n");
    }
    process = false;
  }//*/
  /*char val=0, tmp;
  while(1)
  {
    tmp = SPI.transfer(val);
    if(tmp == 110)
      break;
  }
  SPI.transfer((void*)buff, (size_t)sizeof(ControlPackets));
//  if(cp.magic == 110)// && cp.checksum == checksum(buff, sizeof(ControlPackets)))
    {
      //Serial.print((int)sizeof(ControlPackets));
      //Serial.print("->");
      //Serial.print((int)cp.magic);
      //Serial.print(" ");
      Serial.print((int)cp.throttle);
      Serial.print(" ");
      Serial.print((int)cp.pitch);
      Serial.print(" ");
      Serial.print((int)cp.roll);
      Serial.print(" ");
      Serial.print((int)cp.yaw);
      Serial.print("\n");
    }*/
}

int SPI_transfer(char* buff, int len)
{
  for(int i = 0; i < len; i++)
  {
    buff[i] = SPDR;
  }
}





