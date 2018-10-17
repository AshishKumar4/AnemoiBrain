#include <SPI.h>

int checksum(char* buf, int len)
{
  char tt = 0;
  for (int i = 0; i < len-1; i++)
  {
    tt ^= buf[i];
  }
  return tt;
}

struct ControlPackets
{
  char magic;
  char throttle;
  char pitch;
  char roll;
  char yaw;
  char aux1;
  char aux2;
  char switches;
  char random[9];
  char checksum;
};

ControlPackets cp;
char* buff = (char*)&cp;

volatile int index = 0;
volatile boolean process = false;

void setup(void)
{
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  // SPI.begin();
  index = 0;
  SPI.attachInterrupt();
  Serial.begin(115200);
}

ISR (SPI_STC_vect)
{
  if(!process)// && (SPSR & (1<<SPIF)) != 0)
  {
    if(index < sizeof(ControlPackets))
    {
      buff[index++] = SPDR;
    }
    else 
    {
      //if(buff[0] == 110)
      process = true;
      // index = 0;
    }
  }
}

void loop()
{

  /*memcpy(&cp.random, "hello\0", 9);
   char* tt = (char*)&cp;
   SPI_transfer(tt, sizeof(ControlPackets));*/
  if(process)
  {
    if(cp.magic == 110)// && cp.checksum == checksum(buff, sizeof(ControlPackets)))
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
    index = 0;
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





