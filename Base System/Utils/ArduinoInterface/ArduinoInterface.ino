#define channel1 A0 
#define channel2 A1 
#define channel3 A2 
#define channel4 A3
#define channel5 A4 
#define channel6 A5

int pwm_value;

void setup()
{
    pinMode(channel1, INPUT);
    pinMode(channel2, INPUT);
    pinMode(channel3, INPUT);
    pinMode(channel4, INPUT);
    pinMode(channel5, INPUT);
    pinMode(channel6, INPUT);
    Serial.begin(57600);
}

void loop()
{
    Serial.print(analogRead(channel1));       // Channel 1 is throttle
    Serial.print(" ");
    Serial.print(analogRead(channel2));     // Channel 2 is Yaw
    Serial.print(" ");
    Serial.print(analogRead(channel3));      // Channel 3 is Pitch
    Serial.print(" ");
    Serial.print(analogRead(channel4));      // Channel 4 is Roll
    Serial.print(" ");
    Serial.print(0);
    Serial.print(" ");
    Serial.print(0);
    Serial.print("\n");
} /*/
 void setup()
 {
 Serial.begin(9600);
 pinMode(13, OUTPUT);
 digitalWrite(3, HIGH);
 pinMode(A0, INPUT);
 }
 
 void loop()
 {
 Serial.println(analogRead(A0));
 }//*/
