int channel1 = A0;
int channel2 = A1;
int channel3 = A2;
int channel4 = A3;
int channel5 = A4;
int channel6 = A5;

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
    //if (Serial.available())
    {
        //int a = Serial.read();
        //for (int i = 0; i < a; i++)
        {
            pwm_value = analogRead(channel1); // Channel 1 is throttle
            Serial.print(pwm_value);
            Serial.print(" ");
            pwm_value = analogRead(channel2); // Channel 2 is Yaw
            Serial.print(pwm_value);
            Serial.print(" ");
            pwm_value = analogRead(channel3); // Channel 3 is Pitch
            Serial.print(1024-pwm_value);
            Serial.print(" ");
            pwm_value = analogRead(channel4); // Channel 4 is Roll
            Serial.print(1024-pwm_value);
            Serial.print(" ");
            pwm_value = analogRead(channel5);
            Serial.print(pwm_value);
            Serial.print(" ");
            pwm_value = analogRead(channel6);
            Serial.print(pwm_value);
            Serial.print("\n");
        }
    }
} /*/
 void setup()
 {
 Serial.begin(9600);
 pinMode(3, OUTPUT);
 digitalWrite(3, HIGH);
 pinMode(A0, INPUT);
 }
 
 void loop()
 {
 Serial.println(analogRead(A0));
 }//*/
