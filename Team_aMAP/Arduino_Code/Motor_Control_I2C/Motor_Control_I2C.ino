#include <Wire.h>

#define debug 1

int Steering_Angle = 90;  // 16bit
int Motor_Speed    = 0;   // 16bit
int sonar = 10;

void receiveEvent(int howMany)
{
  unsigned  char a[6];
  a[0] = Wire.read();
  a[1] = Wire.read();
  a[2] = Wire.read();
  a[3] = Wire.read();
  a[4] = Wire.read();
  a[5] = Wire.read();

  Steering_Angle =  a[1]*256 + a[2];
  Motor_Speed    =  a[4]*256 + a[5];
  
/*  
  Serial.print(a[0]);   Serial.print("  ");  Serial.print(a[1]);  Serial.print("  ");
  Serial.print(a[2]);   Serial.print("  ");  Serial.print(a[4]);   Serial.print("  ");
  Serial.print(a[5]);   Serial.print("  ");
  Serial.print(Steering_Angle);   Serial.print("  ");
  Serial.println(Motor_Speed);
*/
  
  
}

void requestEvent() 
{
  unsigned char s[2] = {0,};
  
  int temp;
  temp = sonar*10;
  
  s[0]= (temp&0xff00)>>8;
  s[1]= (temp&0x00ff); 
  Wire.write(s,2); // respond 
}

void setup()
{
  Wire.begin(5);    // I2C bus  #5
  Wire.onRequest(requestEvent); // register events
  Wire.onReceive(receiveEvent);
  Serial.begin(115200);
}

void loop()
{
 if(debug == 1)
  {   
    Serial.print("Steering Angle : ");
    Serial.print(Steering_Angle);
  
    Serial.print("  Motor PWM : ");
    Serial.println(Motor_Speed);
  }
}
