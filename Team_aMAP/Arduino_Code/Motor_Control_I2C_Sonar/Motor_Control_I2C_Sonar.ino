#define debug 0
///////////////////// Steering Servo Control /////////////////////
#define RC_SERVO_PIN 8
#define NEURAL_ANGLE 90
#define LEFT_STEER_ANGLE  -40
#define RIGHT_STEER_ANGLE  40

#include <Servo.h>
Servo   Steeringservo;
int Steering_Angle = NEURAL_ANGLE;

void steering_control()
{
  if(Steering_Angle<= LEFT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle  = LEFT_STEER_ANGLE + NEURAL_ANGLE;
  if(Steering_Angle>= RIGHT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle = RIGHT_STEER_ANGLE + NEURAL_ANGLE;
  Steeringservo.write(Steering_Angle);  
}

///////////////////// Steering Servo Control /////////////////////

/////////////////////// DC Motor Control /////////////////////

#define MOTOR_PWM 5 
#define MOTOR_DIR 4
int Motor_Speed = 0;

void motor_control(int dir, int motor_pwm)
{
  
  if(dir == 1) // forward
  { 
    digitalWrite(MOTOR_DIR, LOW);
    analogWrite(MOTOR_PWM,motor_pwm);
    
  }
  else if(dir == -1) // backward
  {
    digitalWrite(MOTOR_DIR, HIGH);
    analogWrite(MOTOR_PWM,motor_pwm);
  }
  else // stop
  {
    analogWrite(MOTOR_PWM,0);
  }
  
}

/////////////////////// DC Motor Control /////////////////////

////////////////////// Sonar Sensor Setup ////////////////////
#define TRIG 9
#define ECHO 10
int sonar = 10;

float sonar_sensor_read(void)
{
  float duration =0.0;
  float distance = 0.0;
  digitalWrite(TRIG,LOW);
  digitalWrite(TRIG,HIGH);
  delay(10);
  digitalWrite(TRIG,LOW);

  duration = pulseIn(ECHO,HIGH);
  //Serial.println(duration);
  distance = duration * 340.0/(2.0*10000);
  //Serial.print(distance);  Serial.println(" cm");

  return distance;
}

////////////////////// Sonar Sensor Setup ////////////////////

/////////////////////////// I2C 통신 //////////////////////////

#include <Wire.h>

#define debug 1
int sensor_flag = 0;
void receiveEvent(int howMany)
{
  unsigned  char a[7];
  a[0] = Wire.read();
  a[1] = Wire.read();
  a[2] = Wire.read();
  a[3] = Wire.read();
  a[4] = Wire.read();
  a[5] = Wire.read();
  a[6] = Wire.read();

  if( (a[0]=='#') && (a[1] == 'C') && (a[6] == '*'))
  { 
    Steering_Angle =  a[2]*256 + a[3];
    Motor_Speed    =  a[4]*256 + a[5];

    steering_control();
   
    if(Motor_Speed> 0)      motor_control(1,Motor_Speed);
    else if(Motor_Speed< 0) motor_control(-1,-Motor_Speed);
    else motor_control(0,0);
  }
  if( (a[0]=='#') && (a[1] == 'S') && (a[6] == '*'))
  { 
    sensor_flag = 1;
  }
  
  /*
  Serial.print(a[0]);   Serial.print("  ");  Serial.print(a[1]);  Serial.print("  ");
  Serial.print(a[2]);   Serial.print("  ");  Serial.print(a[4]);   Serial.print("  ");
  Serial.print(a[5]);   Serial.print("  ");
  Serial.print(Steering_Angle);   Serial.print("  ");
  Serial.println(Motor_Speed);
*/
  
   
}
/////////////////////////// I2C 통신 //////////////////////////

void requestEvent() 
{
  unsigned char s[8] = {0,};
  
  int temp;
  temp = sonar*10;
  s[0]='$';
  s[1]= (temp&0xff00)>>8;
  s[2]= (temp&0x00ff); 
  s[3]= 0;
  s[4]= 0;
  s[5]= 0;
  s[6]= 0;
  s[7]='#';
  
  Wire.write(s,8); // respond 
  sensor_flag = 0;
}

void setup()
{
  Wire.begin(5);    // I2C bus  #5
  Wire.onRequest(requestEvent); // register events
  Wire.onReceive(receiveEvent);
  Steeringservo.attach(RC_SERVO_PIN);
  Steeringservo.write(Steering_Angle);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  Serial.begin(115200);
  delay(2000);
}

void loop()
{
 if(debug == 1)
  {   

    Serial.print("Sonar : ");
    Serial.print(sonar);

    Serial.print("  Steering Angle : ");
    Serial.print(Steering_Angle);
  
    Serial.print("  Motor PWM : ");
    Serial.println(Motor_Speed);
  }
  
   sonar= sonar_sensor_read()*10;
   
 
}
