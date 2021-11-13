#define RC_SERVO_PIN 8
#define NEURAL_ANGLE 98
#define LEFT_STEER_ANGLE  -40
#define RIGHT_STEER_ANGLE  40

#include <Servo.h>
Servo   Steeringservo;
int Steering_Angle = NEURAL_ANGLE;


void setup() {
  // put your setup code here, to run once:
  Steeringservo.attach(RC_SERVO_PIN);
  Steeringservo.write(Steering_Angle);
}

void loop() {
  // put your main code here, to run repeatedly:

}
