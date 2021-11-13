
/*
설 명 : I2C를 사용하여 데이타를 전송하는 예제이다.
*/
#include <string.h>  
#include <unistd.h>  
#include <errno.h>  
#include <stdio.h>  
#include <stdlib.h>  
#include <linux/i2c-dev.h> 
#include <sys/ioctl.h>  
#include <fcntl.h>  
#include <unistd.h>  


#include <sstream>

//i2c address  
#define ADDRESS 0x05

//I2C bus  
static const char *deviceName = "/dev/i2c-1";



#define RAD2DEG(x) ((x)*180./M_PI)

int steering_angle = 90;
int motor_speed = 128;

int steering_angle_old = 90;
int motor_speed_old = 128;

float sonar=0;

unsigned char protocol_data[6] = {'S',0,0,'D',0,0};

int file_I2C;

int open_I2C(void)
{
   int file;  
   
    if ((file = open( deviceName, O_RDWR ) ) < 0)   
    {  
        fprintf(stderr, "I2C: Failed to access %s\n", deviceName);  
        exit(1);  
    }  
    printf("I2C: Connected\n");  
  
   
    printf("I2C: acquiring buss to 0x%x\n", ADDRESS);  
    if (ioctl(file, I2C_SLAVE, ADDRESS) < 0)   
    {  
        fprintf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x\n", ADDRESS);  
        exit(1);  
    } 
    
    return file; 
}

/*
설명 : 시리얼포트를 닫는다.
*/

void close_I2C(int fd)
{
   close(fd);
}



void CarSteerControl(void)
{
    
  if(steering_angle >= 40+90)  steering_angle = 90+40;
  if(steering_angle <=-40+90)  steering_angle = 90-40;
  
}

void CarSpeedControl(void)
{
	
	(motor_speed  >= 255) ? 255 : motor_speed;
	(motor_speed  <= 255) ? 0   : motor_speed;
}


int main(void)
{
  char buf[2];
  float sonar_data;
  
  file_I2C = open_I2C();
  if(file_I2C < 0)
  {
	  printf("Unable to open I2C\n");
	  return -1;
  }
  else
  {
	  printf("I2C is Connected\n");
  }

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  motor_speed    = 0;
  steering_angle = 250;
  while(1)
  {
    
   
    protocol_data[1] = (steering_angle&0xff00)>>8 ;
    protocol_data[2] = (steering_angle&0x00ff);
    protocol_data[4] = (motor_speed&0xff00)>>8 ;
    protocol_data[5] = (motor_speed&0x00ff);
    
    if(steering_angle != steering_angle_old) 
    {
       write(file_I2C, protocol_data, 6);
       read(file_I2C,buf,2);
       sonar = (buf[0]*256+buf[1])/10.0;
    }
     if(motor_speed != motor_speed_old)
    {
       write(file_I2C, protocol_data, 6);
       read(file_I2C,buf,2);
       sonar = (buf[0]*256+buf[1])/10.0;
    } 
    steering_angle_old = steering_angle;
    motor_speed_old = motor_speed ; 
    
    sleep(1);
    printf("Steering Angle : %3d / Motro PWM %3d ",steering_angle, motor_speed);
    printf("/ Sonar : %lf\n", sonar);
    ++count;
    motor_speed--;
    steering_angle++;
  }

  close_I2C(file_I2C);
  return 0;
}




