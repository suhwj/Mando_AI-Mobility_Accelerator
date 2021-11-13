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

int file_I2C;

/*
설명 : I2C 포트를 연다.
*/
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
설명 : I2C 포트를 닫는다.
*/

void close_I2C(int fd)
{
   close(fd);
}

int main(void)
{
	
  file_I2C = open_I2C();
  if(file_I2C < 0)
  {
	  printf("Unable to open I2C");
	  return -1;
  }
  
  return 1;
}
