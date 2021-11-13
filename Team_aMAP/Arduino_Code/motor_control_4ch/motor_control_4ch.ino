#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

////////////////// Mini OLED Driver ////////////////////////
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
/////////////////// Encoder Driver ////////////////////////
#define ENC1_ADD 41
#define ENC2_ADD 42

signed long encoder1count = 0;
signed long encoder2count = 0;


void initEncoders() {
  
  // Set slave selects as outputs
  pinMode(ENC1_ADD, OUTPUT);
  pinMode(ENC2_ADD, OUTPUT);

  
  // Raise select pins
  // Communication begins when you drop the individual select signsl
  digitalWrite(ENC1_ADD,HIGH);
  digitalWrite(ENC2_ADD,HIGH);

  
  SPI.begin();
  
  // Initialize encoder 1
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(ENC1_ADD,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(ENC1_ADD,HIGH);       // Terminate SPI conversation 

  // Initialize encoder 2
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(ENC2_ADD,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(ENC2_ADD,HIGH);       // Terminate SPI conversation 
}

long readEncoder(int encoder_no) 
{
  
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;  
  
  
    digitalWrite(ENC1_ADD + encoder_no-1,LOW);      // Begin SPI conversation
   // digitalWrite(ENC4_ADD,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                     // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);           
    count_3 = SPI.transfer(0x00);           
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
    //digitalWrite(ENC4_ADD,HIGH);      // Begin SPI conversation
// Calculate encoder count
  count_value= ((long)count_1<<24) + ((long)count_2<<16) + ((long)count_3<<8 ) + (long)count_4;
  
  return count_value;
}


void clearEncoderCount(int encoder_no) {
    
  // Set encoder1's data register to 0
  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
}
/////////////////// Motor Driver ////////////////////////
#define STBY1  39
#define STBY2  40

#define MOTOR1_EN1 32
#define MOTOR1_EN2 31
#define MOTOR1_PWM 8

#define MOTOR2_EN1 33
#define MOTOR2_EN2 34
#define MOTOR2_PWM 7

void motor1_control(int dir, int speed)
{
  digitalWrite(STBY1, HIGH);
   switch(dir)
   {
      case  1: digitalWrite(MOTOR1_EN1, HIGH);
               digitalWrite(MOTOR1_EN2, LOW);
               analogWrite(MOTOR1_PWM, speed);
               break; 
      
      case -1:    
               digitalWrite(MOTOR1_EN1, LOW);
               digitalWrite(MOTOR1_EN2, HIGH);
               analogWrite(MOTOR1_PWM, speed);      
               break;

      case 0 :
               digitalWrite(MOTOR1_EN1, LOW);
               digitalWrite(MOTOR1_EN2, LOW);
               analogWrite(MOTOR1_PWM, 0);
               break;
      default :                  
               digitalWrite(MOTOR1_EN1, LOW);
               digitalWrite(MOTOR1_EN2, LOW);
               analogWrite(MOTOR1_PWM, 0);
               break;     
   }
}
void motor2_control(int dir, int speed)
{
  digitalWrite(STBY1, HIGH);
   switch(dir)
   {
      case  1: digitalWrite(MOTOR2_EN1, HIGH);
               digitalWrite(MOTOR2_EN2, LOW);
               analogWrite(MOTOR2_PWM, speed);
               break; 
      
      case -1:    
               digitalWrite(MOTOR2_EN1, LOW);
               digitalWrite(MOTOR2_EN2, HIGH);
               analogWrite(MOTOR2_PWM, speed);      
               break;

      case 0 :
               digitalWrite(MOTOR2_EN1, LOW);
               digitalWrite(MOTOR2_EN2, LOW);
               analogWrite(MOTOR2_PWM, 0);
               break;
      default :                  
               digitalWrite(MOTOR2_EN1, LOW);
               digitalWrite(MOTOR2_EN2, LOW);
               analogWrite(MOTOR2_PWM, 0);
               break;     
   }
}

void setup() 
{
  // put your setup code here, to run once:

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  //display.display();
  //delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
  
  display.drawPixel(10, 10, SSD1306_WHITE);
  display.display();
  delay(2000); // Pause for 2 seconds

  Serial.begin(115200);      // Serial com for data output
  //Wire.begin(5);    // I2C bus  #5
  //Wire.onRequest(requestEvent); // register events
  //Wire.onReceive(receiveEvent);
  initEncoders();       Serial.println("Encoders Initialized...");  
  clearEncoderCount(1);  Serial.println("Encoder[1] Cleared...");
  clearEncoderCount(2);  Serial.println("Encoder[2] Cleared...");
 
  pinMode(STBY1, OUTPUT); pinMode(STBY2, OUTPUT);
  pinMode(MOTOR1_EN1, OUTPUT);   pinMode(MOTOR1_EN2, OUTPUT);   pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR2_EN1, OUTPUT);   pinMode(MOTOR2_EN2, OUTPUT);   pinMode(MOTOR2_PWM, OUTPUT);

  delay(200);
  //motor1_control(-1, 30);
  //motor2_control(-1, 30);
  //motor3_control(-1, 30);
  //motor4_control(-1, 30);
}

void loop() {
  // put your main code here, to run repeatedly:

   encoder1count = readEncoder(1);  encoder2count = readEncoder(2); 

   Serial.print("Enc1: "); Serial.print(encoder1count); Serial.print(" "); 
   Serial.print("Enc2: "); Serial.println(encoder2count);


  //forword
  motor1_contorl(1,100);
  delay(1000);
   
  //stop
   motor1_contorl(0,100);
   delay(1000);

   //backward
   motor1_contorl(-1,100);
   delay(1000);
  /* motor1_control(-1, 100);
   delay(1000);
   motor1_control(0, 100);
   delay(1000);
   motor1_control(1, 250);
   delay(1000);
    motor1_control(0, 100);
   delay(1000);
   */
}
