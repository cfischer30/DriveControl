
/*   Drive Control
By Chris Fischer
Read data from serial and use to control direction and speed.
Direction is compared to a MPU6050 6 axis accelerometer
Nominal speed is controlled by motor PWM controls

Speed is coded as 10xxx where xxx is speed (max 255)
Angle is coded as 20xxx where xxx is the target angle (0-359)

Ascii values should be sent with '<' and '>' as start and end 
characters, respectively.

So a speed of 155 should use the ascii string
<10155>
An angle of 97 should use the ascii string
<20097>

*/
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>  // I2C communication
#include <LiquidCrystal_I2C.h>  // librar for I2C 2 row x 16 column LCD
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 20 column and 4 line display


const int baudRate = 9600;
// const int baudRate = 115200;

// accelerometer variables
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ; //linear acceleration
float GyroX, GyroY, GyroZ; //angular velocity
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; //used in void loop()
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;

float elapsedTime, currentTime, previousTime;
int c = 0;
float proportionalRate = 1; //speed adjustment per degree of error
float maxRate = 120;
long int maxDuration = 600000;  // run duration in ms

const int maxSpeed = 255; //max PWM value written to motor speed pin. It is typically 255.
const int minSpeed = 80; //min PWM value at which motor moves

float currentAngle; //if MPU6050 is flat, angle = Z = yaw
float targetAngle = 0;
float deltaAngle;
int targetSpeed = 180;
int speedCorrection;
int correctionAngle;
float angleTolerance = .1;
int dataIsSpeed = 0;
int dataIsAngle = 0;
// lcd display variables
int spdRow, spdCol, tarRow,tarCol,actRow,actCol, corrRow, corrCol;


// for an H bridge with single pin direction control, use only pins left1 and right1
//const int left1 = 4;  //for L298N control
const int left1 = 4; // MD20A control
const int left2 = 5;     // ignored in MD20A
//const int right1 = 7;   // for L298n control
const int right1 = 7;  // for MD20A
const int right2 = 6;    // ignored in MD20A



// const int leftSpeed = 10;  // L298N
const int leftSpeed = 10;      // MD20A
// const int rightSpeed = 11;   //L298N
const int rightSpeed = 11;   // MD20A

int leftSpeedVal, rightSpeedVal;

// *** variables for new readSerial2.h
const byte buffSize = 40;
char inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;
long int readValue, actValue; // maximum value of an standard integer is 32767.   
char messageFromPC[buffSize] = {0};

// **** end new variable block

#include "gyro.h"
#include "movement.h"
//#include "readSerial.h"
//#include "readSerial2.h"


// delaring variables for readSerial3 library
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;
#include "readSerial3.h"   // read serial function from arduino website
#include "compassRead.h"   // read azimuth angle from compass chip



const int buttonPin = 8;






void setup() {
  Serial.begin(baudRate);
  //Serial.println("started");
  // setup I2C communication with accelerometer
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  // Call this function if you need to get the IMU error values for your module
  calculateError();
  // initialize compass - define I2S and error
  compassSetup();

  // setup LCD Display
  lcd.init();
  lcd.clear();         
  lcd.backlight();      // Make sure backlight is on

  pinMode(buttonPin,INPUT);   // start button on Aduino
  Serial.println("<Arduino is ready>");
  analogWrite(leftSpeed, 100);  
  analogWrite(rightSpeed, 100); 
  spdRow = 0;
  lcd.setCursor(0,spdRow);
  lcd.print("Target speed");
  spdCol = 14;
  lcd.setCursor(spdCol,spdRow);
  lcd.print(targetSpeed);
  actRow = 1;
  lcd.setCursor(0,actRow);
  lcd.print("Current Angle ");
  actCol = 14;
  lcd.setCursor(actCol,actRow);
  lcd.print(currentAngle);
  corrRow = 2;
  lcd.setCursor(0, corrRow);
  lcd.print("Correction");
  corrCol = 14;
  lcd.setCursor(corrCol, corrRow);
  lcd.print(correctionAngle);
 
  tarRow = 3;
  lcd.setCursor(0,tarRow);
  lcd.print("Target Angle ");
  tarCol = 14;
  lcd.setCursor(tarCol,tarRow);
  targetAngle = currentAngle + correctionAngle;
  lcd.print(targetAngle);
  delay(1000);

}

void loop() {
  long int time0, timeStart, timeNow;
  // put your main code here, to run repeatedly:
  currentAngle = float(readCompassAzimuth());
  lcd.setCursor(actCol,actRow);
  lcd.print("      ");
  lcd.setCursor(actCol,actRow);
  lcd.print(currentAngle);
  
    
   
    recvWithStartEndMarkers(); // reads from serial and populates the variable receivedChars[]
    readValue = atol(receivedChars);
    if(newData){
      Serial.print("Read Value = ");
      Serial.println(readValue);
      
    }
   
    
    // append 10000 for speed
    // append 20000 for angle
    if ((readValue < 10000) && newData ){
      Serial.print("readValue = ");
      Serial.println(readValue);
      Serial.println("use 10000 + value for speed, 20000 + value for angle");
      newData = false;
      }
    else if ((readValue < 12000) && newData) {
      Serial.print("readValue = ");
      Serial.println(readValue);
      actValue = readValue - 10000;  
      Serial.print("Speed = ");
      Serial.println(actValue);
      targetSpeed = actValue;
      lcd.setCursor(spdCol,spdRow);
      lcd.print("   ");
      lcd.setCursor(spdCol,spdRow);
      lcd.print(targetSpeed);
      }
    else if ((readValue < 30000 )&& newData){
      dataIsAngle = 1;
      actValue = readValue - 20000;  
      Serial.print("Correction Angle = ");
      Serial.println(actValue);
      correctionAngle = actValue;
      targetAngle = currentAngle + correctionAngle;
      lcd.setCursor(corrCol,corrRow);
      lcd.print("      ");
      lcd.setCursor(corrCol,corrRow);
      lcd.print(correctionAngle);
      lcd.setCursor(tarCol,tarRow);
      lcd.print("    ");
      lcd.setCursor(tarCol,tarRow);
      lcd.print(targetAngle);
      }
    else if (newData){
      Serial.println("New Data not processed");
      newData = false;
    }
    
    
    
    
    if(newData){
      newData = false;
      forward();   // sets direction pins, not movement
      rightSpeedVal = targetSpeed;
      leftSpeedVal = targetSpeed;
      
      moveControl();
       
         
    }
    
  // button controlled start for testing
  if (digitalRead(buttonPin)==LOW){
    //Serial.println("Button Pushed");
    delay(1);
    maxDuration = 2000;
  
    rightSpeedVal = targetSpeed;
    leftSpeedVal = targetSpeed;
    timeStart = millis();
    timeNow = millis();
    moveControl();
   /* while((timeNow - timeStart) < maxDuration){
      moveControl();
      timeNow = millis();
    }   */         
  //stopCar(); 
  
        
  
    
  }  

  


  
}
