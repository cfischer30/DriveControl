#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int baudRate = 9600;
// const int baudRate = 119200;

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ; //linear acceleration
float GyroX, GyroY, GyroZ; //angular velocity
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; //used in void loop()
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
float proportionalRate = 3; //speed adjustment per degree of error
float maxRate = 120;
int duration;  // run duration in ms

const int maxSpeed = 255; //max PWM value written to motor speed pin. It is typically 255.
const int minSpeed = 80; //min PWM value at which motor moves

float currentAngle; //if MPU6050 is flat, angle = Z = yaw
float targetAngle = 0;
float deltaAngle;
int targetSpeed = 180;
int speedCorrection;
float angleTolerance = .1;

// const int left1 = 4;  //for L298N control
const int left1 = 2; // MD20A control
const int left2 = 5;     // ignored in MD20A
const int right2 = 6;    // ignored in MD20A

// const int right1 = 7;   // for L298n control
const int right1 = 4;  // for MD20A
// for an H bridge with single pin direction control, use only pins left1 and right1
// const int leftSpeed = 10;  // L298N
const int leftSpeed = 3;      // MD20A
// const int rightSpeed = 11;
const int rightSpeed = 5;   // MD20A

int leftSpeedVal, rightSpeedVal;

// *** variables for new readSerial2.h
//Modifying variable block to accept 2 variabls from 

/* const byte buffSize = 40;
char inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false; */
const byte buffSize = 40;
char inputBuffer[buffSize];
char inputBuffer2[buffSize];
const char startMarker = '<';
const char endMarker = '>';
const char splitMarker = '|';
int isSplit;
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;

char messageFromPC[buffSize] = {0};

// **** end new variable block

#include "gyro.h"
#include "movement.h"
#include "readSerial.h"
#include "readSerial2.h"

const int buttonPin = 8;




void setup() {
  Serial.begin(baudRate);
  //Serial.println("started");
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  // Call this function if you need to get the IMU error values for your module
  calculateError();
  

  pinMode(buttonPin,INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("<Arduino is ready>");

}

void loop() {
  long int time0, timeStart, timeNow;
  // put your main code here, to run repeatedly:
  //if (digitalRead(buttonPin)==HIGH){
    //Serial.println("Button Pushed");
    //delay(1000);
    getDataFromPC();
    targetAngle = atoi(inputBuffer);
    duration = atoi(inputBuffer2);
    //replyToPC();
    //if(readSerial() > 0){

    /*  comment out drive control to test communication
    // uncomment this block to drive car

  
    if(newDataFromPC){
      newDataFromPC = false;
      forward();
      rightSpeedVal = targetSpeed;
      leftSpeedVal = targetSpeed;
      timeStart = millis();
      timeNow = millis();
      while((timeNow - timeStart) < duration){
        moveControl();
        timeNow = millis();
      }  */     

      // use blinking LED to test communication
      if(newDataFromPC){
        newDataFromPC = false;
        for (int i = 1; i <= targetAngle+duration; i++){
          digitalWrite(LED_BUILTIN, HIGH);
          delay(500);
          digitalWrite(LED_BUILTIN, LOW);
          delay(500);      
        }
          delay(3000);
          for (int i = 1; i <= duration; i++){
          digitalWrite(LED_BUILTIN, HIGH);
          delay(500);
          digitalWrite(LED_BUILTIN, LOW);
          delay(500);  
        }  
      stopCar();    

   /* Serial.print("CurrentAngle =");Serial.println(currentAngle);
    Serial.print("TargetAngle = ");Serial.println(targetAngle);
    timeStart = millis();
    timeNow = millis();
    rightSpeedVal = targetSpeed;
    leftSpeedVal = targetSpeed;
    while((timeNow-timeStart) < 10000){
      //Serial.print("TimeS, TimeN = ");Serial.print(timeStart); Serial.print("  "); Serial.println(timeNow);
      moveControl();
      timeNow = millis();
    }
    targetAngle -= 90;
    while((timeNow-timeStart) < 4000){
      moveControl();
      timeNow = millis();
      
    }*/

    stopCar();
        
      
    
  }  

  


  
}


