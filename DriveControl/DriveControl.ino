#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int baudRate = 9600;
// const int baudRate = 115200;

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ; //linear acceleration
float GyroX, GyroY, GyroZ; //angular velocity
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; //used in void loop()
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
float proportionalRate = 0.75; //speed adjustment per degree of error
float maxRate = 120;
int maxDuration = 600000;  // run duration in ms

const int maxSpeed = 255; //max PWM value written to motor speed pin. It is typically 255.
const int minSpeed = 40; //min PWM value at which motor moves

float currentAngle; //if MPU6050 is flat, angle = Z = yaw
float targetAngle = 0;
float deltaAngle;
int targetSpeed = 0;
int speedCorrection;
float angleTolerance = .1;
int dataIsSpeed = 0;
int dataIsAngle = 0;


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
int readValue, actValue;
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
  Serial.println("<Arduino is ready>");
  analogWrite(10, 100);
  analogWrite(11, 100);
  delay(1);
  analogWrite(10, 0);
  analogWrite(11, 0);

}

void loop() {
  long int time0, timeStart, timeNow;
  // put your main code here, to run repeatedly:
  
    //targetAngle = 0;
    getDataFromPC();
    readValue = atoi(inputBuffer);
    // append 10000 for speed
    // append 20000 for angle
    if ((readValue < 12000) && newDataFromPC) {
      Serial.print(readValue);
      dataIsSpeed = 1;
      actValue = readValue - 10000;  
      Serial.print("Speed = ");
      Serial.println(actValue);    
    }
    else if ((readValue < 30000 )&& newDataFromPC){
      dataIsAngle = 1;
      actValue = readValue - 20000;  
      Serial.print("Target Angle = ");
      Serial.println(actValue);
      }
    if (dataIsSpeed == 1){
         targetSpeed = actValue;  
         dataIsSpeed = 0; 
    }
    if (dataIsAngle == 1){
        targetAngle = actValue;
        dataIsAngle = 0;   
        Serial.print("targetAngle = ");
        Serial.println(targetAngle);      
    }
    //duration = atoi(inputBuffer);
    replyToPC();
    
    //if(readSerial() > 0){
    if(newDataFromPC){
      newDataFromPC = false;
      forward();   // sets direction pins, not movement
      rightSpeedVal = targetSpeed;
      leftSpeedVal = targetSpeed;
      //Serial.println("Target Speed = ");
      //Serial.println(targetSpeed);
      timeStart = millis();
      timeNow = millis();
      //if((timeNow - timeStart) < maxDuration){
      //  stopCar;
      //}            
         
    }

  // button controlled start for testing
  if (digitalRead(buttonPin)==LOW){
    //Serial.println("Button Pushed");
    delay(1);
    maxDuration = 2000;
   // targetAngle = 0;
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

    //stopCar();
        
      
    
  }  

  


  
}
