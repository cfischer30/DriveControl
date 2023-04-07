#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


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

const int left1 = 4;
const int left2 = 5;
const int right2 = 6;
const int right1 = 7;
const int leftSpeed = 10;
const int rightSpeed = 11;

int leftSpeedVal, rightSpeedVal;

#include "gyro.h"
#include "movement.h"
#include "readSerial.h"

const int buttonPin = 8;




void setup() {
  Serial.begin(115200);
  Serial.println("started");
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  // Call this function if you need to get the IMU error values for your module
  calculateError();

  pinMode(buttonPin,INPUT);

}

void loop() {
  long int time0, timeStart, timeNow;
  // put your main code here, to run repeatedly:
  //if (digitalRead(buttonPin)==HIGH){
    //Serial.println("Button Pushed");
    //delay(1000);
    targetAngle = 0;

    if(readSerial() > 0){
      forward();
      rightSpeedVal = targetSpeed;
      leftSpeedVal = targetSpeed;
      timeStart = millis();
      timeNow = millis();
      while((timeNow - timeStart) < duration){
        moveControl();
        timeNow = millis();
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


