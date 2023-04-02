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
float proportionalRate = 10;
float maxRate = 120;

const int maxSpeed = 255; //max PWM value written to motor speed pin. It is typically 255.
const int minSpeed = 80; //min PWM value at which motor moves
float angle; //due to how I orientated my MPU6050 on my car, angle = roll
float targetAngle = 0;
int targetSpeed = 180;
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

const int buttonPin = 8;




void setup() {
  Serial.begin(115200);
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
  if (digitalRead(buttonPin)==HIGH){
    Serial.println("Button Pushed");
    delay(1000);
    targetAngle = 45;
    Serial.print("CurrentAngle =");Serial.println(angle);
    Serial.print("TargetAngle = ");Serial.println(targetAngle);
    timeStart = millis();
    timeNow = millis();
    rightSpeedVal = targetSpeed;
    leftSpeedVal = targetSpeed;
    while((timeNow-timeStart) < 5000){
      //Serial.print("TimeS, TimeN = ");Serial.print(timeStart); Serial.print("  "); Serial.println(timeNow);
      moveControl();
      timeNow = millis();
    }
    

    stopCar();
        
      
    
  }  

  


  
}


