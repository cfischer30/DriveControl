/*   Drive Control 2
By Chris Fischer
Based on Drive Control -- same principal, but cleaned up
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
#include "Setup.h"
/*
float currentAngle; //if MPU6050 is flat, angle = Z = yaw
float targetAngle = 0;
float deltaAngle;
int targetSpeed = 0;
int speedCorrection;
int correctionAngle; */
float angleTolerance = .5;



void loop() {
  currentAngle = readCompassAzimuth();
  
  //  If new information is available from serial, read it.
  if (Serial.available() > 0){
    recvWithStartEndMarkers(); // reads from serial and populates the variable receivedChars[]
    readValue = atol(receivedChars);
    if(newData){
      Serial.print("Read Value = ");
      Serial.println(readValue);
      
    }
  }

 // parse information read to extract either new speed or new direction information
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
      newData = false;
      
      }
    else if ((readValue < 30000 )&& newData){
      dataIsAngle = 1;
      actValue = readValue - 20000;  
      Serial.print("Correction Angle = ");
      Serial.println(actValue);
      correctionAngle = actValue;
      
      //targetAngle = currentAngle + correctionAngle;
      targetAngle = findTargetAngle(currentAngle, correctionAngle);
      newData = false;
      
      }
    else if (newData){
      Serial.println("New Data not processed");
      newData = false;
    }
    

  // compare current speed and direction to target speed and direction
  // if they don't match, call moveContol
  lcdStatus(spdCol,spdRow,"TargetSpeed",float(targetSpeed));
  lcdStatus(actCol,actRow,"Current Angle",float(currentAngle));
  lcdStatus(corrCol,corrRow,"correction",float(correctionAngle));
  lcdStatus(tarCol,tarRow,"targetAngle",float(targetAngle));

  moveControl();


  }
