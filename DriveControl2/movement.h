/*   Available functions
moveControl() is the primary function
driving() - manages wheel speed while driving forward
controlSpeed() - adjusts correction speed differential proportional to delta angle.  Called by driving()
rotate() - rotates robot when not moving
changeSpeed() - manages minimum and maximum PWM settings



*/
void forward();
void left();
void right();
void driving();
void controlSpeed();
void rotate();
int changeSpeed (int motorSpeed, int increment);
void forward();
void left();
void right();
void stopCar();
void moveControl();
int findTargetAngle(int currentAngle,int correction);
int findCorrectionAngle(int currentAngle, int targetAngle);

void moveControl(){
 // readAcceleration();
  previousTime = currentTime;
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000;
  
  //uncomment to use accelerometer integrated Azimuth / yaw angle
  /*readGyro();
  GyroX -= GyroErrorX;
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;
  yaw += GyroZ * elapsedTime;
  currentAngle = yaw;*/
  //uncomment to use compass Azimuth / yaw angle
  //currentAngle = float(readCompass());
  
  //Serial.print("Current Angle = ");Serial.println(currentAngle);
  if(targetSpeed == 0){
    rotate();    
    }
  else{
     driving();
    }
  }
  
  
  // rewritten driving() function by Chris Fischer
  
  void driving(){   // called my moveControl
  forward();   // sets forward & reverse pins appropriately on H bridge
	//correctionAngle = targetAngle - currentAngle;
  correctionAngle = findCorrectionAngle(currentAngle, targetAngle);
	speedCorrection = int(correctionAngle * proportionalRate);
	
	rightSpeedVal = targetSpeed + speedCorrection;
	if(rightSpeedVal > maxSpeed)
		{rightSpeedVal = maxSpeed;}
	else if(rightSpeedVal < minSpeed)
		{rightSpeedVal = minSpeed;}
		
	leftSpeedVal = targetSpeed - speedCorrection;
	if(leftSpeedVal > maxSpeed)
		{leftSpeedVal = maxSpeed;}
	else if (leftSpeedVal < minSpeed)
		{leftSpeedVal = minSpeed;}
		
	analogWrite(10, rightSpeedVal);

  analogWrite(11, leftSpeedVal);
  delay(100);  
  }
	
void rotate (){//called by void loop(), which isDriving = false
  correctionAngle = round(targetAngle - currentAngle);
  Serial.print("correctionAngle = "); Serial.println(correctionAngle);
  int targetZRate; // targetGyroZ is amount of yaw, regardless of  using Gyro or compass
  
  if (abs(correctionAngle) <= 5){
    stopCar();
  } else {
    if (correctionAngle > 0) { //turn left
      left();  // left and right set direction pins but not PWM motor speed
    } else if (correctionAngle < 0) {//turn right
      right();
    }

    //setting up propoertional control, see Step 3 on the website
    // target Z rate - wheel speed for rotation
    if (abs(correctionAngle) > 30){
      targetZRate = 60;
    } else {
      targetZRate = proportionalRate * abs(correctionAngle);
    }
    
    if (round(correctionAngle) == 0){
      ;
    } else if (targetZRate > 0){
      leftSpeedVal = changeSpeed(leftSpeedVal, +targetZRate);
    } else {
      leftSpeedVal = changeSpeed(leftSpeedVal, -targetZRate);
    }
    rightSpeedVal = leftSpeedVal;
    analogWrite(rightSpeed, rightSpeedVal);
    analogWrite(leftSpeed, leftSpeedVal);
  }
}   

int changeSpeed (int motorSpeed, int target){
  motorSpeed = target;
  if (motorSpeed > maxSpeed){ //to prevent motorSpeed from exceeding 255, which is a problem when using analogWrite
    motorSpeed = maxSpeed;
  } else if (motorSpeed < minSpeed){
    motorSpeed = minSpeed;
  }
  return motorSpeed;
}

void forward(){ //drives the car forward, assuming leftSpeedVal and rightSpeedVal are set high enough
  digitalWrite(right1, HIGH); //the right motor rotates FORWARDS when right1 is HIGH and right2 is LOW
  digitalWrite(right2, LOW);
  digitalWrite(left1, HIGH);
  digitalWrite(left2, LOW);
}

void left(){ //rotates the car left, assuming speed leftSpeedVal and rightSpeedVal are set high enough
  digitalWrite(right1, LOW);
  digitalWrite(right2, HIGH);
  digitalWrite(left1, HIGH);
  digitalWrite(left2, LOW);
}

void right(){
  digitalWrite(right1, HIGH);
  digitalWrite(right2, LOW);
  digitalWrite(left1, LOW);
  digitalWrite(left2, HIGH);
}

void stopCar(){
  digitalWrite(right1, LOW);
  digitalWrite(right2, LOW);
  digitalWrite(left1, LOW);
  digitalWrite(left2, LOW);
  analogWrite(rightSpeed, 0);
  analogWrite(leftSpeed, 0);
}

int findTargetAngle(int currentAngle,int correction){
  int targetAngle;
  targetAngle = currentAngle + correction;
  if (targetAngle >= 360){
    targetAngle = targetAngle - 360.0;
  }
  if (targetAngle < 0){
    targetAngle = targetAngle + 360.0;
  }
  return(targetAngle);
}

int findCorrectionAngle(int currentAngle, int targetAngle){
  int correctionAngle;
  correctionAngle = targetAngle - currentAngle;
  if (correctionAngle < -180){
    correctionAngle = correctionAngle + 360;
  }
  if (correctionAngle > 180){
    correctionAngle = correctionAngle - 360;
  }
  return(correctionAngle);
}
  
