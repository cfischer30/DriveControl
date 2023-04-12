/*   Available functions

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


void moveControl(){
  readAcceleration();
  previousTime = currentTime;
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000;
  readGyro();
  GyroX -= GyroErrorX;
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;
  yaw += GyroZ * elapsedTime;
  currentAngle = yaw;
  Serial.print("Current Angle = ");Serial.println(currentAngle);
  driving();
  }
  
  
  // rewritten driving() function by Chris Fischer
  
  void driving(){   // called my moveControl
  forward();   // sets forward & reverse pins appropriately on H bridge
	deltaAngle = targetAngle - currentAngle;
	speedCorrection = int(deltaAngle * proportionalRate);
	
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
		
	Serial.print("deltaAngle = ");Serial.println(deltaAngle);  	
	Serial.print("left right speed "); 
    //Serial.print(leftSpeed); Serial.print(" "); Serial.println(rightSpeed);  

  Serial.print(leftSpeedVal); Serial.print(" "); Serial.println(rightSpeedVal);  
		
	analogWrite(rightSpeed, rightSpeedVal);
  analogWrite(leftSpeed, leftSpeedVal);
	
	
		
  }
		




/* Original "driving" function

void driving (){//called by void loop(), which isDriving = true
  int deltaAngle = round(targetAngle - angle); //rounding is neccessary, since you never get exact values in reality
  Serial.print("deltaAngle = ");Serial.println(deltaAngle);  
  forward();  // sets direction
  //if (deltaAngle != 0){
    if (abs(deltaAngle) > angleTolerance){
   
    Serial.print("before left right speed "); 
    Serial.print(leftSpeedVal); Serial.print(" "); Serial.println(rightSpeedVal);    
    controlSpeed ();
   // rightSpeedVal = maxSpeed;
    Serial.print("before left right speed "); 
    Serial.print(leftSpeedVal); Serial.print(" "); Serial.println(rightSpeedVal);    
    
    analogWrite(rightSpeed, rightSpeedVal);
    analogWrite(leftSpeed, leftSpeedVal);
  }
}
*/
/*
void controlSpeed(){//this function is called by original driving() function
// obsolete in current implementation
  int deltaAngle = round(targetAngle - currentAngle);
  int targetGyroZ;
  
  //setting up propoertional control, see Step 3 on the website
  if (deltaAngle > 30){
     // targetGyroZ = 60;
     targetGyroZ = maxRate;
  } else if (deltaAngle < -30){
    // targetGyroZ = -60;
    targetGyroZ = -maxRate;    
  } else {
    targetGyroZ = proportionalRate * deltaAngle;
  }
  
  if (round(targetGyroZ - GyroZ) == 0){
    ;
  } else if (targetGyroZ > GyroZ){
    leftSpeedVal = changeSpeed(leftSpeedVal, -1); //would increase GyroX
  } else {
    leftSpeedVal = changeSpeed(leftSpeedVal, +1);
  }
}
*/
void rotate (){//called by void loop(), which isDriving = false
  int deltaAngle = round(targetAngle - currentAngle);
  Serial.print("deltaAngle = "); Serial.println(deltaAngle);
  int targetGyroZ;
  if (abs(deltaAngle) <= 1){
    stopCar();
  } else {
    if (currentAngle > targetAngle) { //turn left
      left();
    } else if (currentAngle < targetAngle) {//turn right
      right();
    }

    //setting up propoertional control, see Step 3 on the website
    if (abs(deltaAngle) > 30){
      targetGyroZ = 60;
    } else {
      targetGyroZ = proportionalRate * abs(deltaAngle);
    }
    
    if (round(targetGyroZ - abs(GyroZ)) == 0){
      ;
    } else if (targetGyroZ > abs(GyroZ)){
      leftSpeedVal = changeSpeed(leftSpeedVal, +1); //would increase abs(GyroX)
    } else {
      leftSpeedVal = changeSpeed(leftSpeedVal, -1);
    }
    rightSpeedVal = leftSpeedVal;
    analogWrite(rightSpeed, rightSpeedVal);
    analogWrite(leftSpeed, leftSpeedVal);
  }
}   

int changeSpeed (int motorSpeed, int increment){
  motorSpeed += increment;
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


