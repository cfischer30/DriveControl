// developed as CompasAzimuthWithCorrection
// converted to a library to incorporate into DriveControl
//  Chris Fischer November 27, 2024

#include <Wire.h> //I2C Arduino Library
//uncomment the max and min values.   Rotate the compass chip throug
// 360 degrees plus vertical.   Record the max, and min.   Calculate the
// average.   Enter the averages as offset values.
// program assumes board is oriented horizontally (xz plane).


#define addr 0x0D //I2C Address for The HMC5883
int xMax = -9999, yMax = -9999, zMax = -9999;
int xMin = 9999, yMin = 9999, zMin = 9999;
int xOffset = -2975, yOffset = -930, zOffset = -187;


void compassSetup() {

  Serial.begin(9600);
  Wire.begin();


  Wire.beginTransmission(addr); //start talking
  Wire.write(0x0B); // Tell the HMC5883 to Continuously Measure
  Wire.write(0x01); // Set the Register
  Wire.endTransmission();
  Wire.beginTransmission(addr); //start talking
  Wire.write(0x09); // Tell the HMC5883 to Continuously Measure
  Wire.write(0x1D); // Set the Register
  Wire.endTransmission();
}

int readCompassAzimuth() {

  int x, y, z, a; //triple axis data

  //Tell the HMC what regist to begin writing data into


  Wire.beginTransmission(addr);
  Wire.write(0x00); //start with register 3.
  Wire.endTransmission();

  //Read the data.. 2 bytes for each axis.. 6 total bytes
  Wire.requestFrom(addr, 6);
  if (6 <= Wire.available()) {
    x = Wire.read(); //MSB  x
    x |= Wire.read() << 8; //LSB  x)
    z = Wire.read(); //MSB  z
    z |= Wire.read() << 8; //LSB z
    y = Wire.read(); //MSB y
    y |= Wire.read() << 8; //LSB y

    x = x - xOffset;
    y = y - yOffset;
    z = z - zOffset;

    if(x > xMax){xMax = x;}
    if(x < xMin){xMin = x;}
    if(y > yMax){yMax = y;}
    if(y < yMin){yMin = y;}
    if(z > zMax){zMax = z;}
    if(z < zMin){zMin = z;}

    
    

  }

  // Show Values
  /*Serial.print("X Value: ");
  Serial.println(x);
  Serial.print("Y Value: ");
  Serial.println(y);
  Serial.print("Z Value: ");
  Serial.println(z);*/
  a = int(atan2(double(x),double(z))*180.000/3.1415);
  if(a < 0){a = 360 + a;}
  //Serial.print("A Value: ");
  //Serial.println(a);
  return(a);
 

}
