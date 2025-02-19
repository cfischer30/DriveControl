#include <Wire.h> //I2C Arduino Library
//uncomment the max and min values.   Rotate the compass chip throug
// 360 degrees plus vertical.   Record the max, and min.   Calculate the
// average.   Enter the averages as offset values.
// program assumes board is oriented horizontally (xz plane).


#define addr 0x0D //I2C Address for The HMC5883  (confirmed)
int xMax = -9999, yMax = -9999, zMax = -9999;
int xMin = 9999, yMin = 9999, zMin = 9999;
int xOffset = -1482, yOffset = -1151, zOffset = 684;


void setup() {

  Serial.begin(9600);
  Wire.begin();
  Serial.println("connected");

  Wire.beginTransmission(addr); //start talking
  Wire.write(0x0B); // Tell the HMC5883 to Continuously Measure
  Wire.write(0x01); // Set the Register
  Wire.endTransmission();
  Wire.beginTransmission(addr); //start talking
  Wire.write(0x09); // Tell the HMC5883 to Continuously Measure
  Wire.write(0x1D); // Set the Register
  Wire.endTransmission();
}

void loop() {

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
  Serial.print("X Value: ");
  Serial.println(x);
  Serial.print("Y Value: ");
  Serial.println(y);
  Serial.print("Z Value: ");
  Serial.println(z);
  a = int(atan2(double(x),double(z))*180.000/3.1415);
  if(a < 0){a = 360 + a;}
  
  Serial.print("xMin: ");
  Serial.println(xMin);
  Serial.print("xMax: ");
  Serial.println(xMax);
  Serial.print("yMin ");
  Serial.println(yMin);
  Serial.print("yMax: ");
  Serial.println(yMax);
  Serial.print("zMin: ");
  Serial.println(zMin);
  Serial.print("zMax: ");
  Serial.println(zMax); 
  Serial.print("xAvg ");
  Serial.println((xMax+xMin)/2.0);
  Serial.print("yAvg "); 
  Serial.println((yMax+yMin)/2.0);
  Serial.print("zAvg ");
  Serial.println((zMax+zMin)/2.0);
  Serial.print("A Value: ");
  Serial.println(a);
  Serial.println(" ");
  delay(2000);
}
