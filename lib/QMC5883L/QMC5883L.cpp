/*
  QMC5883L.cpp - QMC5883L library
  Copyright (c) 2017 e-Gizmo Mechatronix Central
  Rewritten by Amoree.  All right reserved.
  July 10,2017

  SET continuous measurement mode
  OSR = 512
  Full Scale Range = 8G(Gauss)
  ODR = 200HZ
 
*/

#include "QMC5883L.h"

#include <Wire.h>

void QMC5883L::setAddress(uint8_t addr){
  address = addr;
}

void QMC5883L::WriteReg(byte Reg,byte val){
  Wire.beginTransmission(address); //Start
  Wire.write(Reg); // To tell the QMC5883L to get measures continously
  Wire.write(val); //Set up the Register
  Wire.endTransmission();
}

void QMC5883L::init(){
  WriteReg(0x0B,0x01);
  //Define Set/Reset period
  setMode(Mode_Continuous,ODR_200Hz,RNG_8G,OSR_512);
}

void QMC5883L::setMode(uint16_t mode,uint16_t odr,uint16_t rng,uint16_t osr){
  WriteReg(0x09,mode|odr|rng|osr);
  Serial.println(mode|odr|rng|osr,HEX);
}


void QMC5883L::softReset(){
  WriteReg(0x0A,0x80);
}

bool QMC5883L::read(short* x,short* y,short* z){
  Wire.beginTransmission(address);
  Wire.write(0x00);
  Wire.endTransmission();
  byte count = Wire.requestFrom(address, 6);
  *x = Wire.read(); //LSB  x
  *x |= Wire.read() << 8; //MSB  x
  *y = Wire.read(); //LSB  z
  *y |= Wire.read() << 8; //MSB z
  *z = Wire.read(); //LSB y
  *z |= Wire.read() << 8; //MSB y
if(count != 6)
{
	Serial.print("!!! Read returned ");
	Serial.print((int)count);
	Serial.println(" bytes !!!");
}
  return count==6;
}

void QMC5883L::resetCalibration() {
  xhigh = yhigh = 0;
  xlow = ylow = 0;
}

/* returns -1 if unreliable result */
int QMC5883L::readHeading()
{
  short x, y, z;

  if(!read(&x,&y,&z)) return -1;

  /* Update the observed boundaries of the measurements */
  if(x<xlow) xlow = x;
  if(x>xhigh) xhigh = x;
  if(y<ylow) ylow = y;
  if(y>yhigh) yhigh = y;

  /* Bail out if not enough data is available. */
  
  if( xlow==xhigh || ylow==yhigh ) return -2;

  /* Recenter the measurement by subtracting the average */
  
  x -= (xhigh+xlow)/2;
  y -= (yhigh+ylow)/2;

  /* Rescale the measurement to the range observed. */
  
  float fx = (float)x/(xhigh-xlow);
  float fy = (float)y/(yhigh-ylow);

  //int heading = 180.0*atan2(fy,fx)/M_PI;
  int heading = 180.0*atan2(y,x)/M_PI;
  if(heading<=0) heading += 360;
  return heading;
}