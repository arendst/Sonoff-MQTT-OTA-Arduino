/*
 Copyright (c) 2016 Heiko Krupp.  All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 - Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 - Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/

#ifdef SEND_TELEMETRY_I2C

#define BMP180_ADDR          0x77

#define BMP180_READREG       0xD0
#define BMP180_REG_CONTROL   0xF4
#define BMP180_REG_RESULT    0xF6
#define BMP180_TEMPERATURE   0x2E
#define BMP180_PRESSURE3     0xF4 // Max. oversampling -> OSS = 3

#define BMP180_AC1           0xAA
#define BMP180_AC2           0xAC
#define BMP180_AC3           0xAE
#define BMP180_AC4           0xB0
#define BMP180_AC5           0xB2
#define BMP180_AC6           0xB4
#define BMP180_VB1           0xB6
#define BMP180_VB2           0xB8
#define BMP180_MB            0xBA
#define BMP180_MC            0xBC
#define BMP180_MD            0xBE

#define BMP180_OSS           3

int16_t cal_ac1,cal_ac2,cal_ac3,cal_b1,cal_b2,cal_mc,cal_md;
uint16_t cal_ac4,cal_ac5,cal_ac6;
int32_t bmp180_b5=0;

uint8_t bmp180_detect()
{
	if(i2c_read8(BMP180_ADDR,BMP180_READREG) == 0x55) // Read sensor ID
		return 1;
	else return 0;
}

uint8_t bmp180_calibration()
{
  cal_ac1 = i2c_read16(BMP180_ADDR,BMP180_AC1);
  cal_ac2 = i2c_read16(BMP180_ADDR,BMP180_AC2);
  cal_ac3 = i2c_read16(BMP180_ADDR,BMP180_AC3);
  cal_ac4 = i2c_read16(BMP180_ADDR,BMP180_AC4);
  cal_ac5 = i2c_read16(BMP180_ADDR,BMP180_AC5);
  cal_ac6 = i2c_read16(BMP180_ADDR,BMP180_AC6);
  cal_b1  = i2c_read16(BMP180_ADDR,BMP180_VB1);
  cal_b2  = i2c_read16(BMP180_ADDR,BMP180_VB2);
  cal_mc  = i2c_read16(BMP180_ADDR,BMP180_MC);
  cal_md  = i2c_read16(BMP180_ADDR,BMP180_MD);

  // Check for Errors in calibration data. Value never is 0x0000 or 0xFFFF
  if(!cal_ac1 | !cal_ac2 | !cal_ac3 | !cal_ac4 | !cal_ac5 | 
     !cal_ac6 | !cal_b1 | !cal_b2 | !cal_mc | !cal_md) 
     return 0;

  if((cal_ac1==0xFFFF)|
     (cal_ac2==0xFFFF)|
     (cal_ac3==0xFFFF)|
     (cal_ac4==0xFFFF)|
     (cal_ac5==0xFFFF)|
     (cal_ac6==0xFFFF)|
     (cal_b1==0xFFFF)|
     (cal_b2==0xFFFF)|
     (cal_mc==0xFFFF)|
     (cal_md==0xFFFF)) 
     return 0;

  return 1;
}

double bmp180_readTemperature()
{
  i2c_write8(BMP180_ADDR,BMP180_REG_CONTROL,BMP180_TEMPERATURE);
  delay(5); // 5ms conversion time
  int ut=i2c_read16(BMP180_ADDR,BMP180_REG_RESULT);
  int32_t x1 = (ut - (int32_t)cal_ac6) * ((int32_t)cal_ac5) >> 15;
  int32_t x2 = ((int32_t)cal_mc << 11) / (x1+(int32_t)cal_md);
  bmp180_b5=x1+x2;
  
  return ((bmp180_b5+8)>>4)/10.0;
}

double bmp180_readPressure(double T)
{
  int32_t p;
  uint8_t  msb,lsb,xlsb;
  
  i2c_write8(BMP180_ADDR,BMP180_REG_CONTROL,BMP180_PRESSURE3); // Highest resolution
  delay(26); // 26ms conversion time at ultra high resolution
  
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(BMP180_REG_RESULT);
  Wire.endTransmission();
  Wire.requestFrom(BMP180_ADDR,3);
  if(3 == Wire.available())
  {
    msb=Wire.read();
    lsb=Wire.read();
    xlsb=Wire.read();
  } else return 0.0;
  
  uint32_t up=(uint32_t)(((uint32_t)(msb<<16)|(uint32_t)(lsb<<8)|(uint32_t)xlsb)>>(8-BMP180_OSS));

  int32_t b6 = bmp180_b5 - 4000;
  int32_t x1 = ((int32_t)cal_b2 * ( (b6 * b6)>>12 )) >> 11;
  int32_t x2 = ((int32_t)cal_ac2 * b6) >> 11;
  int32_t x3 = x1 + x2;
  int32_t b3 = ((((int32_t)cal_ac1*4 + x3) << BMP180_OSS) + 2)>>2;
  
  x1 = ((int32_t)cal_ac3 * b6) >> 13;
  x2 = ((int32_t)cal_b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  uint32_t b4 = ((uint32_t)cal_ac4 * (uint32_t)(x3 + 32768)) >> 15;
  uint32_t b7 = ((uint32_t)up - b3) * (uint32_t)( 50000UL >> BMP180_OSS);

  if (b7 < 0x80000000) {
    p = (b7 * 2) / b4;
  } else {
    p = (b7 / b4) * 2;
  }
  
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  
  p = p + ((x1 + x2 + (int32_t)3791)>>4);
  return p/100.0;  // convert to mbar  
}

double bmp180_calcSealevelPressure(float pAbs, float altitude_meters)
{
  double pressure = pAbs*100.0;
  return (double)(pressure / pow(1.0-altitude_meters/44330, 5.255))/100.0;
}
#endif //SEND_TELEMETRY_I2C
