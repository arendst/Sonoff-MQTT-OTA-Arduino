/*
Original Code by Theo Arends. Modified to use Arduino OneWire Library and provide support for multiple DS18x20 Sensors.

Copyright (c) 2016 Theo Arends.  All rights reserved.

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

#define W1_SKIP_ROM 0xCC
#define W1_CONVERT_TEMP 0x44
#define W1_READ_SCRATCHPAD 0xBE

#ifdef SEND_TELEMETRY_DS18x20

#include <OneWire.h>

OneWire ds(DSB_PIN);

uint8_t ds18x20_search()
{
  uint8_t num_sensors=0;
  uint8_t sensor=0;
  
  uint8_t i;

  ds.reset_search();
  for(num_sensors=0;num_sensors<DS18X20_MAX_SENSORS;num_sensors)
  {
    if (!ds.search(addr[num_sensors])) 
    {
      ds.reset_search();
      break;
    }    

    // If CRC Ok and Type DS18S20 or DS18B20
    if ((OneWire::crc8(addr[num_sensors],7) == addr[num_sensors][7]) & 
       ((addr[num_sensors][0]==0x10) | (addr[num_sensors][0]==0x28)))
       num_sensors++;
  }

  return num_sensors;
}

void ds18x20_convert()
{
  ds.reset();
  ds.write(W1_SKIP_ROM);        // Address all Sensors on Bus
  ds.write(W1_CONVERT_TEMP);    // start conversion, no parasite power on at the end

  delay(750);                   // 750ms should be enough for 12bit conv
}                             

boolean ds18x20_read(uint8_t sensor, float &t)
{
  byte data[12];
  uint8_t sign=1;
  uint8_t i=0;
  float temp9=0.0;
  uint8_t present = 0;

  t = NAN;
  
  ds.reset();
  ds.select(addr[sensor]);    
  ds.write(W1_READ_SCRATCHPAD); // Read Scratchpad

  for (i=0;i<9;i++) 
    data[i]=ds.read();

  if(OneWire::crc8(data,8)==data[8])
  {
    switch(addr[sensor][0])
    {
      case 0x10 : if(data[1]>0x80) sign=-1; //App-Note kann sonst zu Vorzeichenfehler kommen
                  if(data[0] & 1) temp9=((data[0]>>1)+0.5)*sign; else temp9=(data[0]>>1)*sign;
                  t=(temp9-0.25)+((16.0-data[6])/16.0);
                  break;
      case 0x28 : t=((data[1]<<8)+data[0])*0.0625;
                  break;
    }         
  }
  return (!isnan(t));
}
#endif  // SEND_TELEMETRY_DS18x20
