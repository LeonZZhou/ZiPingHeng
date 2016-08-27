#include <Wire.h>

byte val = 0;

void setup()
{
  Wire.begin(); // join i2c bus
}

void loop()
{
  Wire.beginTransmission(44); // transmit to device #44 (0x2c)
                              // device address is specified in datasheet
 for(;val<99;++val){
  Wire.write(val);  }           // sends value byte  
  Wire.endTransmission();     // stop transmitting
  delay(500);
}
