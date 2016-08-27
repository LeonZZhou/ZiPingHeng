
// https://zhuanlan.zhihu.com/p/20303537

#include "MeOrion.h"
#include <Wire.h>

MeGyro gyro;
double x, y, z;

void setup()
{
    Serial.begin(9600);
    gyro.begin();
}

void loop()
{
    gyro.update();

    x = gyro.getAngleX();
    Serial.print("x=");
    Serial.print(x);
    y = gyro.getAngleY();
    Serial.print(" y=");
    Serial.print(y);
    z = gyro.getAngleZ();
    Serial.print(" z=");
    Serial.println(z);
    delay(100);
}

void sendData(char *startAddr, int length)
{
    for (int i = 0; i < length; ++i)
    {
        Serial.write(startAddr[length]);
    }
}
