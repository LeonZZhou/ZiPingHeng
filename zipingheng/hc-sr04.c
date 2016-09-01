/*************************************************************************
	> File Name: hc-sr04.c
	> Author:
	> Mail:
	> Created Time: Wednesday 13 July 2016 02:42:09 PM CST
 ************************************************************************/

const int TrigPin = 2;
const int EchoPin = 13;
float cm;
void setup()
{
    Serial.begin(9600);
    pinMode(TrigPin,OUTPUT);
    pinMode(EchoPin,INPUT);
}



float UltraDis()
{
    digitalWrite(TrigPin,LOW);
    delayMicroseconds(2);
    digitalWrite(TrigPin,HIGH);
    delayMicroseconds(10);
    digitalWrite(TrigPin,LOW);
    cm = pulseIn(EchoPin,HIGH)/58.0;
    cm = (int(cm*100.0))/100.0; //保留两位小数
    return cm;
}
