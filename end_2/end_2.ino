#include <MPU6050.h>
#include <I2Cdev.h>
#include <Wire.h>
#include <Kalman.h>
#include <math.h>


double Output;
char hc06Val = 's';
float cm,Angle_ax, Gyro_y, Angle,speeds, speeds_filter, positions,diff_speeds, diff_speeds_all;
int PWM_right = 0;
int PWM_left = 0;
int PWM_left_least = 87; 
int PWM_right_least = 88;
int count_right = 0;
int count_left  = 0;
int old_time = 0;
int Speed_need = 0; 
int Turn_need = 0;
int flag;
int cmContralVarial;

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
double Kp = 10, Kd = 0.06, Ksp = 2.8, Ksi = 0.11;

const int PinA_right = 2; //interrupt 0
const int PinA_left = 3; //interrupt 1
const int E_left = 6; //ENA
const int M_right = 7;
const int M_right2 = 8;
const int E_right = 5; //ENB
const int M_left = 9;
const int M_left2 = 10;
const int TrigPin = 12;
const int EchoPin = 13;

void setup() {
  Angle = 0;
  Wire.begin();
  Serial.begin(9600);
  accelgyro.reset();//reset
  delay(1);
  accelgyro.setClockSource(MPU6050_CLOCK_PLL_YGYRO);//PLL with Y Gyro reference*/
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//0x1B Value 0x18 2000°/s
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);//0x1C Value 0x18 16g
  accelgyro.setDLPFMode(MPU6050_DLPF_BW_5);//0x06 means acc 5Hz delay19.0ms Gyro 5Hz delay 18.6ms
  accelgyro.setTempSensorEnabled(true);//disable temsensor
  accelgyro.setSleepEnabled(false);
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  pinMode(E_right, OUTPUT); pinMode(M_right, OUTPUT); pinMode(M_right2, OUTPUT);
  pinMode(E_left, OUTPUT);  pinMode(M_left, OUTPUT); pinMode(M_left2, OUTPUT);
  pinMode(PinA_right, INPUT); pinMode(PinA_left, INPUT); // in 0 in 1
  pinMode(TrigPin,OUTPUT); pinMode(EchoPin,INPUT);
  Serial.println("Pin mode ...");
  attachInterrupt(0, Code_right, FALLING); attachInterrupt(1, Code_left, FALLING);
}

void loop() {
  float cm = UltraDis();
  if(cm>25){
    cmContralVarial = 0;
  }else{
    cmContralVarial = 10;
  }

 if (Serial.available() > 0){
     hc06Val = Serial.read();
     Serial.println(hc06Val);
   }
 switch(hc06Val){
 case 'a':    //Go
     Speed_need=30;
     Turn_need=cmContralVarial;
     positions=80;
     break;
 case 'b':    //right
     Speed_need=10;
     Turn_need=10;
     positions=10;
     break;
 case 'c':    //left
     Speed_need=10;
     Turn_need=-10;
     positions=10;
     break;
 case 'd':    //stop
     Speed_need=0;
     Turn_need=0;
     positions=0;
     break;
 default:    //stop
     Speed_need=0;
     Turn_need=0;
     positions=0;
     break;
  }

  Angle_Calcu();
  PWM_Calcu();
}


void PWM_Calcu(void)
{
  if (abs(Angle)>40)
    {SetMotorVoltage(0,0);}
    else
    { //Speed_need=30;Turn_need=0;positions=80;
      speeds=(count_left + count_right)*0.5;
      diff_speeds = count_left - count_right;
      diff_speeds_all += diff_speeds;
      speeds_filter *=0.85;  
      speeds_filter +=speeds*0.15;
      positions += speeds_filter;
      positions += Speed_need;
      positions = constrain(positions, -2300, 2300);
      Output = Kp*Angle + Kd*Gyro_y + Ksp*speeds_filter + Ksi*positions;
      //Serial.print("Output");Serial.println(Output);
      if(Turn_need==0){PWM_right=Output-diff_speeds_all;
      PWM_left=Output+diff_speeds_all;}
      PWM_right=Output+Turn_need;
      PWM_left=Output-Turn_need;
      
      if(PWM_right>=0){PWM_right+=PWM_right_least;}else{PWM_right-=PWM_right_least;}
      if(PWM_left>=0){PWM_left+=PWM_left_least;}else{PWM_left-=PWM_left_least;}
 
      SetMotorVoltage(PWM_left,PWM_right);}
      count_left = 0;
      count_right = 0;
}

void SetMotorVoltage(int nLeftVol, int nRightVol) {
  if (nLeftVol >= 0)
  { digitalWrite(M_left, LOW);
    digitalWrite(M_left2, HIGH);
  } else {
    digitalWrite(M_left, HIGH);
    digitalWrite(M_left2, LOW);
    nLeftVol = -nLeftVol;
  }
  if (nRightVol >= 0) {
    digitalWrite(M_right, LOW);
    digitalWrite(M_right2, HIGH);
  } else {
    digitalWrite(M_right, HIGH);
    digitalWrite(M_right2, LOW);
    nRightVol = -nRightVol;
  }
  if (nLeftVol > 255) { nLeftVol = 255 ; } //防止PWM值超过255
  if (nRightVol > 255) { nRightVol = 255 ; } //防止PWM值超过255
  analogWrite(E_left, nLeftVol);
  analogWrite(E_right, nRightVol);
}
void Code_right() {  if (Output >= 0) {count_right += 1;} else {count_right -= 1;} } //if only +,can't stand up 编码器码盘计数加一
void Code_left() {  if (Output >= 0) {count_left += 1;} else {count_left -= 1;}} // 编码器码盘计数加一
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

void Angle_Calcu(void)         {
  Angle_ax = (ax + 1942) / 238.2 ; //去除零点偏移1942,//16384*3.14/1.2*180//弧度转换为度
  Gyro_y = -(gy - 119.58) / 16.4; //去除零点偏移119,2000deg/s 16.4 LSB/(deg/s)250---131 ，负号为方向处理
  //Serial.print("Angle_ax,Angle_gy");Serial.print(Angle_ax);Serial.println(Angle_gy);
  Angle = 0.97 * (Angle + Gyro_y * 0.0005) + 0.03 * Angle_ax;
}


