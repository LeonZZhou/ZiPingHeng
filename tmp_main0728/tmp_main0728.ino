/*************************************************************************
  > File Name: mpu6050_kalman.c
  > Author: Leon Zhou 
  > Mail: 156786363@qq.com
  > Created Time: Wednesday 13 July 2016 02:09:39 PM CST
 ************************************************************************/

//STBY-A0
//SCL-A5
//SDA-A4
//INT-2 (Optional)

#include <Kalman.h>
#include <Wire.h>
#include <math.h>

//mpu6050_kalman
float fRad2Deg = 57.295779513f; //将弧度转为角度的乘数
const int MPU = 0x68; //MPU-6050的I2C地址
const int nValCnt = 7; //一次读取寄存器的数量

const int nCalibTimes = 1000; //校准时读数的次数
int calibData[nValCnt]; //校准数据

unsigned long nLastTime = 0; //上一次读数的时间
float fLastRoll = 0.0f; //上一次滤波得到的Roll角
float fLastPitch = 0.0f; //上一次滤波得到的Pitch角
Kalman kalmanRoll; //Roll角滤波器
Kalman kalmanPitch; //Pitch角滤波器

//TB6612FNG
const int STBY = 9; //standby,当STBY=LOW,motor断电.（option)
const int AIN1 = 8;
const int AIN2 = 7;
const int BIN1 = 10;
const int BIN2 = 11;
const int PWMA = 5;
const int PWMB = 6;
 //HC-SR04
const int TrigPin = 12;
const int EchoPin = 13;
float cm;
 //

void setup() {
  Serial.begin(9600); //初始化串口，指定波特率
    //TB6612FNG
    pinMode(STBY,OUTPUT);
    pinMode(PWMA,OUTPUT);
    pinMode(PWMB,OUTPUT);
    pinMode(AIN1,OUTPUT);
    pinMode(AIN2,OUTPUT);
    pinMode(BIN1,OUTPUT);
    pinMode(BIN2,OUTPUT);
    //HC-SR04
    pinMode(TrigPin,OUTPUT);
    pinMode(EchoPin,INPUT);
  Wire.begin(); //初始化Wire库
  mpu6050_WriteMPUReg(0x6B, 0); //启动MPU6050设备

  mpu6050_Calibration(); //执行校准
  nLastTime = micros(); //记录当前时间
  PID_init();
}

void loop(){
  mpu6050_kalmanLoop();
 // float pwm = PID_realize(100.0);
  int pwm = 100;
  motor_move(0,int(pwm),0); 
  motor_move(1,int(pwm),0); 
}



                    //自定义函数部分//
//HC-SR04,测距
float hcsr04_UltraDis()
{
    float cm;
    digitalWrite(TrigPin,LOW);
    delayMicroseconds(2);
    digitalWrite(TrigPin,HIGH);
    delayMicroseconds(10);
    digitalWrite(TrigPin,LOW);
    cm = pulseIn(EchoPin,HIGH)/58.0;
    cm = (int(cm*100.0))/100.0; //保留两位小数
    return cm;
}

//motor控制
void motor_move(int motor, int speed, int direction){
//Move specific motor at speed and direction
//motor: 0 for B 1 for A
//speed: 0 is off, and 255 is full speed
//direction: 0 clockwise, 1 counter-clockwise

  digitalWrite(STBY, HIGH); //disable standby

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if(motor == 1){
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  }else{
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }
}
//motor停止
void motor_stop(){
//enable standby  
  digitalWrite(STBY, LOW); 
}

// PID
struct _pid{
  float SetSpeed;
  float ActualSpeed;
  float err;
  float err_last;
  float Kp,Ki,Kd;
  float voltage;
  float integral;
  float umax;
  float umin;
}pid;

void PID_init(){
  pid.SetSpeed=0.0;
  pid.ActualSpeed=0.0;
  pid.err=0.0;
  pid.err_last=0.0;
  pid.voltage=0.0;
  pid.integral=0.0;
  pid.Kp=0.2;
  pid.Ki=0.1;
  pid.Kd=0.2;
  pid.umax=255;
  pid.umin=-255;
}

float PID_realize(float speed){
  int index;
  pid.SetSpeed=speed;
  pid.err=pid.SetSpeed-pid.ActualSpeed;

  if(pid.ActualSpeed>pid.umax){
    if(abs(pid.err)>100){
      index=0;
    }else{
    index=1;
    if(pid.err<0){
      pid.integral+=pid.err;
    }
    }
  }else if(pid.ActualSpeed<pid.umin){
  if(abs(pid.err)>100){
    index=0;
  }else{
    index=1;
    if(pid.err>0){
      pid.integral+=pid.err;
    }
  }
  }else{
    if(abs(pid.err)>100){
      index=0;
    }else{
      index=1;
      pid.integral+=pid.err;
    }
  }
  pid.voltage=pid.Kp*pid.err+index*pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
  pid.err_last = pid.err;
  pid.ActualSpeed = pid.voltage*1.0;
  return pid.ActualSpeed;
}

//mpu6050_kalman
void mpu6050_kalmanLoop() {
  int readouts[nValCnt];
  mpu6050_ReadAccGyr(readouts); //读出测量值
  
  float realVals[7];
  mpu6050_Rectify(readouts, realVals); //根据校准的偏移量进行纠正

  //计算加速度向量的模长，均以g为单位
  float fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);
  float fRoll = mpu6050_GetRoll(realVals, fNorm); //计算Roll角
  if (realVals[1] > 0) {
    fRoll = -fRoll;
  }
  float fPitch = mpu6050_GetPitch(realVals, fNorm); //计算Pitch角
  if (realVals[0] < 0) {
    fPitch = -fPitch;
  }

  //计算两次测量的时间间隔dt，以秒为单位
  unsigned long nCurTime = micros();
  float dt = (double)(nCurTime - nLastTime) / 1000000.0;
  //对Roll角和Pitch角进行卡尔曼滤波
  float fNewRoll = kalmanRoll.getAngle(fRoll, realVals[4], dt);
  float fNewPitch = kalmanPitch.getAngle(fPitch, realVals[5], dt);
  //跟据滤波值计算角度速
  float fRollRate = (fNewRoll - fLastRoll) / dt;
  float fPitchRate = (fNewPitch - fLastPitch) / dt;
 
 //更新Roll角和Pitch角
  fLastRoll = fNewRoll;
  fLastPitch = fNewPitch;
  //更新本次测的时间
  nLastTime = nCurTime;

  //Serial.print(fNewRoll); Serial.print(',');
  //Serial.print(fRollRate); Serial.print(",");
  //Serial.print(fNewPitch); Serial.print(',');
  //Serial.print(fPitchRate); Serial.print("\n");
  delay(10);
}


//向MPU6050写入一个字节的数据
//指定寄存器地址与一个字节的值
void mpu6050_WriteMPUReg(int nReg, unsigned char nVal) {
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.write(nVal);
  Wire.endTransmission(true);
}

//从MPU6050读出一个字节的数据
//指定寄存器地址，返回读出的值
unsigned char mpu6050_ReadMPUReg(int nReg) {
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.requestFrom(MPU, 1, true);
  Wire.endTransmission(true);
  return Wire.read();
}

//从MPU6050读出加速度计三个分量、温度和三个角速度计
//保存在指定的数组中
void mpu6050_ReadAccGyr(int *pVals) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.requestFrom(MPU, nValCnt * 2, true);
  Wire.endTransmission(true);
  for (long i = 0; i < nValCnt; ++i) {
    pVals[i] = Wire.read() << 8 | Wire.read();
  }
}

//对大量读数进行统计，校准平均偏移量
void mpu6050_Calibration()
{
  float valSums[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0};
  //先求和
  for (int i = 0; i < nCalibTimes; ++i) {
    int mpuVals[nValCnt];
    mpu6050_ReadAccGyr(mpuVals);
    for (int j = 0; j < nValCnt; ++j) {
      valSums[j] += mpuVals[j];
    }
  }
  //再求平均
  for (int i = 0; i < nValCnt; ++i) {
    calibData[i] = int(valSums[i] / nCalibTimes);
  }
  calibData[2] += 16384; //设芯片Z轴竖直向下，设定静态工作点。
}

//算得Roll角。
float mpu6050_GetRoll(float *pRealVals, float fNorm) {
  float fNormXZ = sqrt(pRealVals[0] * pRealVals[0] + pRealVals[2] * pRealVals[2]);
  float fCos = fNormXZ / fNorm;
  return acos(fCos) * fRad2Deg;
}

//算得Pitch角。
float mpu6050_GetPitch(float *pRealVals, float fNorm) {
  float fNormYZ = sqrt(pRealVals[1] * pRealVals[1] + pRealVals[2] * pRealVals[2]);
  float fCos = fNormYZ / fNorm;
  return acos(fCos) * fRad2Deg;
}

//对读数进行纠正，消除偏移，并转换为物理量。
void mpu6050_Rectify(int *pReadout, float *pRealVals) {
  for (int i = 0; i < 3; ++i) {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 16384.0f;
  }
  pRealVals[3] = pReadout[3] / 340.0f + 36.53;
  for (int i = 4; i < 7; ++i) {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 131.0f;
  }
}

