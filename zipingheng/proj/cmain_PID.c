#include <MPU6050.h>

#include <I2Cdev.h>

#include <SoftwareSerial.h>

#include <Wire.h>

#include <PID_v1.h>

#include <Kalman.h>

#include <math.h>

float Angle_ax, Gyro_y, Angle;
double Setpoint, Input, Output;
//Define Variables we'll be connecting to
char val = 's'; int Speed_need = 0; int Turn_need = 0;
float speeds, speeds_filter, positions;
float diff_speeds, diff_speeds_all;

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
//double Kp=1.2, Ki=0.5, Kd=0.1;
float Kp = 10, Ki = 0.5, Kd = 0.06, Ksp = 2.8, Ksi = 0.11;
double aggKp = 4, aggKi = 0.2, aggKd = 1;
double consKp = 1, consKi = 0.05, consKd = 0.25;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT) ; //PID对象声明
int PinA_right = 2; //interrupt 0
int PinA_left = 3; //interrupt 1
int E_left = 6; //ENA
int M_right = 7;
int M_right2 = 8;
int E_right = 5; //ENB
int M_left = 9;
int M_left2 = 10;
//////////////////////////////////////////////////////////////////////////////
int PWM_right = 0; int PWM_left = 0;
int PWM_left_least = 87; int PWM_right_least = 88; //left:77,right:78
///////////////////////////////interrupt for Speed/////////////////////////////////
int count_right = 0;
int count_left  = 0;
int old_time = 0;
int flag;

void setup() {
  // put your setup code here, to run once:
  setupPID();  //PID初始化
  Wire.begin();
  Serial.begin(9600);
  ///////////////////////////////////////////////////////////////////
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
  ////////////////////////pin mode////////////////////////////////////////
  pinMode(E_right, OUTPUT); pinMode(M_right, OUTPUT); pinMode(M_right2, OUTPUT); //right
  pinMode(E_left, OUTPUT);  pinMode(M_left, OUTPUT); pinMode(M_left2, OUTPUT); //left
  pinMode(PinA_right, INPUT); pinMode(PinA_left, INPUT); // in 0 in 1
  Serial.println("Pin mode ...");
  /////////////////////////////interrupt/////////////////////////////////////////////
  attachInterrupt(0, Code_right, FALLING); attachInterrupt(1, Code_left, FALLING);


}

void loop() {
  // put your main code here, to run repeatedly:
  //Kalman_Filter(Angle_ax, Gyro_y); //卡尔曼融合获取angle
  Angle_Calcu();
  Input = Angle;
  myPID.Compute();  //PID计算获取 Output; Contains the pid algorithm. it should be called once every loop(). Most of the time it will just return without doing anything. At a frequency specified by SetSampleTime it will calculate a new Output.
//  Drive(Output);   //根据Output驱动电机
  if (abs(Angle) > 40)
  {SetMotorVoltage(0, 0);} //角度大于45度 停止PWM输出
  else {
    myPID.Compupe();
  }




  double gap = abs(Setpoint - Input); //distance away from setpoint
  if (gap < 10) //gab ?
  { //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
    //we're far from setpoint, use aggressive tuning parameters
    myPID.SetTunings(aggKp, aggKi, aggKd);
  }
}


void setupPID() {
  Input = 0;
  Setpoint = 17;  //我的小车自平衡角度为17
  myPID.SetSampleTime(100);  //控制器的采样时间100ms //The default is 200mS; SampleTime: How often, in milliseconds, the PID will be evaluated. (int>0)
  //myPID.SetOutputLimits(0, 2000);//SetOutputLimits(min, max); double main,max
  myPID.SetMode(AUTOMATIC);//Specifies whether the PID should be on (AUTOMATIC) or off (MANUL.) The PID defaults to the off position when created.
  myPID.SetControllerDirection(DIRECT); //Direction: DIRECT (like a car) or REVERSE (like a refrigerator)
}
void Angle_Calcu(void)         {
  Angle_ax = (ax + 1942) / 238.2 ; //去除零点偏移1942,//16384*3.14/1.2*180//弧度转换为度,
  Gyro_y = -(gy - 119.58) / 16.4; //去除零点偏移119,2000deg/s 16.4 LSB/(deg/s)250---131 ，负号为方向处理
  //Serial.print("Angle_ax,Angle_gy");Serial.print(Angle_ax);Serial.println(Angle_gy);
  Angle = 0.97 * (Angle + Gyro_y * 0.0005) + 0.03 * Angle_ax;
  //Kalman_Filter(Angle_ax,Gyro_y);       //卡尔曼滤波计算倾角
}
void PWM_Calcu(void)
{
  if (abs(Angle) > 40)
  {SetMotorVoltage(0, 0);} //角度大于45度 停止PWM输出
  else
  { //Speed_need=30;Turn_need=0;positions=80;
    //////////////////////
    speeds = (count_left + count_right) * 0.5;
    diff_speeds = count_left - count_right;
    diff_speeds_all += diff_speeds;
    speeds_filter *= 0.85; //一阶互补滤波
    speeds_filter += speeds * 0.15;
    positions += speeds_filter;
    positions += Speed_need;
    //positions = 0.85*speeds_filter + 0.15*speeds + positions + Speed_need;
    //positions = Speed_need + (positions + (0.15*speeds + (0.85*speeds_filter)))
    positions = constrain(positions, -2300, 2300);//抗积分饱和
    ////////////////////
    Output = Kp * Angle + Kd * Gyro_y + Ksp * speeds_filter + Ksi * positions ;
    //Serial.print("Output");Serial.println(Output);
    if (Turn_need == 0) {
      PWM_right = Output - diff_speeds_all;
      PWM_left = Output + diff_speeds_all;
    }
//貌似有毛病
    PWM_right = Output + Turn_need;
    PWM_left = Output - Turn_need;

    if (PWM_right >= 0) {PWM_right += PWM_right_least;} else {PWM_right -= PWM_right_least;}
    if (PWM_left >= 0) {PWM_left += PWM_left_least;} else {PWM_left -= PWM_left_least;}

    SetMotorVoltage(PWM_left, PWM_right);
  }
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


