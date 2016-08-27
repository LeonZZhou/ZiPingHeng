#include "Arduino.h"
#include <MPU6050.h>
#include <I2Cdev.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <PID_v1.h>
#include <Kalman.h>
#include <math.h>

/////->
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

int PWM_right = 0; int PWM_left = 0;
int PWM_left_least = 87; int PWM_right_least = 88; //left:77,right:78
///////////////////////////////interrupt for Speed/////////////////////////////////
int count_right = 0;
int count_left  = 0;
int old_time = 0;
int flag;
/////<-
void setup(){
/////->
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
/////<-
}

void loop(){
  Angle_Calcu();
  Input = Angle;
  myPID.Compute();  //PID计算获取 Output; Contains the pid algorithm. it should be called once every loop(). Most of the time it will just return without doing anything. At a frequency specified by SetSampleTime it will calculate a new Output.
//  Drive(Output);   //根据Output驱动电机
  if (abs(Angle) > 40)
  {SetMotorVoltage(0, 0);} //角度大于45度 停止PWM输出
  else {
    myPID.Compupe();
}

//sick
void setupPID() {
  Input = 0;
  Setpoint = 17;  //我的小车自平衡角度为17
  myPID.SetSampleTime(100);  //控制器的采样时间100ms //The default is 200mS; SampleTime: How often, in milliseconds, the PID will be evaluated. (int>0)
  //myPID.SetOutputLimits(0, 2000);//SetOutputLimits(min, max); double main,max
  myPID.SetMode(AUTOMATIC);//Specifies whether the PID should be on (AUTOMATIC) or off (MANUL.) The PID defaults to the off position when created.
  myPID.SetControllerDirection(DIRECT); //Direction: DIRECT (like a car) or REVERSE (like a refrigerator)
}

//1
class PID //.h
{
  public:
  //Constants used in some of the functions below
  #define AUTOMATIC 1
  #define MANUAL  0
  #define DIRECT  0
  #define REVERSE  1
  //commonly used functions **************************************************************************
    PID(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and
        double, double, double, int);     //   Setpoint.  Initial tuning parameters are also set here
    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)
    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively
    void SetOutputLimits(double, double); //clamps the output to a specific range. 0-255 by default, but
                      //it's likely the user will want to change this depending on
                      //the application
  //available but not commonly used functions ********************************************************
    void SetTunings(double, double,       // * While most users will set the tunings once in the
                    double);            //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
  void SetControllerDirection(int);   // * Sets the Direction, or "Action" of the controller. DIRECT
                      //   means the output will increase when error is positive. REVERSE
                      //   means the opposite.  it's very unlikely that this will be needed
                      //   once it is set in the constructor.
    void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which
                                          //   the PID calculation is performed.  default is 100
  //Display functions ****************************************************************
  double GetKp();             // These functions query the pid for interal values.
  double GetKi();             //  they were created mainly for the pid front-end,
  double GetKd();             // where it's important to know what is actually
  int GetMode();              //  inside the PID.
  int GetDirection();           //
  private:
  void Initialize();
  double dispKp;        // * we'll hold on to the tuning parameters in user-entered
  double dispKi;        //   format for display purposes
  double dispKd;        //
  double kp;                  // * (P)roportional Tuning Parameter
  double ki;                  // * (I)ntegral Tuning Parameter
  double kd;                  // * (D)erivative Tuning Parameter
  int controllerDirection;
    double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;             //   This creates a hard link between the variables and the
    double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
  unsigned long lastTime;
  double ITerm, lastInput;
  unsigned long SampleTime;
  double outMin, outMax;
  bool inAuto;
};

//2
PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int ControllerDirection)
{

    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
  inAuto = false;

  PID::SetOutputLimits(0, 255);       //default output limit corresponds to
                        //the arduino pwm limits

    SampleTime = 100;             //default Controller Sample Time is 0.1 seconds

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd);

    lastTime = millis()- SampleTime;
}

void PID::Initialize()
{
   ITerm = *myOutput;
   lastInput = *myInput;
   if(ITerm > outMax) ITerm = outMax;
   else if(ITerm < outMin) ITerm = outMin;
}

bool PID::Compute()
{
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
    double input = *myInput;
      double error = *mySetpoint - input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      double dInput = (input - lastInput);

      /*Compute PID Output*/
      double output = kp * error + ITerm- kd * dInput;

    if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
    *myOutput = output;

      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
    return true;
   }
   else return false;
}

void PID::SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   dispKp = Kp; dispKi = Ki; dispKd = Kd;

   double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;

  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}

void PID::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

void PID::SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

   if(inAuto)
   {
     if(*myOutput > outMax) *myOutput = outMax;
     else if(*myOutput < outMin) *myOutput = outMin;

     if(ITerm > outMax) ITerm= outMax;
     else if(ITerm < outMin) ITerm= outMin;
   }
}

void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}

void PID::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
    kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   controllerDirection = Direction;
}

double PID::GetKp(){ return  dispKp; }
double PID::GetKi(){ return  dispKi;}
double PID::GetKd(){ return  dispKd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}

//3
void Code_right() {  if (Output >= 0) {count_right += 1;} else {count_right -= 1;} } //if only +,can't stand up 编码器码盘计数加一
void Code_left() {  if (Output >= 0) {count_left += 1;} else {count_left -= 1;}} // 编码器码盘计数加一
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
