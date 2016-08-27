
//copyright by kaiser 20140521 V1.0
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include "Wire.h"            //serial
#include "I2Cdev.h"          //IIC
#include "MPU6050.h"         //acc&gyro Sensor
//Define Variables we'll be connecting to
char val='s';int Speed_need=0;int Turn_need=0;
float speeds,speeds_filter, positions;
float diff_speeds,diff_speeds_all;
////////////////////PID parameter///////////////////////////////////////
double Output=0;
float Kp=10,Kd=0.06,Ksp = 2.8,Ksi = 0.11;        //

////////////////////////////////////////////////
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
/********************角度参数**************/ 
float Gyro_y;            //Y轴陀螺仪数据暂存
float Angle_ax;          //由加速度计算的倾斜角度
float Angle;             //小车最终倾斜角度
//int Setpoint=0;
////////////////////////////////////////Pin assignment///////////////////////////////////////
int PinA_right= 2; //interrupt 0 
int PinA_left= 3; //interrupt 1
int E_left =6;//ENA
int M_right =7; 
int M_right2=8;
int E_right =5; //ENB
int M_left =9;  
int M_left2 =10; 
//////////////////////////////////////////////////////////////////////////////
int PWM_right=0; int PWM_left=0;
int PWM_left_least=87; int PWM_right_least=88;//left:77,right:78
///////////////////////////////interrupt for Speed/////////////////////////////////
int count_right =0;
int count_left  =0;
int old_time=0;
int flag;
void Code_right(){  if(Output>=0){count_right += 1;}else{count_right -= 1;} }//if only +,can't stand up 编码器码盘计数加一 
void Code_left(){  if(Output>=0){count_left += 1;} else{count_left -= 1;}}// 编码器码盘计数加一   
/////////////////////////Right&Left&Stop///////////////////////////////////////////////
void SetMotorVoltage(int nLeftVol, int nRightVol) {
    if(nLeftVol >=0) 
  {  digitalWrite(M_left,LOW);
     digitalWrite(M_left2,HIGH);
   } else {       
     digitalWrite(M_left,HIGH);
     digitalWrite(M_left2,LOW);
     nLeftVol=-nLeftVol;
   }
    if(nRightVol >= 0) {
     digitalWrite(M_right,LOW);
     digitalWrite(M_right2,HIGH);
    } else {
     digitalWrite(M_right,HIGH);
     digitalWrite(M_right2,LOW);
    nRightVol=-nRightVol;
  }
    if(nLeftVol>255) { nLeftVol = 255 ; }   //防止PWM值超过255
    if(nRightVol>255) { nRightVol = 255 ; }//防止PWM值超过255
    analogWrite(E_left,nLeftVol);
    analogWrite(E_right,nRightVol);
}


void setup()
{
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
   pinMode(E_right, OUTPUT); pinMode(M_right, OUTPUT);pinMode(M_right2, OUTPUT);  //right
   pinMode(E_left, OUTPUT);  pinMode(M_left, OUTPUT);pinMode(M_left2, OUTPUT);  //left
   pinMode(PinA_right,INPUT);pinMode(PinA_left,INPUT);// in 0 in 1
   Serial.println("Pin mode ...");
   /////////////////////////////interrupt/////////////////////////////////////////////
   attachInterrupt(0, Code_right, FALLING);attachInterrupt(1, Code_left, FALLING);
   
}

void loop()
{
   if (Serial.available() > 0){val = Serial.read();Serial.println(val);}
     switch(val){
     case 'a':Speed_need=30;Turn_need=0;positions=80;break;//Go
     case 'b':Speed_need=10;Turn_need=-10;positions=10;break;//right
     case 'c':Speed_need=10;Turn_need=10;positions=10;break;//left 
     case 'd':Speed_need=0;Turn_need=0;positions=0;break;
     default:Speed_need=0;Turn_need=0;positions=0;break;}//stop
   
     //Speed_need=30;Turn_need=0;positions=80;  
     //SetMotorVoltage(255,255);
    //Kp=15,Kd=0.09,Ksp = 2.8,Ksi = 0.11;
         //Serial.print("count_left");Serial.println(count_left);
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Angle_Calcu();
    //Serial.print("Angle");Serial.println(Angle);
    PWM_Calcu();
  
   //if(millis()-old_time>=500){ Serial.print("count_right");Serial.print(count_right);Serial.print("count_left");Serial.println(count_left);old_time=millis();count_right=0;count_left=0;}     
}



/////////////////////////////////////////////////////////////////////////////////////////////
void Angle_Calcu(void)         {
        Angle_ax = (ax+1942)/238.2 ;   //去除零点偏移1942,//16384*3.14/1.2*180//弧度转换为度,
        Gyro_y = -(gy-119.58)/16.4;    //去除零点偏移119,2000deg/s 16.4 LSB/(deg/s)250---131 ，负号为方向处理 
        //Serial.print("Angle_ax,Angle_gy");Serial.print(Angle_ax);Serial.println(Angle_gy);
        Angle=0.97*(Angle+Gyro_y*0.0005)+0.03*Angle_ax;
        //Kalman_Filter(Angle_ax,Gyro_y);       //卡尔曼滤波计算倾角                                                                                                          
}   

void PWM_Calcu(void)
{
  if (abs(Angle)>40)
    {SetMotorVoltage(0,0);}//角度大于45度 停止PWM输出
    else
    { //Speed_need=30;Turn_need=0;positions=80;
      //////////////////////
      speeds=(count_left + count_right)*0.5;
      diff_speeds = count_left - count_right;
      diff_speeds_all += diff_speeds;
      speeds_filter *=0.85;  //一阶互补滤波
      speeds_filter +=speeds*0.15;
      positions += speeds_filter;
      positions += Speed_need;
      //positions = 0.85*speeds_filter + 0.15*speeds + positions + Speed_need;
      //positions = Speed_need + (positions + (0.15*speeds + (0.85*speeds_filter)))
      positions = constrain(positions, -2300, 2300);//抗积分饱和 
      ////////////////////
      Output=Kp*Angle+Kd*Gyro_y+ Ksp*speeds_filter + Ksi*positions ;
      //Serial.print("Output");Serial.println(Output);
      if(Turn_need==0){PWM_right=Output-diff_speeds_all;
      PWM_left=Output+diff_speeds_all;}
//貌似有毛病      
      PWM_right=Output+Turn_need;
      PWM_left=Output-Turn_need;
      
      if(PWM_right>=0){PWM_right+=PWM_right_least;}else{PWM_right-=PWM_right_least;}
      if(PWM_left>=0){PWM_left+=PWM_left_least;}else{PWM_left-=PWM_left_least;}
 
      SetMotorVoltage(PWM_left,PWM_right);}
       count_left = 0;
  count_right = 0;
}


