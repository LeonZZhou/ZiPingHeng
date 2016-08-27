//原始6050数据+双测速码盘
#include "Wire.h" //串口
#include "I2Cdev.h" //IIC总线
#include "MPU6050.h" //加速度与陀螺传感器

//自定义变量
char val='z'; //调节与控制命令字
int Speed_need = 0, Turn_need = 0; //运动速度，转弯速度
float speeds, speeds_filter, positions,Q; //速度，速度滤波，位置
float diff_speeds, dspeeds = 0, dspeeds_all = 0; //速度差
int text_time = 0, spcount = 0, dspcount = 0; //测试时间，速度测量次数

//PID参数
double Output = 0; //PID输出
float Kp=12, Kd=0.11, Ksp = 0.10 ,Ksi = 0.05, Kdsp = 5; //PID角度环、速度环系数
// MPU6050参数
MPU6050 accelgyro;
int16_t ax, ay, az, gx, gy, gz;
//角度参数
float Gyro_y; //Y轴陀螺仪数据暂存
float Angle_ax; //由加速度计算的倾斜角度
float Angle; //一阶互补滤波计算出的小车最终倾斜角度
float Angle0 = -3.00; //机械平衡角

//引脚分配
/*
int PinA_right= 2; //interrupt 0 
int PinA_left= 3; //interrupt 1
int E_left =5;//PWMA
int M_left =4; 
int M_right =7; 
int E_right =6; //PWMB
*/
int PinA_left = 2; //中断0 
int PinA_right = 3; //中断1
int M_left = 8;  
int M_left2 = 7; 
int E_left = 5; //ENA
int M_right = 10; 
int M_right2= 11;
int E_right = 6; //ENB*/
//电机输出
int PWM_right = 0, PWM_left = 0;
int PWM_left_least = 40, PWM_right_least = 40; //左右电机启动补偿50
//测速码盘中断
int count_right = 0;
int count_left = 0;

void Code_left() {
  if(Output>=0) {
    count_left ++;
  }else{
    count_left --;
  }
} //左测速码盘计数
void Code_right() {
  if(Output>=0) {
    count_right ++;
  }else {
    count_right --;
  }
} //右测速码盘计数

//电机输出
/*
void SetMotorVoltage(int nLeftVol, int nRightVol) {
    if(nLeftVol >=0) 
  {  digitalWrite(M_left,HIGH);
    
   } else {       
     digitalWrite(M_left,LOW);
      nLeftVol=-nLeftVol;
   }
    if(nRightVol >= 0) {
     digitalWrite(M_right,HIGH);
    
    } else {
     digitalWrite(M_right,LOW);
  
    nRightVol=-nRightVol;
  }
    if(nLeftVol>255) { nLeftVol = 255 ; }   //防止PWM值超过255
    if(nRightVol>255) { nRightVol = 255 ; }//防止PWM值超过255
    analogWrite(E_left,nLeftVol);
    analogWrite(E_right,nRightVol);
}
*/
void SetMotorVoltage(int nLeftVol, int nRightVol) {
  if(nLeftVol >=0) {
    digitalWrite(M_left, LOW);
    digitalWrite(M_left2, HIGH);
  }else {
    digitalWrite(M_left, HIGH);
    digitalWrite(M_left2, LOW);
    nLeftVol = -nLeftVol;
  }
  if(nRightVol >= 0) {
    digitalWrite(M_right, LOW);
    digitalWrite(M_right2, HIGH);
  }else {
    digitalWrite(M_right, HIGH);
    digitalWrite(M_right2, LOW);
    nRightVol = -nRightVol;
  }
  if(nLeftVol > 255) nLeftVol = 255; //防止PWM值超过255
  if(nRightVol > 255) nRightVol = 255; //防止PWM值超过255
  analogWrite(E_left, nLeftVol);
  analogWrite(E_right, nRightVol);
}

//计算小车角度
void Angle_Calcu(void) {
  Angle_ax = (ax+200)/200 ; //去除零点偏移1942，16384*3.14/1.2*180//弧度转换为度,
  Gyro_y = -(gy - 10.58)/16.4; //去除零点偏移119，2000deg/s 16.4 LSB/(deg/s)250---131 
  Angle = 0.97 * (Angle + Gyro_y * 0.0005) + 0.03 * Angle_ax;
} 

void setup() {
  Wire.begin();
  Serial.begin(9600); 
  accelgyro.initialize(); //初始化设备
  //引脚模式设置
   pinMode(E_right, OUTPUT); pinMode(M_right, OUTPUT);
   pinMode(E_left, OUTPUT);  pinMode(M_left, OUTPUT);
   pinMode(PinA_right,INPUT);pinMode(PinA_left,INPUT);//
   
  /*pinMode(E_left, OUTPUT);
  pinMode(M_left, OUTPUT);
  pinMode(M_left2, OUTPUT); //左电机
  pinMode(E_right, OUTPUT); 
  pinMode(M_right, OUTPUT);
  pinMode(M_right2, OUTPUT); //右电机
  pinMode(PinA_right, INPUT);
  pinMode(PinA_left, INPUT); //测速码盘输入*/
  //中断设置
  attachInterrupt(0, Code_right, RISING);
  attachInterrupt(1, Code_left, RISING);
}

void loop() {
  if (Serial.available() > 0) {
    val = Serial.read();
   //参数调节
    if(val == 'X') Kp += 0.1;
    if(val == 'x') Kp -= 0.1;
    if(val == 'V') Kd += 0.01;
    if(val == 'v') Kd -= 0.01;
    if(val == 'N') Ksp += 0.1;
    if(val == 'n') Ksp -= 0.1;
    if(val == 'M') Ksi += 0.01;
    if(val == 'm') Ksi -= 0.01;
    if(val == 'L') Angle0 += 0.1;
    if(val == 'l') Angle0 -= 0.1;    
    if(val == 'P') PWM_left_least += 1;
    if(val == 'p') PWM_left_least -= 1; 
    if(val == 'G') PWM_right_least += 1;
    if(val == 'g') PWM_right_least -= 1;
    //参数查看
    if(val == 'H') {
      Serial.print(ax); Serial.print("\t"); //用于测量零点偏移
      Serial.println(gy); //用于测量零点偏移
    }
    if(val == 'I') {
      Serial.print(Angle0); Serial.print("\t");
      Serial.print(PWM_left_least); Serial.print("\t");
      Serial.print(PWM_right_least); Serial.print("\t");
      Serial.print(Kp); Serial.print("\t");
      Serial.print(Kd); Serial.print("\t");
      Serial.print(Ksp); Serial.print("\t");
      Serial.print(Ksi); Serial.print("\t");
       Serial.print(Q); Serial.print("\t");
      Serial.println(Kdsp);
    }
    if(val == 'J') {
      Serial.print(Angle); Serial.print("\t");
      Serial.println(dspeeds_all);
    }
    if(val == 'K') Kdsp += 1;
    if(val == 'k') Kdsp -= 1;    
    //小车控制
    if(val == 'A') {
      Speed_need = 900; //前进
      Turn_need = 0;
    }
    if(val == 'B') {
      Speed_need = -900;
      00; //后退
      Turn_need = 0;
    }
    if(val == 'C') {
      Speed_need = 0;
      Turn_need = 20; //左转
    }
    if(val == 'D') {
      Speed_need = 0;
      Turn_need = -20; //右转
    }
    if(val == 'E') 
    {
      Speed_need = 0; //停止
      Turn_need = 0;
    }
    
        if(val == 'F') 
    {
      Speed_need = 0; //停止
      Turn_need = 0;
    }
  }

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //获取传感器原始值
  Angle_Calcu(); //计算角度
  PWM_Calcu(); //PWM输出计算
  spcount ++;
  if(spcount > 40)
  {
    spcount = 0;
    dspcount ++;
    //速度与速度积分
    speeds = (count_left + count_right)*0.5;
    diff_speeds = count_left - count_right;
    dspeeds += diff_speeds;
    if(dspcount > 80) {
      dspeeds_all = dspeeds;
      dspeeds = 0;
    }
    speeds_filter *= 0.85; //一阶互补滤波
    speeds_filter += speeds*0.15;
    positions += speeds_filter;
    positions = constrain(positions, -300, 300); //抗积分饱和
    count_left = 0;
    count_right = 0;
  }
  Serial.print(count_left);Serial.print(count_right);Serial.println();
}

void PWM_Calcu(void) 
{
  if (abs(Angle) > 45)
  {
    SetMotorVoltage(0,0); //角度大于30度 停止PWM输出
  }
  else 
  {
    //PWM输出计算
    Q= Ksi*positions;
    Output = Kp*(Angle - Angle0) + Kd*Gyro_y + Ksp*(speeds_filter -Speed_need) + Q;
    if(Turn_need == 0)
    {
      PWM_left = Output - Kdsp * dspeeds_all;
      PWM_right = Output + Kdsp * dspeeds_all;
    }
    
    PWM_left = Output - Turn_need;
    PWM_right = Output + Turn_need;
     
    if(PWM_left >= 0) 
    {
      PWM_left += PWM_left_least;
    }
    else 
    {
      PWM_left -= PWM_left_least;
    }
    if(PWM_right >= 0)
    {
      PWM_right += PWM_right_least;
    }
    else
    {
      PWM_right -= PWM_right_least;
    }
    SetMotorVoltage(PWM_left, PWM_right);
  }
}

