/*************************************************************************
	> File Name: Cmain.c
	> Author: 
	> Mail: 
	> Created Time: Wednesday 13 July 2016 03:06:12 PM CST
 ************************************************************************/

//定义变量程序段
//外部中断0
//数字端口2是Arduino的外部中断0的端口。
//数字端口3是Arduino的外部中断1的端口。
//外部中断1

#define PinA_left 3 //把小车左轮电机编码器码盘的OUTA信号连接到Arduino控制器的数字端口2，
#define PinB_left 10 //小车左车轮电机编码器码盘的OUTB信号连接到数字端口8 

#define PinA_right 4 //把小车右车轮电机编码器码盘的OUTA信号连接到Arduino控制器的数字端口3，
#define PinB_right 11 //小车右车轮电机编码器码盘的OUTB信号连接到数字端口9

int E_left =A0; //L298P直流电机驱动板的左轮电机使能端口连接到数字接口5
int M_left =6; //L298P直流电机驱动板的左轮电机转向端口连接到数字接口4
int M_left2=7;
int E_right =A1; //连接小车右轮电机的使能端口到数字接口6
int M_right =8; //连接小车右轮电机的转向端口到数字接口7
int M_right2=9;
int val_right; //小车右轮电机的PWM功率值

int val_start;//上位机控制字节，用于控制电机是否启动；
int val_FB;   //上位机控制字节，用于控制电机是正转还是反转；
int val_left;//上位机控制字节，用于提供给左轮电机PWM功率值。

int count1 = 0;  //左轮编码器码盘脉冲计数值
int count2= 0;   //右轮编码器码盘脉冲计数值
int rpm1 = 0;  //左轮电机每分钟(min)转速(r/min)
int rpm2 = 0;  //右轮电机每分钟(min)转速(r/min)

int rpm1_HIGH = 0;//左轮电机转速分解成高、低两个字节数据，以方便上传给PC机
int rpm1_LOW = 0;
int rpm2_HIGH = 0;//右轮电机转速分解成高、低两个字节数据
int rpm2_LOW = 0;

int flag;//设置小车行车状态，是前进、后退还是停止
unsigned long time = 0, old_time = 0; // 时间标记
unsigned long time1 = 0, time2 = 0; // 时间标记

//初始化程序段
void setup()
{
  Serial.begin(9600);    // 启动串口通信，波特率为9600b/s
  pinMode(M_left, OUTPUT);   //L298P直流电机驱动板的控制端口设置为输出模式
  pinMode(M_left2, OUTPUT); 
  pinMode(E_left, OUTPUT); 
  pinMode(M_right, OUTPUT); 
  pinMode(M_right2, OUTPUT); 
  pinMode(E_right, OUTPUT);
  pinMode(PinA_left,INPUT); //伺服电机编码器的OUTA和OUTB信号端设置为输入模式
  pinMode(PinB_left,INPUT);
  pinMode(PinA_right,INPUT); 
  pinMode(PinB_right,INPUT);
  //定义外部中断0和1的中断子程序Code(),中断触发为下跳沿触发
  //当编码器码盘的OUTA脉冲信号发生下跳沿中断时，
  //将自动调用执行中断子程序Code()。
  attachInterrupt(0, Code1, FALLING);//小车左车轮电机的编码器脉冲中断函数
  attachInterrupt(1, Code2, FALLING);//小车右车轮电机的编码器脉冲中断函数
}

//主程序段
void loop()
{
  if (Serial.available()>0) //如果Arduino控制器读缓冲区中存在上位机下达的字节
  {
      val_start= Serial.read(); //从读缓冲区中读取上位机的三个控制字节
      delay(5); 
      val_FB = Serial.read(); 
      delay(5);  
      val_left= Serial.read(); 
      delay(5);      
     if(val_start==0x11)     //如果读出的第一个字节为小车启动标志字节0x11
     {
       if(val_FB ==0xAA)   //如果读出的第二个字节为小车前进标志字节0xAA
       { 
         //读出的第三个字节为小车左车轮电机的PWM功率值，把它赋值给右车轮电机功率变量
         val_right=val_left; 
         advance(); //小车前进
         flag='a';   //设置小车前进标志字符     
         count1 = 0; //恢复到编码器测速的初始状态
         count2 = 0;
         old_time=  millis();    
       }
       else if(val_FB ==0xBB) //如果读出的第二个字节为小车后退标志字节0xBB    
       { 
         val_right=val_left;
         back();  //小车后退
         flag='b'; //设置小车后退标志字符       
         count1 = 0; //恢复到编码器测速的初始状态
         count2 = 0;
         old_time=  millis();        
       }
     }
      else if(val_start==0x22) //如果读出的第一个字节为小车停止标志字节0x22
      { 
         Stop(); //小车停止
         flag='s'; //设置小车停止标志字符         
      }
  }
  
  time = millis();//以毫秒为单位，计算当前时间 
  //计算出每一秒钟编码器码盘计得的脉冲数，
  if(abs(time - old_time) >= 1000) // 如果计时时间已达1秒
  {
    detachInterrupt(0); // 关闭外部中断0
    detachInterrupt(1); // 关闭外部中断1    
     //把每一秒钟编码器码盘计得的脉冲数，换算为当前转速值
     //转速单位是每分钟多少转，即r/min。这个编码器码盘为12个齿。
    rpm1 =(float)count1*60/12;//小车左车轮电机转速
    rpm2 =(float)count2*60/12; //小车右车轮电机转速
    rpm1_HIGH=rpm1/256;//把转速值分解为高字节和低字节
    rpm1_LOW=rpm1%256; 
    rpm2_HIGH=rpm2/256;
    rpm2_LOW=rpm2%256; 
   //根据左右车轮转速差rpm1-rpm2，乘以比例因子0.4，获得比例调节后的右车轮电机PWM功率值
    val_right=(float)val_right+(rpm1-rpm2)*0.4; 
   // val_right = map(val_right,100,25500,0,255);

    if(rpm1_HIGH !=0)
    {
       Serial.print("rpm1_HIGH:");
      Serial.println(rpm1_HIGH);//向上位计算机上传左车轮电机当前转速的高、低字节
    }
    if(rpm1_LOW !=0)
    {
       Serial.print("rpm1_LOW:");
      Serial.println(rpm1_LOW);
    }
    if(rpm2_HIGH !=0)
    {
       Serial.print("rpm2_HIGH:");
      Serial.println(rpm2_HIGH);//向上位计算机上传右车轮电机当前转速的高、低字节
    }
    if(rpm2_LOW !=0)
    {
       Serial.print("rpm2_LOW:");
       Serial.println(rpm2_LOW);
    }
     if(rpm1 !=0)
    {
       Serial.print("rpm1:");
       Serial.println(rpm1);
    }
     if(rpm2 !=0)
    {
       Serial.print("rpm2:");
       Serial.println(rpm2);
    }
    if(val_right !=0)
    {
       Serial.print("val_right:");
       Serial.println(val_right);// 向上位计算机上传PID调节后的右轮电机PWM功率值   
    }
    if(flag=='a') //根据刚刚调节后的小车电机PWM功率值，及时修正小车前进或者后退状态
    advance();
    if(flag=='b') 
    back();   
   //恢复到编码器测速的初始状态
    count1 = 0;   //把脉冲计数值清零，以便计算下一秒的脉冲计数
    count2 = 0; 
    old_time=  millis();     // 记录每秒测速时的时间节点   
    attachInterrupt(0, Code1,FALLING); // 重新开放外部中断0
    attachInterrupt(1, Code2,FALLING); // 重新开放外部中断1
  }
}

   //定义外部中断0和1的中断子程序Code(),中断触发为下降沿触发
   //当编码器码盘的OUTA脉冲信号发生下降沿中断时，
   //将自动调用执行中断子程序Code()。
// 左侧车轮电机的编码器码盘计数中断子程序
void Code1()
{  
  //为了不计入噪音干扰脉冲，
   //当2次中断之间的时间大于5ms时，计一次有效计数
  if((millis()-time1)>5) 
  //当编码器码盘的OUTA脉冲信号下跳沿每中断一次，
  count1 += 1; // 编码器码盘计数加一  
  time1==millis();
}
// 右侧车轮电机的编码器码盘计数中断子程序
void Code2()
{  
  if((millis()-time2)>5) 
  //当编码器码盘的OUTA脉冲信号下跳沿每中断一次，
  count2 += 1; // 编码器码盘计数加一
  time2==millis();  
}

//子程序程序段
void advance()//小车前进
{
     digitalWrite(M_left,HIGH);
     digitalWrite(M_left2,LOW);
     analogWrite(E_left,val_left);
     digitalWrite(M_right,LOW);
     digitalWrite(M_right2,HIGH);
     analogWrite(E_right,val_right);
}
void back()//小车后退
{
     digitalWrite(M_left,LOW);
     digitalWrite(M_left2,HIGH);
     analogWrite(E_left,val_left);
     digitalWrite(M_right,HIGH);
     digitalWrite(M_right2,LOW);
     analogWrite(E_right,val_right);
}
void Stop()//小车停止
{
     digitalWrite(E_right, LOW);
     digitalWrite(E_left, LOW); 
} 
