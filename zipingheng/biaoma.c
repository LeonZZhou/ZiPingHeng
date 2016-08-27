/************************************************************************************//**
 *  @file       biaoma.c
 *
 *  @brief      Brief descriptinon of biaoma.c 
 *
 *  @date       2016-07-21 17:02
 *
 ***************************************************************************************/
//数字端口2是Arduino外部中断0的端口
//数字端口3是Arduino外部中断1的端口
int L_bianma_A = 2;
int L_bianma_B = 10;
int R_bianma_A = 3;
int R_bianma_B = 11;
int count1 = 0; //左侧编码盘脉冲计数值
int count2 = 0; //右侧编码盘脉冲计数值
int rpm1 = 0; //左侧电机每分钟转速
int rpm2 = 0; //右侧电机每分钟转速

void setup(){
  Serial.begin(9600);
  pinMode();
//编码盘计数，中断触发为下降沿触发
  attachInterupt(0,bianma1,FALLING);
  attachInterupt(1,bianma2,FALLING);

}
//左侧编码盘计数中断程序
void bianma1(){
  if((millis()-time1)>5)
    count1 +=1 ;
  time1 = millis();
}
//右侧编码盘技术中断程序
void bianma2(){
  if((millis()-time2)>5)
    count2 +=1 ;
  time2 = millis();
}

