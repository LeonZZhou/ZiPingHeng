/*************************************************************************
	> File Name: motor.c
	> Author: 
	> Mail: 
	> Created Time: Wednesday 13 July 2016 03:46:05 PM CST
 ************************************************************************/

//int STBY = 13; //standby,当STBY=LOW,motor断电
int NC = A3;
//Motor A
int PWMA = 5; //Speed control 
int AIN1 = 8; //Direction
int AIN2 = 9; //Direction

//Motor B
int PWMB = 6; //Speed control
int BIN1 = 10; //Direction
int BIN2 = 11; //Direction
void setup(){
  pinMode(NC, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  PID_init();
}

void loop(){
  int count=0;
  while(count<=1000){
  float pwm_pid = PID_realize(127.0);
  count++;
  }
  move(1, pwm_pid, 1); //motor 1, left
  move(2, pwm_pid, 1); //motor 2, left
  //move(1, pwm_pid, 0); //motor 1, right
  //move(2, pwm_pid, 0); //motor 2, right

  delay(1000);
}


void move(int motor, int speed, int direction){
//Move specific motor at speed and direction
//motor: 0 for B 1 for A
//speed: 0 is off, and 255 is full speed
//direction: 0 clockwise, 1 counter-clockwise

  digitalWrite(NC, HIGH); //disable standby

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

void stop(){
//enable standby  
  digitalWrite(NC, LOW); 
}

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

