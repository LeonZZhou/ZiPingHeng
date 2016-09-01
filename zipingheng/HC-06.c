/* 本例程演示从手机蓝牙串口接收指令，点亮或熄灭11脚的LED，或者调整亮度。
 * 接收指令格式：“101X”点亮，“102X"熄灭，”000X"调整亮度。（000为0-100的整数，表示亮度值，X是结束符。）
 * 串口是异步接收的，用parseInt()查找第一个有效整数，检测到结束符后开始处理。
 * 接收数据显示在1602LCD屏上。
 * 点亮或熄灭LED后，将收到数据回传给手机。用pringln()函数，数据后面加了"\r"和"\n"两个字符，可以用作结束符。
 * 串口传送的是ASCII字符，数据格式可以自动转换。
 */

#include <Wire.h>                  // 包含LCD库。
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2);  // 初始化LCD对象，地址为0x27，每行16个字符，共两行。
int p;                             // 定义整数型全局变量，保存接收的数据。

void setup() {
  lcd.init();                      // 初始化LCD。
  lcd.backlight();                 // 打开LCD背光。
  Serial.begin(9600);              // 打开串口（蓝牙）
  pinMode(11,OUTPUT);              // 定义11脚为输出。（数字或PWM）
}

void loop() {
  while (Serial.available() > 0) {   // 串口收到字符数大于零。
    p = Serial.parseInt();           // 在串口数据流中查找一个有效整数。
    if (Serial.read() == 'X') {      // 收到结束符后开始处理数据。
      lcd.clear();                   // LCD清屏。
      lcd.print(p);                  // LCD显示收到的数据（整数）。
      switch (p) {                   // 判断数据内容。
      case 101:
        digitalWrite(11,HIGH);       // 点亮LED。
        Serial.println(p);           // 回传数据。
        break;
      case 102:
        digitalWrite(11,LOW);        // 熄灭LED。
        Serial.println(p);           // 回传数据。
        break;
      default:
        p = map(p,0,100,0,255);      // 转换数据范围到PWM输出值。
        analogWrite(11,p);           // 调整LED亮度。（PWM）
      }
    }
  }
}
