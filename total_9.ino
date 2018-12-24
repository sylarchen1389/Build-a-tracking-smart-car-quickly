#include <Servo.h>
#include <SoftwareSerial.h>
// #include <TimerOne.h> //申明库文件


// #define S0    22   //物体表面的反射光越强，TCS3002D内置振荡器产生的方波频率越高，
// #define S1    24  //S0和S1的组合决定输出信号频率比例因子，比例因子为2%
//                  //比率因子为TCS3200传感器OUT引脚输出信号频率与其内置振荡器频率之比
// #define S2     26   //S2和S3的组合决定让红、绿、蓝，哪种光线通过滤波器
// #define S3     28
// #define OUT    18  //TCS3200颜色传感器输出信号连接到Arduino中断0引脚，并引发脉冲信号中断
//                   //在中断函数中记录TCS3200输出信号的脉冲个数
// #define LED    30  //控制TCS3200颜色传感器是否点亮LED灯
// float g_SF[2];     //从TCS3200输出信号的脉冲数转换为RGB标准值的RGB比例因子
// int   g_count = 0;  // 计算与反射光强相对应TCS3200颜色传感器输出信号的脉冲数
// int   g_array[2];
// int   g_flag = 0;   // 滤波器模式选择顺序标志


int now_state=0;
char statesl[][6]={"setup","run","brake","back","left","right","s_lef","s_rig"};
//蓝牙
SoftwareSerial BT(52,53);


//定义舵机引脚
Servo lowerservo;
Servo middleservo;
Servo higherservo;
Servo clawservo;
//舵机位置记录
int lowerservo_value;
int middleservo_value;
int higherservo_value = 90;
int clawservo_value;


//定义电机引脚
#define leftmotor_pwm   9
#define rightmotor_pwm  10
int leftmotor_go=35;
int leftmotor_back=36;
int rightmotor_go=37;
int rightmotor_back=38;

//定义光电对管引脚 从左开始数
int finding_pin4 = 39;     //x4
int finding_pin3 = 40;     //x3
int finding_pin2 = 42;    //x2
int finding_pin1 = 41;    //x1


//定义光电对管读取值
int finding_value2;
int finding_value1;
int finding_value3;
int finding_value4;

//电机函数
void motor_Init();
void run(int left_speed, int right_speed);
void brake();       //刹车
void back();        //后退
void left(int left_speed, int right_speed);
void right(int left_speed, int right_speed);
void spin_left(int left_speed, int right_speed);
void spin_right(int left_speed, int right_speed);
void sendstate();

//寻迹函数
void judge();
void finding_Init();

//颜色传感器函数
// void TSC_Init();
// void TSC_WB(int Level0, int Level1);
// void TSC_Callback();

//机械臂函数
void servo_Init2();//初始化位置
void Grab();//抓取
void Ready();//准备放球
void HoldUp();//竖直
void Put();//放球
void claw_close();
void Fuck();
// void rise();//上升
// void decline();//下降


int flag=0;
// int lflag=0;
// int lastflag=0;
char data[10]="";

void setup(){
    BT.begin(9600);
    Serial.begin(9600);
    now_state=0;

    // TSC_Init();
    motor_Init();
    servo_Init();
    finding_Init();
    servo_Init2();
}

int extraspeed=0;
void loop(){

    int span;
    span=0;
    if(flag&&flag<5){
        judge();
    }else if(flag==5){
      switch(data[1]){
        case '0':run(255-extraspeed, 255-extraspeed);break;
        case '1':back();break;
        case '2':spin_left(150,150);break;
        case '3':spin_right(150,150);break;
        default:;
        }
    }else if(flag==6){
        switch(data[1]){
          case 'g':Grab();break;
          case 'r':Ready();break;
          case 'p':Put();break;
          case 'i':servo_Init2();break;
          case 'd':HoldUp();break;
          case 'f':Fuck();break;
          default:;
        }
        flag=0;
    }else{
        brake();
    }
//BTRead:
    int v=0;
    char vs[10]="";
    while(BT.available()){
        vs[v++]=BT.read();
        delay(50);
    }
//Main BT control,data will remain till next successful read.
    if(vs[0]!=0){
     memcpy(data,vs,10);
     memcpy(vs,0,10);
     Serial.println(data);
     BT.print(data);
     BT.println("<--R");
     flag=data[0]-'0';
     if(flag==4){
     extraspeed=(data[1]-'0')*100+(data[2]-'0')*10+data[3]-'0';
     if(data[4]=='-')
        extraspeed=0-extraspeed;
     }
    }
}


/*寻迹函数*/
void judge(){
    //检测到黑线时循迹模块相应的指示灯亮，端口电平为LOW
    //未检测到黑线时循迹模块相应的指示灯灭，端口电平为HIGH
    finding_value2  = digitalRead(finding_pin1);
    finding_value3  = digitalRead(finding_pin2);
    finding_value1  = digitalRead(finding_pin3);
    finding_value4  = digitalRead(finding_pin4);

    //Serial.println("---光电对管数据始----");
    //Serial.print(finding_value1);
    //Serial.print(finding_value2);
    //Serial.print(finding_value3);
    //Serial.print(finding_value4);
    //Serial.println("---光电对管数据终----");

    //delay(10000);

    //四路循迹引脚电平状态
    // 0 0 X 0
    // 1 0 X 0
    // 0 1 X 0
    //以上6种电平状态时小车原地右转，速度为250,延时80ms
    //处理右锐角和右直角的转动

    if ( (finding_value1 == HIGH || finding_value2 == LOW) &&  finding_value4 == HIGH)
    {
      //brake();
      spin_right(250-extraspeed,250-extraspeed);
      //lastflag=lflag;
      //lflag=1;

      delay(300);
    }
    //四路循迹引脚电平状态
    // 0 X 0 0
    // 0 X 0 1
    // 0 X 1 0
    //处理左锐角和左直角的转动
    else if ( finding_value1 == HIGH && (finding_value3 == LOW ||  finding_value4 == HIGH))
    {
      //brake();
      spin_left(250-extraspeed, 250-extraspeed);
      //lastflag=lflag;
      //lflag=1;
      delay(300);
    }
    // 0 X X X
    //最左边检测到
    else if ( finding_value1 == HIGH)
    {
      spin_left(150-extraspeed, 150-extraspeed);
     //delay(2);
    }
    // X X X 0
    //最右边检测到
    else if ( finding_value4 == HIGH )
    {
      spin_right(150-extraspeed, 150-extraspeed);
      //delay(2);
    }
    //四路循迹引脚电平状态
    // X 0 1 X
    //处理左小弯
    else if ( finding_value2 == LOW && finding_value3 == HIGH)
    {
      left(0, 220-extraspeed);
    }
    //四路循迹引脚电平状态
    // X 1 0 X
    //处理右小弯
    else if ( finding_value2 == HIGH && finding_value3 == LOW)
    {
      right(220-extraspeed, 0);
    }
    //四路循迹引脚电平状态
    // X 0 0 X
    //处理直线
    else if (finding_value2 == LOW && finding_value3 == LOW)
    {
      run(255-extraspeed, 255-extraspeed);
    }
//   else if (  finding_value4 == HIGH&&finding_value1 == LOW && finding_value3 == LOW && finding_value4 == LOW)//
  //  {
    //  back(50);
//      delay(50);
   //Sss }
//    当为1 1 1 1时小车保持上一个小车运行状态

}
void finding_Init(){
    //定义四路循迹红外传感器为输入接口
    pinMode(finding_pin4, INPUT);
    pinMode(finding_pin3, INPUT);
    pinMode(finding_pin2, INPUT);
    pinMode(finding_pin1, INPUT);

    //四路循迹红外传感器初始化为高电平
    digitalWrite(finding_pin4, HIGH);
    digitalWrite(finding_pin3, HIGH);
    digitalWrite(finding_pin2, HIGH);
    digitalWrite(finding_pin1, HIGH);
}

/*电机函数*/
void motor_Init(){
    //设置电机引脚模式
    pinMode(leftmotor_go,OUTPUT);
    pinMode(leftmotor_back,OUTPUT);
    pinMode(rightmotor_go,OUTPUT);
    pinMode(rightmotor_back,OUTPUT);
}

void brake(){
  now_state=3;
  sendstate();
  digitalWrite(leftmotor_go, LOW);
  digitalWrite(leftmotor_back, LOW);
  digitalWrite(rightmotor_go, LOW);
  digitalWrite(rightmotor_back, LOW);
  delay(100);
//  delay(time * 100);
}

void left(int left_speed, int right_speed){
  now_state=4;
  sendstate();
  //左电机停止
  digitalWrite(leftmotor_go, LOW);    //左电机前进禁止
  digitalWrite(leftmotor_back, LOW);  //左电机后退禁止
  analogWrite(leftmotor_pwm, left_speed);

  //右电机前进
  digitalWrite(rightmotor_go, HIGH);  //右电机前进使能
  digitalWrite(rightmotor_back, LOW); //右电机后退禁止
  analogWrite(rightmotor_pwm, right_speed);
}

void right(int left_speed, int right_speed){
  now_state=5;
  sendstate();
  //左电机前进
  digitalWrite(leftmotor_go, HIGH);   //左电机前进使能
  digitalWrite(leftmotor_back, LOW);  //左电机后退禁止
  analogWrite(leftmotor_pwm, left_speed);

  //右电机停止
  digitalWrite(rightmotor_go, LOW);   //右电机前进禁止
  digitalWrite(rightmotor_back, LOW); //右电机后退禁止
  analogWrite(rightmotor_pwm, right_speed);
}

void spin_left(int left_speed, int right_speed){
  now_state=6;
  sendstate();
  //左电机后退
  digitalWrite(leftmotor_go, LOW);     //左电机前进禁止
  digitalWrite(leftmotor_back, HIGH);  //左电机后退使能
  analogWrite(leftmotor_pwm, left_speed);

  //右电机前进
  digitalWrite(rightmotor_go, HIGH);  //右电机前进使能
  digitalWrite(rightmotor_back, LOW); //右电机后退禁止
  analogWrite(rightmotor_pwm, right_speed);
}

void spin_right(int left_speed, int right_speed){
  now_state=7;
  sendstate();
  //左电机前进
  digitalWrite(leftmotor_go, HIGH);    //左电机前进使能
  digitalWrite(leftmotor_back, LOW);   //左电机后退禁止
  analogWrite(leftmotor_pwm, left_speed);

  //右电机后退
  digitalWrite(rightmotor_go, LOW);    //右电机前进禁止
  digitalWrite(rightmotor_back, HIGH); //右电机后退使能
  analogWrite(rightmotor_pwm, right_speed);
}

void run(int left_speed, int right_speed){
  now_state=1;
  sendstate();
  //左电机前进
  digitalWrite(leftmotor_go, HIGH);   //左电机前进使能
  digitalWrite(leftmotor_back, LOW);  //左电机后退禁止
  analogWrite(leftmotor_pwm, left_speed );

  //右电机前进
  digitalWrite(rightmotor_go, HIGH);  //右电机前进使能
  digitalWrite(rightmotor_back, LOW); //右电机后退禁止
  analogWrite(rightmotor_pwm, right_speed);



}

void back(){
  now_state=2;
  sendstate();
  //左电机后退
  digitalWrite(leftmotor_go, LOW);     //左电机前进禁止
  digitalWrite(leftmotor_back, HIGH);  //左电机后退使能
  analogWrite(leftmotor_pwm, 250);

  //右电机后退
  digitalWrite(rightmotor_go, LOW);    //右电机前进禁止
  digitalWrite(rightmotor_back, HIGH); //右电机后退使能
  analogWrite(rightmotor_pwm,250);
}


/*舵机函数*/
void control(int pos_low, int pos_middle,int pos_claw){
    lowerservo.write(pos_low);
    middleservo.write(pos_middle);
    // higherservo.write(pos_higher);
    clawservo.write(pos_claw);
    lowerservo_value=pos_low;
    middleservo_value=pos_middle;
    // higherservo_value=pos_claw;
    clawservo_value=pos_claw;

}
//初始化
void servo_Init(){
     // 将引脚与舵机与声明的舵机对象连接起来
         lowerservo.attach(3);
         middleservo.attach(4);
         higherservo.attach(2);
         clawservo.attach(11);
 }
void servo_Init2(){
    control(155,30,180);
    higherservo.write(90);
}
//抓取~
void Grab(){
    //下降并打开
    clawservo.write(140);
    clawservo_value=140;
    delay(1000);
    control(90,90,140);
    delay(1000);
    control(1,140,140);
    delay(1000);
    claw_close();
    delay(1000);
    HoldUp();
    delay(2000);
    servo_Init2();
}
//竖起~
void HoldUp(){
    control(90,30,180);
    delay(1000);
    control(90,150,180);
}
//到达放球位置
void Ready(){
    for(int i=0;i<5;i++){
          control(155-10*i,30+i*18,180);
          delay(100);
      }
      delay(1000);
}
//放球
void Put(){
    clawservo.write(168);
    clawservo_value=168;
    delay(1000);
    clawservo.write(180);
    clawservo_value=180;
}
//上升
// void rise(){
//
// }
// //下降
// void decline(){}
// void middle_rise(){
//     middleservo.write(middleservo_value+5);
//     middleservo_value=+5;
// }
// void middle_decline(){
//     middleservo.write(middleservo_value-5);
//     middleservo_value=-5;
// }
// void lower_rise(){
//     lowerservo.write(lowerservo_value+5);
//     lowerservo_value=+5;
// }
// void lower_decline(){
//     lowerservo.write(lowerservo_value-5);
//     lowerservo_value=-5;
// }
// void claw_open(){
//     clawservo.write(0);
//     clawservo_value=0;
// }
 void claw_close(){
   control(0,160,180);
     //clawservo.write(180);
    // clawservo_value=180;
 }
// void claw_sopen(){
//     clawservo.write(170);
//     clawservo_value=170;
// }
void Fuck(){
  middleservo.write(140);
  delay(500);
  control(1,140,165);
  delay(500);
  }


 void sendstate(){
   if(flag>=2&&flag<4){
   BT.print("Now in-->");
   BT.println(statesl[now_state]);
   }
   if(flag>=3&&flag<4){
   BT.print(!finding_value1);
   BT.print(finding_value2);
   BT.print(finding_value3);
   BT.println(!finding_value4);
   }
   }
