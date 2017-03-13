#include "ioCC2540.h"
#include "oled.h"
//#include "bmp.h"  引用后产生错误冲突，目前暂未查出错误的根源。
#include "PWM.h"
#include "PID.h"
#include <stdio.h>
#include <string.h>
#include "ds18b20.h"  
#include <math.h> 
//#include<intrins.h> ,此部分为调用汇编相关命令时的函数，百度可知

struct PID 
{ 
unsigned int SetPoint; // 设定目标 Desired Value 
unsigned int Proportion; // 比例常数 Proportional Const 
unsigned int Integral; // 积分常数 Integral Const 
unsigned int Derivative; // 微分常数 Derivative Const 
unsigned int LastError; // Error[-1] 
unsigned int PrevError; // Error[-2] 
unsigned int SumError; // Sums of Errors  
};
struct PID spid; // PID Control Structure 
unsigned int rout; // PID Response (Output) 
unsigned int rin; // PID Feedback (Input) 
unsigned char flag,flag_1=0; 
unsigned char high_time,low_time,count=0;//占空比调节参数 
unsigned char set_temper=35; 
unsigned char temper; 
unsigned char i; 
unsigned char j=0; 
unsigned int s;  //暂时用不上
float pilot_process=0.0;


extern void PIDInit (struct PID *pp); 
extern unsigned int PIDCalc( struct PID *pp, unsigned int NextPoint ) ;
extern void Pid_main(void); 

/*********************************************************** 
获取温度子程序 
***********************************************************/ 
void get_temper() 
{ 
temper=(char)DS18B20_ReadMain(); /*获取的温度放在temper中*/ 
} 
/*==================================================================================================== 
Initialize PID Structure 初始化PID结构体
=====================================================================================================*/ 
void PIDInit (struct PID *pp) 
{ 
high_time=50; 
low_time=50; 
//PIDInit ( &spid ); // Initialize Structure 
spid.Proportion = 10; // Set PID Coefficients 
spid.Integral = 8; 
spid.Derivative =6; 
spid.SetPoint = 100; // Set PID Setpoint 
memset ( pp,0,sizeof(struct PID)); 

} 
/*==================================================================================================== 
PID计算部分 
=====================================================================================================*/ 
unsigned int PIDCalc( struct PID *pp, unsigned int NextPoint ) 
{ 
unsigned int dError,Error; 
Error = pp->SetPoint - NextPoint; // 偏差 
pp->SumError += Error; // 积分 
dError = pp->LastError - pp->PrevError; // 当前微分 
pp->PrevError = pp->LastError; 
pp->LastError = Error; 
return (pp->Proportion * Error // 比例项 
+ pp->Integral * pp->SumError // 积分项 
+ pp->Derivative * dError); // 微分项 
} 

unsigned int sensor (void)        // 虚拟传感器功能   Dummy Sensor Function
{    return 100;}
/*********************************************************** 
温度比较处理子程序 
***********************************************************/ 
void compare_temper() 
{ 
unsigned char i; 
if(set_temper>temper)   //如果设定温度大于实际温度
{ 
if(set_temper-temper>1)   //如果设定温度大于实际温度1度以上
{ 
high_time=100;      //全部高电平
low_time=0; 
} 
else          
{ 
for(i=0;i<10;i++)        //否则分十次循环检测温度
{ 
get_temper(); 
rin = sensor(); // Read Input 
rout = PIDCalc ( &spid,rin ); // Perform PID Interation 
} 
if (high_time<=100) 
high_time=(unsigned char)(rout/800); 
else 
high_time=100; 
low_time= (100-high_time); 
} 
} 
else if(set_temper<=temper) 
{ 
if(temper-set_temper>0) 
{ 
high_time=0; 
low_time=100; 
} 
else 
{ 
for(i=0;i<10;i++) 
{ 
get_temper(); 
rin = sensor(); // Read Input 
rout = PIDCalc ( &spid,rin ); // Perform PID Interation 
} 
if (high_time<100) 
high_time=(unsigned char)(rout/10000); 
else 
high_time=0; 
low_time= (100-high_time); 
} 
} 
} 

void Pid_main(void) 
{ 
//unsigned char a,b,flag_2=1; 

//while(1)     //主程序循环内容如下
//{ 
get_temper();   //获取当前温度
//b=temper;       //当前温度赋值给b
//if(flag_2==1)   //第一次时把当前温度值给a
//a=b; 
//if((fabs(a-b))>5)   //比较当前温度和第一次温度的差值，如果产生了5度偏差，修正A
//temper=a; 
//else temper=b; 
//a=temper; 
// flag_2=0;  
compare_temper(); 
pilot_process=high_time*2.55;
high_time=(unsigned char)pilot_process;
//}
}