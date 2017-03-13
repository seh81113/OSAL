#ifndef __OLED_MAIN_H
#define __OLED_MAIN_H

#include "ioCC2540.h"
#include "oled.h"
#include "bmp.h"
#include "PWM.h"
#include <stdio.h>
#include <string.h>
#include "ds18b20.h"  
#include "PID.H"


#define LED1 P1_1           // 定义P1.1口为LED3是三色灯其中红
#define LED2 P1_2           // 定义P1.2口为LED4是三色灯其中绿
#define LED3 P1_3           // 定义P1.3口为LED5是三色灯其中蓝
#define KEY_POWER P1_5      // 定义P1.5口为S1控制端,开关机按键
#define KEY_ADD P1_4        // 定义P1.4口为S2控制端，按键+
#define KEY_DEC P0_6        // 定义P1.4口为S2控制端，按键-
#define MOTOR  P0_7         // 定义P0.7口为马达运行端口
#define PEAK  P0_0          // 定义P0.0口为喇叭端口，暂时保留
#define Ultrasonic_atomizer_bak  P0_4          // 定义P0.4口为超声雾化备用端口，可不用
#define Ultrasonic_atomizer  P0_5              // 定义P0.5口为超声雾化控制，PWM  115KHZ
#define battery_control  P1_6                  // 定义P1.6口为充电控制端口
#define battery_state  P1_7                    // 定义P1.7口为充电为充电是否饱和的状态端口
#define HOT   P1_0          //定义P1.0口为加热控制端口，使用PWM调制的方式来控制
#define DS18B20   P2_0      //定义P2.0口为温度采集通讯端
#define low_temperature 2   //设置低温线，温度到达时停止加热，但可能温度继续升高，当温度低于设置温度一定值时，加热开启
#define Offset_temperature 10   //温度偏移值，因实际温度与外皮温度存在偏差，所以给出一个校正，目前暂用纯数字校正，后续用区间和查表的方式校准
#define ON      1
#define OFF     0
#define H_time  20000        //加热时间   基数是1毫秒
#define U_time  700          //超声时间
#define PnLi    0x01         //设置为P01口
#define PnUsb   0x04         //设置为P04口
#define PnLiPower4_4     1400        //满电的4/4      1400时是3.6V
#define PnLiPower4_3     1300        //满电的4/3
#define PnLiPower4_2     1200        //满电的4/2
#define PnLiPower4_1     1100        //满电的4/1
#define PnLiPower4_0     950        //满电的4/0       950时是2.5V   1050时是2.75V  760时2V
#define PnUsbPower4_4     1600        //USB供电，电压大于4.6V为USB供电状态,4.6V大约是1800,  1670为4.4V ,加热时会引起电压波动，所以这个值要设置低一些，避免波动造成误判断。
#define HAL_ADC_REF_1V25                0x00
#define ADC_REF_AVDD5                   0x80
#define ADC_7_BIT                       0x0
#define ADC_9_BIT                       0x10
#define ADC_10_BIT                      0x20
#define ADC_12_BIT                      0x30
#define ADC_EMP_SENS                    0x0E
#define ADC_TO_CELSIUS(ADC_VALUE)       ((ADC_VALUE>>4)-334)   //温度校正
#define TEMP_CMP_HIGH                   70    //原本120度，现在调整了位置最高给定70度。
#define TEMP_CMP_TipBurn                65     //超过此温度，提示小心烫伤
#define TEMP_CMP_TipBurn_count          20     //超过此温度每隔20个基数,提示小心烫伤
#define TEMP_CMP_LOW                    37
#define AD_AbnormalTermination   if(j>2048)j=0    //解决无电压采样异常
typedef unsigned char uchar;
typedef unsigned int  uint;
unsigned char strTemp[2];
char buf[6];
char ucTemp=0,ucTemp_on=0,high_temperature_count=0;
char PowerLow=0;  //LI电池电压低标志位，若为1则电压低，否则LI电池可以工作。
char UsbPowerElectricize=0; //USB电源充电状态标志
uint PwmVariable=10;//PWM调节加热变量
char LoopClearOver=0;       //USB充电循环图标是否执行过，如果断开过电压就要重新执行，否则不执行
uint BattleCycleCount=0;  //电池小图标循环计算计数
extern  char temp=0;  //串行0中断时临时使用
extern  uchar KEY_POWER_MARK=0,KEY_ADD_MARK=0,KEY_DEC_MARK=0,KEY_SET_MARK=0;   //按键的状态标志位，为1代表曾按下过
extern  uchar KEY_ADD_DEC_MARK=0;
extern  uchar TEMP_CMP=60;  //设置起始温度50度
extern  uchar KeyScan_finsh=0; //当两个按键同时按下后，处理完成后给出一个标志
extern  uchar Battle_6432Finish=0;  //电池充满电后显示和BI是否执行的标志位
extern  uint test1=0;  
extern  uint test2=0;  
extern  uchar RoomTemperatureValue=25; //定义默认室温25度
extern  uint ErrorCount=0;


#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr)[0])
#define FLOAT_TEMP      0          //输出更高精度时打开此注释
extern void Delay_ms(unsigned int k);//外部函数ms的声明
//extern void delay_nus(unsigned short timeout);
extern void Turn_on_picture(void);
extern void Turn_off_picture(void);
extern void Turnon_main_picture(void);
extern void Turnon_set_picture(void);
extern void PowerOffCharging(void);
extern void PWM_Hot(void);
extern void ShutDown(void);
extern void RoomTemperature(void);
extern void Surface_temperature_conversion(void);
extern uint VoltageMean(unsigned char Pnn);
extern void Charge_complete(void);
extern u8 LoopCheck(void);
extern void Error(void);
#endif