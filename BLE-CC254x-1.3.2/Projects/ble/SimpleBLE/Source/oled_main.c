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

/****************************************************************************
* 名    称: 
* 功    能: 定时器延时函数
* 入口参数: 
* 出口参数: 无
****************************************************************************/

void Delay_us(unsigned int k)//us延时函数
{
    T1CC0L = 0x06; 
    T1CC0H = 0x00; 
    T1CTL = 0x02; 
    while(k)
    { 
        while(!(T1CNTL >= 0x04));
        k--;
    }
    T1CTL = 0x00;  //关闭定时器
}

void Delay_ms(unsigned int k)
{
    T1CC0L = 0xe8;
    T1CC0H = 0x03;
    T1CTL = 0x0a; //模模式 32分频
    while(k)
    {
        while(!((T1CNTL >= 0xe8)&&(T1CNTH >= 0x03)));
        k--;
    }
    T1CTL = 0x00; //关闭定时器
}

void Delay_s(unsigned int k)
{
    while(k)
    {
        Delay_ms(1000);
        k--;
    }
}
/****************************************************************************
* 名    称: LedOnOrOff()
* 功    能: 点亮或熄灭所有LED灯 或者端口   
* 入口参数: mode为1时LED灯亮  mode为0时LED灯灭
* 出口参数: 无
****************************************************************************/
void LedOnOrOff(uchar mode)
{
    LED1 = mode;
    LED2 = mode;
    LED3 = mode; 
}

/****************************************************************************
* 名    称: InitLed()
* 功    能: 设置LED相应的IO口
* 入口参数: 无
* 出口参数: 无
****************************************************************************/
void InitLed(void)
{
    P1DIR |= 0x0f;  // P1.0、P1.1、P1.2、P1.3定义为输出   P1DIR端口1方向寄存器
    P0DIR |= 0xC4;  // P0.2、P0.6、P0.7定义为输出   P0DIR端口1方向寄存器
    LedOnOrOff(0);  // 使所有LED灯默认为熄灭状态  
}

/****************************************************************************
* 名    称: InitIO()
* 功    能: 设置按键相应的IO口
* 入口参数: 无
* 出口参数: 无
****************************************************************************/
void InitIO(void)
{
  //  P1SEL &= ~0x3e;     //设置P1.4，P1.5为普通IO口  
  //  P1DIR &= ~0x30;     //按键接在P1.4，P1.5口上，设P1.4，P1.5为输入模式 
  //  P1INP &= ~0x30;     //打开P1.4，P1.5上拉电阻   
  P1SEL &= ~0x30;  //设置P1.4,P1.5为普通IO    P15-S1,P14-S2,P06-S3
  P1DIR &= ~0x30;  //键接在P1.4，P1.5口上，设P1.4，P1.5为输入模式 
  P1INP &= ~0x30;  //打开P1.4，P1.5上拉电阻   
  
  P0SEL &= ~0x40;  //设置P0.6为普通IO,是按键
  P0DIR &= ~0x40;  //键接在P0.6口上，设P0.6为输入模式 
  P0INP &= ~0x40;  //打开P0.6上拉电阻  
  
  P1SEL &= ~0x80;  //设置P1.7为普通IO      P17为充电信号采集
  P1DIR &= ~0x80;  //键接在P1.7口上，设P1.7为输入模式 
  P1INP |= 0x80;  //打开P1.7上拉电阻,错了，应该是设置为3态，为高阻状态。
  
  P2SEL &= ~0x1;  //设置P2.0为普通IO      P20为DS18B20信号采集
  P2DIR &= ~0x1;  //键接在P2.0口上，设P2.0为输入模式 
  P2INP &= ~0x1;  //打开P2.0上拉电阻
  
  P1DIR |= 0x40;   // P1.6定义为输出        P16是控制是否充电的  
  P1DIR |= 0x0e;   // P1.1，P1.2，P1.3定义为输出        P11,P12,P13是三色灯的控制线  
  P1DIR |= 0x01;   // P1.0定义为输出        P10是控制加热的  
  P0DIR |= 0x80;   // P0.7定义为输出        P07是控制震动电机是否震动的
  P0_7=0;
  P0DIR |= 0x30;   // P0.4,P0.5定义为输出，P05是PWM  115KHZ控制线，P04是辅助控制线（P04可不用）
  
}
/****************************************************************************
* 名    称: TEMP_CMP_SET()
* 功    能: 判断是否进入温度设置界面
* 入口参数: 无
* 出口参数: 
****************************************************************************/
void TEMP_CMP_SET(void)
{
     if ((KEY_ADD == 0) && (KEY_DEC == 0))     
    {
        Delay_ms(10);             
        if ((KEY_ADD == 0) && (KEY_DEC == 0))
        {
          while((!KEY_ADD)&&(!KEY_DEC));
          if(KEY_ADD_DEC_MARK==1)
          {
          MOTOR=OFF;
          KEY_ADD_DEC_MARK=0;
          }
          else
          {
          MOTOR=ON;
          KEY_ADD_DEC_MARK=1;
          }
          while((!KEY_ADD)&&(!KEY_DEC)); 
          KeyScan_finsh=1;
          /*
           if(KEY_SET_MARK==1)
           {
             Turnon_main_picture();//判断如果已经在设置界面，这样就是退出设置进入主界面，否则进入温度设置界面
             KEY_SET_MARK=0;
           }
           else
           {
             Turnon_set_picture();
             KEY_SET_MARK=1;
           }
           while((!KEY_ADD)&&(!KEY_DEC));
           KeyScan_finsh=1;
                                  //设置进入温度设置标志位,若标志位为ON,则OFF       
          */
        }
    }
}
/****************************************************************************
* 名    称: ShutDown()
* 功    能: 关机
* 入口参数: 无
* 出口参数: 0为抬起   1为按键按下
****************************************************************************/
void ShutDown(void)
{
 //HOT=OFF;
 Turn_off_picture();
 PwmVariable=0;
 PWM_Set(0,0);
 PWM_Pulse();
 KEY_POWER_MARK=0;
}
/****************************************************************************
* 名    称: KeyScan()
* 功    能: 读取按键状态并控制按键指示灯
* 入口参数: 无
* 出口参数: 0为抬起   1为按键按下
****************************************************************************/
uchar KeyScan(void)
{
    if (KEY_POWER == 0)         
    {
        Delay_ms(10);      //延时10MS去抖
        if (KEY_POWER == 0)
        {
            while(!KEY_POWER); //松手检测
               if(KEY_POWER_MARK==1)
               {
                 ShutDown();
                 
               }
               else
               {
                 RoomTemperature(); //加热前先计算室温
                 HOT=ON;
                 Turn_on_picture();
                 KEY_POWER_MARK=1;
                 
               } 
                  
         }
            return 1;     //有按键按下
        }
    
    
    if (KEY_POWER_MARK&&(KEY_ADD == 0))
    {
        Delay_ms(10);      //延时10MS去抖
        if (KEY_ADD == 0)
        {
           int tt=10;
           while(tt--)
            {
              TEMP_CMP_SET();
            }
            if(KeyScan_finsh==1)
            {
              KeyScan_finsh=0;
              return 2;
            }
            while(!KEY_ADD);
            TEMP_CMP=TEMP_CMP+1;
            MOTOR=OFF;
            if(TEMP_CMP>=TEMP_CMP_HIGH)TEMP_CMP=TEMP_CMP_HIGH;
            OLED_ShowNum(0,2,TEMP_CMP,4,16);  //对设置界面的设定温度进行调整，并显示出来。
            Delay_ms(300);
        }
        return 3;
    }
    
    
    if (KEY_POWER_MARK&&(KEY_DEC == 0))
    {
        Delay_ms(10);      //延时10MS去抖
        if (KEY_DEC == 0)
        {
           int tt=10;
           while(tt--)
            {
              TEMP_CMP_SET();
            }
            if(KeyScan_finsh==1)
            {
              KeyScan_finsh=0;
              return 2;
            }
            while(!KEY_DEC);
            TEMP_CMP=TEMP_CMP-1;
            if(TEMP_CMP<=TEMP_CMP_LOW)TEMP_CMP=TEMP_CMP_LOW;    
            OLED_ShowNum(0,2,TEMP_CMP,4,16);
            Delay_ms(300);
        }
        return 4;
    }   
#if 0
   if(!KEY_POWER_MARK)
   {
     PowerOffCharging();
   }
#endif    
/*
    
    if ((KEY_SET_MARK==1) &&(KEY_ADD == 0))
    {
        Delay_ms(10);      //延时10MS去抖
        if (KEY_ADD == 0)
        {
            while(!KEY_ADD)
            {
              TEMP_CMP_SET();
            }
            if(KeyScan_finsh==1)
            {
              KeyScan_finsh=0;
              return 2;
            }
            TEMP_CMP=TEMP_CMP+1;
            if(TEMP_CMP>=120)TEMP_CMP=120;
            OLED_ShowNum(0,2,TEMP_CMP,4,16);  //对设置界面的设定温度进行调整，并显示出来。
        }
        return 3;
    }
    
    
    if ((KEY_SET_MARK==1) &&(KEY_DEC == 0))
    {
        Delay_ms(10);      //延时10MS去抖
        if (KEY_DEC == 0)
        {
            while(!KEY_DEC)
            {
              TEMP_CMP_SET();
            }
            if(KeyScan_finsh==1)
            {
              KeyScan_finsh=0;
              return 2;
            }
            TEMP_CMP=TEMP_CMP-1;
            if(TEMP_CMP<=37)TEMP_CMP=37;
            OLED_ShowNum(0,2,TEMP_CMP,4,16);
        }
        return 4;
    }   
    
     if ((KEY_SET_MARK==0) &&(KEY_ADD == 0))
     {
        Delay_ms(10);      //延时10MS去抖
        if (KEY_ADD == 0)
        {
           int tt=10;
           while(tt--)
            {
              Delay_ms(10); 
              TEMP_CMP_SET();
            }
            if(KeyScan_finsh==1)
            {
              KeyScan_finsh=0;
              return 2;
            }
            while(!KEY_ADD)
            {
              MOTOR=ON;//开启震动
            }
              MOTOR=OFF;
        }
        return 5;
     }
    */
    
    return 0;             //无按键按下
}
/****************************************************************************
* 名    称: Turn_on_picture()
* 功    能: 开机显示画面
* 入口参数: 无
* 出口参数: 无
****************************************************************************/
void Turn_on_picture(void)
{
          OLED_Clear();
          OLED_DrawBMP(0,0,64,4,BMP2);	
          delay_ms(1000);
          OLED_Clear();
          OLED_DrawBMP(0,0,64,4,BMP3);	
          delay_ms(2000);
          OLED_Clear();
          OLED_DrawBMP(0,0,64,4,BMP_border);	
          OLED_DrawBMP(47,0,64,1,BMP_Battle_81744);
          OLED_DrawBMP(0,0,16,1,BMP_Hot81644);
          OLED_DrawBMP(23,0,39,1,BMP_ZigBeeUpDown);
          OLED_DrawBMP(0,2,64,4,Hoting);
          delay_ms(2000);
          OLED_DrawBMP(0,2,64,4,BMP0F);
          OLED_ShowCHinese(40,2,0);//显示℃
}

/****************************************************************************
* 名    称: Turnon_main_picture()
* 功    能: 进入主显示界面
* 入口参数: 无
* 出口参数: 无
****************************************************************************/
void Turnon_main_picture(void)
{
          OLED_Clear();
          OLED_DrawBMP(0,0,64,2,BMP4);	
          OLED_ShowCHinese(8,2,13);//加热中
          OLED_ShowCHinese(24,2,14);//
          OLED_ShowCHinese(40,2,15);//
          delay_ms(2000);
          OLED_DrawBMP(0,2,64,4,BMP0);
          OLED_ShowCHinese(40,2,0);//显示℃
}

/****************************************************************************
* 名    称: Turnon_set_picture()
* 功    能: 进入设置显示界面
* 入口参数: 无
* 出口参数: 无
****************************************************************************/
void Turnon_set_picture(void)
{
          OLED_Clear();
          OLED_ShowCHinese(0,0,16);
          OLED_ShowCHinese(16,0,17);
          OLED_ShowCHinese(40,2,0);//显示℃
          OLED_ShowNum(0,2,TEMP_CMP,4,16);
}
/****************************************************************************
* 名    称: Turn_off_picture()
* 功    能: 关机显示画面
* 入口参数: 无
* 出口参数: 无
****************************************************************************/
void Turn_off_picture(void)
{
          OLED_Clear();
          OLED_DrawBMP(0,0,64,4,BMP2);	
          delay_ms(1000);
          OLED_Clear();
}

/****************************************************************************
* 名    称: InitCLK()
* 功    能: 设置系统时钟源
* 入口参数: 无
* 出口参数: 无
****************************************************************************/
void InitCLK()
{
    CLKCONCMD &= ~0x40;             //设置系统时钟源为32MHZ晶振
    while(CLKCONSTA & 0x40);        //等待晶振稳定为32M
    CLKCONCMD &= ~0x47;             //设置系统主时钟频率为32MHZ   
}

/****************************************************************************
* 名    称: Heating_control()
* 功    能: 加温控制
* 入口参数: 无
* 出口参数: 无
****************************************************************************/
void Heating_control(void)
{
  if(KEY_POWER_MARK)
  {
    if(ucTemp>=TEMP_CMP)
    {
    //  HOT=OFF;
      PwmVariable=0;
      PWM_Set(0,0);
      PWM_Pulse();
      
    }
    else
    {
      if((TEMP_CMP-ucTemp)>=1)
      {
    //    HOT=ON;
        PWM_Hot();
      }
    }
  }
  //当温度达到或超过设定值时，停止加热
  //当温度低于一个偏离值时，开启加热，温度偏移值用宏定义
}
/****************************************************************************
* 名    称: Temperature_conversion_shows()
* 功    能: 温度转换显示
* 入口参数: 无
* 出口参数: 无
****************************************************************************/
void Temperature_conversion_shows(void)
{
         if(KEY_POWER_MARK)//&&(KEY_SET_MARK==0))
          {
          Surface_temperature_conversion();
          }
}
/****************************************************************************
* 名    称: battery_charging()
* 功    能: 电池充电
* 入口参数: 无
* 出口参数: 无
****************************************************************************/
void battery_charging(void)
{
  if(HOT==ON)
  {
    battery_control=ON;
  }
  else 
  {
    if(battery_state)
    {
      battery_control=ON;
    }
    else battery_control=OFF;
  }

  //当关闭加热的时候，允许充电(含当关机的时候，允许充电）,当充电满时关闭充电
}
/****************************************************************************
* 名    称: GetVoltage()
* 功    能: AD 值
* 入口参数: 无
* 出口参数: 通过计算返回实际的温度值
****************************************************************************/
/*
float GetVoltage(void)
{ 
   unsigned int  value; 
   unsigned char VolADCCON3 = ADCCON3;
   
   ADCIF = 0;   
   
   //选择1.25V为参考电压；12位分辨率；对片内温度传感器采样
   ADCCON3  = (HAL_ADC_REF_1V25 | ADC_12_BIT | ADC_EMP_SENS);            
   
   while(!ADCIF);                     //等待 AD 转换完成 
   value =  ADCL >> 2;                //ADCL 寄存器低 2 位无效 
   value |= (((unsigned int)ADCH) << 6);

   ADCCON3 = VolADCCON3;

   return ADC_TO_CELSIUS(value);                               
}
*/
/******************************************************************************
*函 数 名：Read_advalue
*功    能：ADC电池电压
*入口参数：参考电压 reference、转换通道 channel、分辨率resolution
*出口参数：ADC转换结果
******************************************************************************/
uint Read_advalue(uchar reference, uchar channel, uchar  resolution)
{
   uint value; 
   uchar tmpADCCON3 = ADCCON3;
   
   APCFG |= 1 << channel ; //设置ADC输入通道,模拟I/O使能
/*
   //APCFG (0xF2) – Analog Peripheral I/O Configuration
Bit Name Reset R/W Description
7:0 APCFG[7:0]
//0x00 R/W Analog Perpheral I/O configuration . APCFG[7:0] select P0.7–P0.0 as 
//analog I/O.
0: Analog I/O disabled
1: Analog I/O enabled
//使能模拟IO口，就是使能ADC采集引脚吧
*/
   ADCCON3  = (reference | resolution | channel);
   ADCIF = 0;                         //
 
   while(!ADCIF);               //等待 AD 转换完成 
   value =  ADCL >> 2;          //ADCL 寄存器低 2 位无效
   value |= ((uint)ADCH << 6);  //连接AD转换结果高位和低位 
   
 //根据分辨率获得ADC转换结果有效位 
   switch(resolution)
   {
     case ADC_7_BIT:  value >>= 7;break;
     case ADC_9_BIT:  value >>= 5;break;
     case ADC_10_BIT: value >>= 4;break;
     case ADC_12_BIT: value >>= 2;break;
     default:;
   }
   
   ADCCON3 = tmpADCCON3;
   return (value);
 }

/******************************************************************************
*函 数 名：
*功    能：UART初始化
*入口参数：
*出口参数：
******************************************************************************/
void InitUART(void)
{
  PERCFG &= ~0x01;     //USART0为位置1
  P2DIR &= ~0xc0;      //优先级USART0 >USART1 >定时器1
  P0SEL |= 0x0c;       //P0_2,P0_3 用作串口（外部设备功能）
  U0CSR |= 0x80;       //设置为UART 方式
  U0CSR |= 0x40;       //使能接收器
  U0UCR = 0x02;        //禁止流控制，8bits,无校验位，1位停止位
                       //起始位低电平，停止位高电平
  U0GCR |=8;          //32MHz 下的BAUD_E:11, 115200    8    9600
  U0BAUD |= 59;       //32MHz 下的BAUD_M:216 115200    59   9600
  UTX0IF = 0;          //UART0 TX 中断标志初始置位0
  IEN0 |= 0x04;        //使能USART0 RX中断
  EA = 1;              //开总中断
}
  
/******************************************************************************
*函 数 名：UART0_ISR
*功    能：串口0中断服务程序
*入口参数：
*出口参数：
******************************************************************************/
#pragma vector = URX0_VECTOR 
__interrupt void UART0_ISR(void) 
{
  URX0IF = 0;           //UART0 RX中断标志位清0
  temp = U0DBUF;        //读取U0DBUF的值
}

/****************************************************************************
* 名    称: UartSendString()
* 功    能: 串口发送函数
* 入口参数: Data:发送缓冲区   len:发送长度
* 出口参数: 无
****************************************************************************/
void UartSendString(char *Data, int len)
{
    uint i;
    
    for(i=0; i<len; i++)
    {
        U0DBUF = *Data++;
        while(UTX0IF == 0);
        UTX0IF = 0;
    }
}
/****************************************************************************
* 名    称: PowerAD()
* 功    能: AD转换检测电池电压，以0.1V来分辨
* 入口参数: 
* 出口参数: 
****************************************************************************/
uint PowerAD(unsigned char Pnn)
   {

     uint vddvalue;       //ADC转换值 
 //ADC参考电压AVDD5引脚电源电压：3.3V，分辨率12位，采集通道：VDD/3,VDD=3.3V
     vddvalue = Read_advalue(HAL_ADC_REF_1V25, Pnn , ADC_12_BIT);  //0x0F为VDD/3，01为P01  04为P04  HAL_ADC_REF_1V25  ADC_REF_AVDD5
  //   vddvalue = (vddvalue*125) >> 11;   //这里存在一个问题，当经过运算后，再加上3.3V不稳，不是USB电压不对，就是锂电池电压不对。当不添加运算时都挺好的
  //   vddvalue = vddvalue*4;
  //  vddvalue = (vddvalue*33)/2048;
#if 0
     buf[0] = vddvalue/10 + '0';
     buf[1] = '.';
     buf[2] =vddvalue%10 + '0';
     buf[3] =0x0;
     UartSendString(buf,strlen(buf));  //串口上传采样VDD值   //代码存在问题，strlen计算出来的长度比实际长度多一位。
                                         //找到问题了，应该接受字符的存储位置放到了数组后面，把数组扩大，最后一个给0，可以解决这个问题
                                         //之前给出的buf[3]，共3个数，现在改为buf[6]，结束位buf[3] =0x0;
     ucTemp=vddvalue;
     OLED_ShowNum(0,2,ucTemp,4,16);
     delay_ms(2000);                  //每隔2s上传一次值
 #endif
     return vddvalue;
   }
   

/****************************************************************************
* 名    称: BattleCycle()
* 功    能: 小电池图标循环显示
* 入口参数: 
* 出口参数: 
****************************************************************************/
void BattleCycle(void)
{
  if(UsbPowerElectricize&&(!P1_7))
  {
    switch(BattleCycleCount/3)
    {
    case 0:OLED_DrawBMP(47,0,64,1,BMP_Battle_81740);BattleCycleCount++;break;
    case 1:OLED_DrawBMP(47,0,64,1,BMP_Battle_81741);BattleCycleCount++;break;
    case 2:OLED_DrawBMP(47,0,64,1,BMP_Battle_81742);BattleCycleCount++;break;
    case 3:OLED_DrawBMP(47,0,64,1,BMP_Battle_81743);BattleCycleCount++;break;
    case 4:OLED_DrawBMP(47,0,64,1,BMP_Battle_81744);BattleCycleCount++;break;
    default: 
      {
        if(PowerAD(PnUsb)<PnUsbPower4_4)UsbPowerElectricize=0;
        BattleCycleCount=0;
      }break;

    }
  }
  if(P1_7==1)OLED_DrawBMP(47,0,64,1,BMP_Battle_817Finish);  //当充电满时提示小的满格图标
}

/****************************************************************************
* 名    称: Error(void) 
* 功    能: 探测N次后发现温度没有变化,也不等于温度设定值,则提示:不加热?返厂维修
* 入口参数: 
* 出口参数: 
****************************************************************************/
void Error(void)     
{
  if(ucTemp_on==ucTemp)   //与上一次温度ucTemp_on比较，若相同且不等于目标温度则计数，计数N次后则认为加热失灵。
  {
    if(ucTemp_on==TEMP_CMP)ErrorCount=0;
    else ErrorCount++;
  }
  else ErrorCount=0;
  ucTemp_on=ucTemp;

  if(ErrorCount>=10000)
  {
    PwmVariable=0;
    PWM_Set(0,0);
    PWM_Pulse();
    while(1)
    {
      OLED_ShowCHinese(0,0,18);//不
      OLED_ShowCHinese(16,0,19);//加
      OLED_ShowCHinese(32,0,20);//热
      OLED_ShowCHinese(48,0,21);//？
      OLED_ShowCHinese(0,2,22);//返
      OLED_ShowCHinese(16,2,23);//厂
      OLED_ShowCHinese(32,2,24);//维
      OLED_ShowCHinese(48,2,25);//修
      delay_ms(1000);
    }
  }
}

/****************************************************************************
* 名    称: PWM_Hot()
* 功    能: PWM调节温度启动
* 入口参数: 
* 出口参数: 
****************************************************************************/
void PWM_Hot(void)
{
  uint i,j;
  if(KEY_POWER_MARK==1)
  { 
    if(PwmVariable>=255)PwmVariable=255;  //软启动到最大脉冲
    
    else 
    {
  //  PwmVariable=PwmVariable*2;
    PwmVariable++;
    }
//    OLED_ShowNum(0,2,PwmVariable,3,16);
    PWM_Set(PwmVariable,0);//数字0是准备给另外一个IO的，先保留
    PWM_Pulse();
    delay_ms(10);
    i=VoltageMean(PnLi);
    test1=i;
    j=VoltageMean(PnUsb);
    AD_AbnormalTermination;  //因为在实际测试时发现电压悬空，无电压状态时采集出来的结果大于4000，是错误的，故暂时先用这个办法解决此问题
    test2=j;
    if(j>PnUsbPower4_4)
    {  
   //   OLED_ShowNum(0,2,j,4,16);
   //   delay_ms(2000);
      UsbPowerElectricize=1;
      return;
    }
    UsbPowerElectricize=0;
    if(i>=PnLiPower4_4)OLED_DrawBMP(47,0,64,1,BMP_Battle_81744);
    if((i<PnLiPower4_4)&&(i>=PnLiPower4_3))OLED_DrawBMP(47,0,64,1,BMP_Battle_81743);
    if((i<PnLiPower4_3)&&(i>=PnLiPower4_2))OLED_DrawBMP(47,0,64,1,BMP_Battle_81742);
    if((i<PnLiPower4_2)&&(i>=PnLiPower4_1))OLED_DrawBMP(47,0,64,1,BMP_Battle_81741);
    if((i<PnLiPower4_1)&&(i>=PnLiPower4_0))OLED_DrawBMP(47,0,64,1,BMP_Battle_81740);
    if(i<PnLiPower4_0)
    {
      PowerLow=1;   //提示电压不足，准备关机。
      PwmVariable=0;
    }
    else PowerLow=0;
    
  }
}
/****************************************************************************
* 名    称: PowerLow_Shutdown()
* 功    能: 电池供电电压低自动关机
* 入口参数: 
* 出口参数: 
****************************************************************************/
void PowerLow_Shutdown(void)
{
  if(PowerLow==1)
  {
    PWM_Set(0,0);  //先设置为0,避免自动没电，连显示画面的机会都没有了
    PWM_Pulse();
    P0_7=1;                        //先BI一声
    OLED_DrawBMP(0,0,64,4,BMP_Please);   //电压低修改为请充电！
    for(char i=0;i<50;i++)
    {
            //OLED_DrawBMP(0,0,64,4,BMP_Please);//OLED_DrawBMP(0,2,64,4,PowerLow_1664);   电压低修改为请充电！
            //delay_ms(500);
            //OLED_Clear(); 
            //delay_ms(500);
      delay_ms(1000);
      if(LoopCheck())     //170216
      {
        PowerLow=0;
        return;
      }
      
    }
    P0_7=0;
    ShutDown();
    PowerLow=0; //清除标志位
  }
}

/****************************************************************************
* 名    称: Heating_intensity_icon()
* 功    能: 加热强度图标
* 入口参数: 
* 出口参数: 
****************************************************************************/
void Heating_intensity_icon(void)
{
#if 1
  if(KEY_POWER_MARK==1)                        //目前显示的是设定的强度
  {
    if(TEMP_CMP==TEMP_CMP_HIGH)OLED_DrawBMP(0,0,16,1,BMP_Hot81644);
    if((TEMP_CMP<TEMP_CMP_HIGH)&&(TEMP_CMP>=(TEMP_CMP_LOW+(3*(TEMP_CMP_HIGH-TEMP_CMP_LOW))/4)))OLED_DrawBMP(0,0,16,1,BMP_Hot81643);
    if((TEMP_CMP<(TEMP_CMP_LOW+(3*(TEMP_CMP_HIGH-TEMP_CMP_LOW))/4))&&(TEMP_CMP>=(TEMP_CMP_LOW+(2*(TEMP_CMP_HIGH-TEMP_CMP_LOW))/4)))OLED_DrawBMP(0,0,16,1,BMP_Hot81642);
    if((TEMP_CMP<(TEMP_CMP_LOW+(2*(TEMP_CMP_HIGH-TEMP_CMP_LOW))/4))&&(TEMP_CMP>=(TEMP_CMP_LOW+(1*(TEMP_CMP_HIGH-TEMP_CMP_LOW))/4)))OLED_DrawBMP(0,0,16,1,BMP_Hot81641);
    if(TEMP_CMP<=TEMP_CMP_LOW)OLED_DrawBMP(0,0,16,1,BMP_Hot81640);
  }
#endif
}

/****************************************************************************
* 名    称: LoopCheck()
* 功    能: 循环中检查
* 入口参数: 
* 出口参数: 
****************************************************************************/
u8 LoopCheck(void)
{
  uint j;
  j=VoltageMean(PnUsb);
  AD_AbnormalTermination;
  if((j<PnUsbPower4_4) || (KEY_POWER == 0))
    {
      OLED_Clear(); 
      return 0;
    }
  else 
    {
      return 1;
    }
}

/****************************************************************************
* 名    称: Charge_complete()
* 功    能: 充电完成处理
* 入口参数: 
* 出口参数: 
****************************************************************************/
void Charge_complete(void)
{
  if(P1_7==1)
    {
      delay_ms(10);
      if(P1_7==1)  //确认电已经充满，给出图标显示，然后BI一声
      {
        OLED_DrawBMP(0,0,64,4,BMP_Battle_6432Finish);
        P0_7=1;
        delay_ms(1000);
        P0_7=0;
  //      OLED_Clear();    //先不清除屏幕
        Battle_6432Finish=1;
      }
    }
  
}
/****************************************************************************
* 名    称: PowerOffCharging()
* 功    能: 关机充电
* 入口参数: 
* 出口参数: 
****************************************************************************/
void PowerOffCharging(void)
{
  uint j;
  j=VoltageMean(PnUsb);
  AD_AbnormalTermination;
//  OLED_ShowNum(0,2,j,4,16);
//  delay_ms(100);
#if 1
  if(j<PnUsbPower4_4)
    {
      OLED_Clear(); 
      UsbPowerElectricize=0; //USB充电状态标志
      LoopClearOver=0;       //USB充电循环图标是否执行过，如果断开过电压就要重新执行，否则不执行
      Battle_6432Finish=0;   //充电满标志提醒一次
      return;
    }
#if 0
  if(LoopClearOver==1)
  {
    OLED_Clear(); 
    UsbPowerElectricize=0;
    if((P1_7==1)&&(Battle_6432Finish==0))
    {
      delay_ms(10);
      if(P1_7==1)  //确认电已经充满，给出图标显示，然后BI一声
      {
        OLED_DrawBMP(0,0,64,4,BMP_Battle_6432Finish);
        P0_7=1;
        delay_ms(1000);
        P0_7=0;
  //      OLED_Clear();    //先不清除屏幕
        Battle_6432Finish=1;
      }
    }
  }
#endif
 // else UsbPowerElectricize=1;
  //if(LoopClearOver==1)return;

  while(!KeyScan())
  {   
    if(P1_7==1)
    {
      Charge_complete();
    }
    else
    {
  OLED_DrawBMP(0,0,64,4,BMP_Battle_643240);
  delay_ms(300);
  if(!LoopCheck())return;
  OLED_DrawBMP(0,0,64,4,BMP_Battle_643241);
  delay_ms(300);
  if(!LoopCheck())return;
  OLED_DrawBMP(0,0,64,4,BMP_Battle_643242);
  delay_ms(300);
  if(!LoopCheck())return;
  OLED_DrawBMP(0,0,64,4,BMP_Battle_643243);
  delay_ms(300);
  if(!LoopCheck())return;
  OLED_DrawBMP(0,0,64,4,BMP_Battle_643244);
  delay_ms(300);
  if(!LoopCheck())return;
    }
  }
  //OLED_Clear(); 
 // LoopClearOver=1;//待会再处理，先处理第一个问题
#endif
}
/****************************************************************************
* 名    称: RoomTemperature()
* 功    能: 计算室内温度
* 入口参数: 无
* 出口参数: 无
****************************************************************************/
void RoomTemperature(void)
{
  uint j=0,k=0;
  for(int i=0;i<10;i++)
  {
    j+=(char)DS18B20_ReadMain();
    delay_ms(1);
  }
  k=(char)j/10;
  if(k>30)RoomTemperatureValue=25;
  else RoomTemperatureValue=(unsigned char)k;
    
  //上电前测温十次，求平均值，每次测试间隔1MS
  //如果温度超过30度，则应用之前上一次测室温的值，否则默认室温25度。
}

/****************************************************************************
* 名    称: unsigned char TemperatureMean(void)
* 功    能: 求电池平均值
* 入口参数: 无
* 出口参数: 无
****************************************************************************/
uint VoltageMean(unsigned char Pnn)
{
  uint j=0,k=0;
#if 0
  j+=PowerAD(Pnn);
  OLED_ShowNum(0,2,j,4,16);
  j=0;
  delay_ms(2000);
#endif
  for(int i=0;i<10;i++)
  {
    j+=PowerAD(Pnn);
    delay_ms(1);
  }
  k=(j/10);
//  OLED_ShowNum(0,2,k,4,16);
//  delay_ms(2000);
  return k;
}

/****************************************************************************
* 名    称: Surface_temperature_conversion()
* 功    能: 实际温度与表面温度的转换程序
* 入口参数: 无
* 出口参数: 无
****************************************************************************/
void Surface_temperature_conversion(void)
{
  int i=0;
   i=(char)DS18B20_ReadMain();
  if(i<=RoomTemperatureValue)
  {
    ucTemp=RoomTemperatureValue;//如果测得的温度小于室温，则显示室温

  }
  else
  {
    ucTemp=thermometer[i-1];

  }
    OLED_DrawBMP(0,2,16,4,BMP0F81); //暂时先用这个代码解决，后续再优化吧
    OLED_ShowNum(0,2,ucTemp,4,16);
    delay_ms(10);
#if 1                        //小心烫伤的警告提醒!
    if(ucTemp>=TEMP_CMP_TipBurn)high_temperature_count++;
    if(high_temperature_count>=TEMP_CMP_TipBurn_count)  //一定次数后才提示警告.
    {
     high_temperature_count=0;
     OLED_DrawBMP(0,2,64,4,BMP_Becareful);
     delay_ms(500);
     OLED_DrawBMP(0,2,64,4,BMP0F41);  //清除下班部分内容,并显示当前值和℃
     OLED_ShowNum(0,2,ucTemp,4,16);
     OLED_ShowCHinese(40,2,0);//显示℃
    }
    
#endif    
} 



/****************************************************************************
* 名    称: IO_Reset()
* 功    能: IO口复位函数
* 入口参数: Data:发送缓冲区   len:发送长度
* 出口参数: 无
****************************************************************************/
void IO_Reset(void)
{
  P1_0=0; //发热控制端口，上电进行复位。
//  P1_7=0; //充电满检测，上电进行复位，测试用。
  P1_6=1; //充电使能控制端，上电给高电平，是直接给值还是先配置端口后给值？
}

/****************************************************************************
* 名    称: main()
* 功    能: 程序入口
* 入口参数: 无
* 出口参数: 无
****************************************************************************/
#if 0
 int 11main(void)
  {	
    IO_Reset();
    
    InitCLK();  
    InitIO();
#if 0
    P1_0=1;
    while(1);
#endif
                //增加一个清除所有IO口的函数

    PWM_Set(0,0);
    PWM_Init();  //62.5kHZ
    PWM_Pulse();

    /*
    while(1)
    {
    delay_ms(2000);  
    PWM_Set(100,0);
    PWM_Pulse();
    delay_ms(2000);
    PWM_Set(0,0);
    PWM_Pulse();
    }
    */
               //需要把端口先配置成高组态，再然后依据实际应用来配置。解决电池充电时莫名LED会点亮，待写代码
//     P1_0=0;  
     P0_7=0;
    I2CCFG = 0;             
    I2CWC |= 0x83;
    I2CIO |=0x0;
    OLED_Init();	
    OLED_Clear(); 

   InitUART();          //UART0串口初始化
 //  PIDInit ( &spid ); // Initialize Structure 
   
   battery_control=OFF;  //允许电池充电
 // OLED_DrawBMP(0,0,64,4,BMP0F);
 //   delay_ms(2000);
  // P1_6=0;
   
   
   
#if 0
while(1)
{
  P0_7=1;
  delay_ms(2000);
  P0_7=0;
  delay_ms(2000);
}
#endif   
#if 0
   // P1_0=1;
    while(1)
    {
    PWM_Set(0,0);
    PWM_Init();  //62.5kHZ
    PWM_Pulse();
     while(1)
     {
    if(PwmVariable<255)PwmVariable++;
    if(PwmVariable==255)
    {
    }
    else
    {
      //PwmVariable=255;
    OLED_ShowNum(0,2,PwmVariable,3,16);
    PWM_Set(PwmVariable,0);//数字0是准备给另外一个IO的，先保留
    PWM_Pulse();
    delay_ms(20);
     }
    }
    }
#endif  
#if 0   
       while(1)
    {
      int qq;
      qq=P1_7;
      if(qq==0)OLED_ShowNum(0,2,qq,1,16);
      if(qq==1)OLED_ShowNum(0,2,qq,1,16);
    }
#endif
#if 0    
   while(1)
   {
     u8 app=2;
     OLED_ShowChar(32,3,app,16);
    //Lcd_Pixel(0,2,16,16*8,Hoting);	
   }
#endif
#if 0
   while(1)
   {
     OLED_Clear();
     delay_ms(500);     
     OLED_DrawBMP(0,0,64,4,BMP_border);	
    OLED_ShowCHinese(8,2,13);//加热中
    OLED_ShowCHinese(24,2,14);//
    OLED_ShowCHinese(40,2,15);//
    OLED_DrawBMP(0,4,64,4,line);	
    delay_ms(2000);
    OLED_DrawBMP(0,2,64,4,Hoting);	
    delay_ms(2000);
    //OLED_DrawBMP(0,2,64,4,BMP0);
    OLED_ShowCHinese(40,2,0);//显示℃
    OLED_ShowNum(0,2,1,1,16);
     delay_ms(5000);
     for(int i=1;i<10;i++)
     {
     OLED_DrawBMP(47,0,64,1,BMP_Battle_81740);
     OLED_DrawBMP(0,0,16,1,BMP_Hot81640);
     OLED_DrawBMP(23,0,39,1,BMP_ZigBeeUpDown);
     OLED_DrawBMP(0,2,64,4,Hoting);
     
     delay_ms(300);
     OLED_DrawBMP(47,0,64,1,BMP_Battle_81741);
     OLED_DrawBMP(0,0,16,1,BMP_Hot81641);
     OLED_DrawBMP(23,0,39,1,BMP_ZigBeeUp);
     delay_ms(300);
     OLED_DrawBMP(47,0,64,1,BMP_Battle_81742);
     OLED_DrawBMP(0,0,16,1,BMP_Hot81642);
     OLED_DrawBMP(23,0,39,1,BMP_ZigBeeDown);
     delay_ms(300);
     OLED_DrawBMP(47,0,64,1,BMP_Battle_81743);
     OLED_DrawBMP(0,0,16,1,BMP_Hot81643);
     OLED_DrawBMP(23,0,39,1,BMP_ZigBeeUp);
     delay_ms(300);
     OLED_DrawBMP(47,0,64,1,BMP_Battle_81744);
     OLED_DrawBMP(0,0,16,1,BMP_Hot81644);
     OLED_DrawBMP(23,0,39,1,BMP_ZigBeeDown);
     OLED_ShowNum(0,2,1,4,16);
     delay_ms(300);
     
     }
   }
#endif

    

          while(1)
          {
            KeyScan();
            Heating_control();
            Temperature_conversion_shows();
//            Pid_main();  //170222
            BattleCycle();
            Heating_intensity_icon();
            PowerLow_Shutdown();//电压不足自动关机，关机前显示界面和BI一声PowerLow_Shutdown
            if(KEY_POWER_MARK)Error();
            if(!KEY_POWER_MARK)PowerOffCharging();
 //           battery_charging();  //电池充电控制暂时屏蔽，用电源可以一边充电一边使用。
          }	  
	
  }
          
        //ucTemp = ReadDs18B20();           //温度读取函数
        //strTemp[0] = ucTemp/10+16;        //取出十位数
        //strTemp[1] = ucTemp%10+16;        //取出个位数
       // OLED_ShowString(0,2,"  11",16);
          /*
	   OLED_ShowString(0,0,"AAAAAAAA",8);
           delay_ms(1000);
		 OLED_ShowString(0,1,"BBBBBBBB",8);
                 delay_ms(1000);
		 OLED_ShowString(0,2,"   45678",8);
                 delay_ms(1000);
		OLED_ShowString(0,3,"12345678",8);

	  	delay_ms(1000);
	 	  OLED_Clear();
			OLED_ShowString(0,0,"0.49OLED",8);
		 OLED_ShowString(0,1,"12345678",8);
  	 OLED_ShowString(0,2,"1234AABB",16);

	  	delay_ms(1000);
	 	  OLED_Clear();
          
		OLED_ShowCHinese(8,0,0);//中
		OLED_ShowCHinese(24,0,1);//景
		OLED_ShowCHinese(40,0,2);//园
		OLED_ShowCHinese(8,2,7);//超
		OLED_ShowCHinese(40,2,8);//亮
	  	delay_ms(1000);
			OLED_Clear();
		OLED_DrawBMP(0,0,64,4,BMP1);	
		delay_ms(1000);
			delay_ms(1000);
	  	delay_ms(1000);
			 OLED_Clear();

          
                OLED_ShowCHinese(0,0,9);//梦
		OLED_ShowCHinese(16,0,10);//回
		OLED_ShowCHinese(32,0,11);//唐
		OLED_ShowCHinese(48,0,12);//朝
                    */
#endif