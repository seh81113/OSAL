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


#define LED1 P1_1           // ����P1.1��ΪLED3����ɫ�����к�
#define LED2 P1_2           // ����P1.2��ΪLED4����ɫ��������
#define LED3 P1_3           // ����P1.3��ΪLED5����ɫ��������
#define KEY_POWER P1_5      // ����P1.5��ΪS1���ƶ�,���ػ�����
#define KEY_ADD P1_4        // ����P1.4��ΪS2���ƶˣ�����+
#define KEY_DEC P0_6        // ����P1.4��ΪS2���ƶˣ�����-
#define MOTOR  P0_7         // ����P0.7��Ϊ������ж˿�
#define PEAK  P0_0          // ����P0.0��Ϊ���ȶ˿ڣ���ʱ����
#define Ultrasonic_atomizer_bak  P0_4          // ����P0.4��Ϊ���������ö˿ڣ��ɲ���
#define Ultrasonic_atomizer  P0_5              // ����P0.5��Ϊ���������ƣ�PWM  115KHZ
#define battery_control  P1_6                  // ����P1.6��Ϊ�����ƶ˿�
#define battery_state  P1_7                    // ����P1.7��Ϊ���Ϊ����Ƿ񱥺͵�״̬�˿�
#define HOT   P1_0          //����P1.0��Ϊ���ȿ��ƶ˿ڣ�ʹ��PWM���Ƶķ�ʽ������
#define DS18B20   P2_0      //����P2.0��Ϊ�¶Ȳɼ�ͨѶ��
#define low_temperature 2   //���õ����ߣ��¶ȵ���ʱֹͣ���ȣ��������¶ȼ������ߣ����¶ȵ��������¶�һ��ֵʱ�����ȿ���
#define Offset_temperature 10   //�¶�ƫ��ֵ����ʵ���¶�����Ƥ�¶ȴ���ƫ����Ը���һ��У����Ŀǰ���ô�����У��������������Ͳ��ķ�ʽУ׼
#define ON      1
#define OFF     0
#define H_time  20000        //����ʱ��   ������1����
#define U_time  700          //����ʱ��
#define PnLi    0x01         //����ΪP01��
#define PnUsb   0x04         //����ΪP04��
#define PnLiPower4_4     1400        //�����4/4      1400ʱ��3.6V
#define PnLiPower4_3     1300        //�����4/3
#define PnLiPower4_2     1200        //�����4/2
#define PnLiPower4_1     1100        //�����4/1
#define PnLiPower4_0     950        //�����4/0       950ʱ��2.5V   1050ʱ��2.75V  760ʱ2V
#define PnUsbPower4_4     1600        //USB���磬��ѹ����4.6VΪUSB����״̬,4.6V��Լ��1800,  1670Ϊ4.4V ,����ʱ�������ѹ�������������ֵҪ���õ�һЩ�����Ⲩ��������жϡ�
#define HAL_ADC_REF_1V25                0x00
#define ADC_REF_AVDD5                   0x80
#define ADC_7_BIT                       0x0
#define ADC_9_BIT                       0x10
#define ADC_10_BIT                      0x20
#define ADC_12_BIT                      0x30
#define ADC_EMP_SENS                    0x0E
#define ADC_TO_CELSIUS(ADC_VALUE)       ((ADC_VALUE>>4)-334)   //�¶�У��
#define TEMP_CMP_HIGH                   70    //ԭ��120�ȣ����ڵ�����λ����߸���70�ȡ�
#define TEMP_CMP_TipBurn                65     //�������¶ȣ���ʾС������
#define TEMP_CMP_TipBurn_count          20     //�������¶�ÿ��20������,��ʾС������
#define TEMP_CMP_LOW                    37
#define AD_AbnormalTermination   if(j>2048)j=0    //����޵�ѹ�����쳣
typedef unsigned char uchar;
typedef unsigned int  uint;
unsigned char strTemp[2];
char buf[6];
char ucTemp=0,ucTemp_on=0,high_temperature_count=0;
char PowerLow=0;  //LI��ص�ѹ�ͱ�־λ����Ϊ1���ѹ�ͣ�����LI��ؿ��Թ�����
char UsbPowerElectricize=0; //USB��Դ���״̬��־
uint PwmVariable=10;//PWM���ڼ��ȱ���
char LoopClearOver=0;       //USB���ѭ��ͼ���Ƿ�ִ�й�������Ͽ�����ѹ��Ҫ����ִ�У�����ִ��
uint BattleCycleCount=0;  //���Сͼ��ѭ���������
extern  char temp=0;  //����0�ж�ʱ��ʱʹ��
extern  uchar KEY_POWER_MARK=0,KEY_ADD_MARK=0,KEY_DEC_MARK=0,KEY_SET_MARK=0;   //������״̬��־λ��Ϊ1���������¹�
extern  uchar KEY_ADD_DEC_MARK=0;
extern  uchar TEMP_CMP=60;  //������ʼ�¶�50��
extern  uchar KeyScan_finsh=0; //����������ͬʱ���º󣬴�����ɺ����һ����־
extern  uchar Battle_6432Finish=0;  //��س��������ʾ��BI�Ƿ�ִ�еı�־λ
extern  uint test1=0;  
extern  uint test2=0;  
extern  uchar RoomTemperatureValue=25; //����Ĭ������25��
extern  uint ErrorCount=0;


#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr)[0])
#define FLOAT_TEMP      0          //������߾���ʱ�򿪴�ע��
extern void Delay_ms(unsigned int k);//�ⲿ����ms������
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