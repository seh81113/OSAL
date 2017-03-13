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

/****************************************************************************
* ��    ��: 
* ��    ��: ��ʱ����ʱ����
* ��ڲ���: 
* ���ڲ���: ��
****************************************************************************/

void Delay_us(unsigned int k)//us��ʱ����
{
    T1CC0L = 0x06; 
    T1CC0H = 0x00; 
    T1CTL = 0x02; 
    while(k)
    { 
        while(!(T1CNTL >= 0x04));
        k--;
    }
    T1CTL = 0x00;  //�رն�ʱ��
}

void Delay_ms(unsigned int k)
{
    T1CC0L = 0xe8;
    T1CC0H = 0x03;
    T1CTL = 0x0a; //ģģʽ 32��Ƶ
    while(k)
    {
        while(!((T1CNTL >= 0xe8)&&(T1CNTH >= 0x03)));
        k--;
    }
    T1CTL = 0x00; //�رն�ʱ��
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
* ��    ��: LedOnOrOff()
* ��    ��: ������Ϩ������LED�� ���߶˿�   
* ��ڲ���: modeΪ1ʱLED����  modeΪ0ʱLED����
* ���ڲ���: ��
****************************************************************************/
void LedOnOrOff(uchar mode)
{
    LED1 = mode;
    LED2 = mode;
    LED3 = mode; 
}

/****************************************************************************
* ��    ��: InitLed()
* ��    ��: ����LED��Ӧ��IO��
* ��ڲ���: ��
* ���ڲ���: ��
****************************************************************************/
void InitLed(void)
{
    P1DIR |= 0x0f;  // P1.0��P1.1��P1.2��P1.3����Ϊ���   P1DIR�˿�1����Ĵ���
    P0DIR |= 0xC4;  // P0.2��P0.6��P0.7����Ϊ���   P0DIR�˿�1����Ĵ���
    LedOnOrOff(0);  // ʹ����LED��Ĭ��ΪϨ��״̬  
}

/****************************************************************************
* ��    ��: InitIO()
* ��    ��: ���ð�����Ӧ��IO��
* ��ڲ���: ��
* ���ڲ���: ��
****************************************************************************/
void InitIO(void)
{
  //  P1SEL &= ~0x3e;     //����P1.4��P1.5Ϊ��ͨIO��  
  //  P1DIR &= ~0x30;     //��������P1.4��P1.5���ϣ���P1.4��P1.5Ϊ����ģʽ 
  //  P1INP &= ~0x30;     //��P1.4��P1.5��������   
  P1SEL &= ~0x30;  //����P1.4,P1.5Ϊ��ͨIO    P15-S1,P14-S2,P06-S3
  P1DIR &= ~0x30;  //������P1.4��P1.5���ϣ���P1.4��P1.5Ϊ����ģʽ 
  P1INP &= ~0x30;  //��P1.4��P1.5��������   
  
  P0SEL &= ~0x40;  //����P0.6Ϊ��ͨIO,�ǰ���
  P0DIR &= ~0x40;  //������P0.6���ϣ���P0.6Ϊ����ģʽ 
  P0INP &= ~0x40;  //��P0.6��������  
  
  P1SEL &= ~0x80;  //����P1.7Ϊ��ͨIO      P17Ϊ����źŲɼ�
  P1DIR &= ~0x80;  //������P1.7���ϣ���P1.7Ϊ����ģʽ 
  P1INP |= 0x80;  //��P1.7��������,���ˣ�Ӧ��������Ϊ3̬��Ϊ����״̬��
  
  P2SEL &= ~0x1;  //����P2.0Ϊ��ͨIO      P20ΪDS18B20�źŲɼ�
  P2DIR &= ~0x1;  //������P2.0���ϣ���P2.0Ϊ����ģʽ 
  P2INP &= ~0x1;  //��P2.0��������
  
  P1DIR |= 0x40;   // P1.6����Ϊ���        P16�ǿ����Ƿ����  
  P1DIR |= 0x0e;   // P1.1��P1.2��P1.3����Ϊ���        P11,P12,P13����ɫ�ƵĿ�����  
  P1DIR |= 0x01;   // P1.0����Ϊ���        P10�ǿ��Ƽ��ȵ�  
  P0DIR |= 0x80;   // P0.7����Ϊ���        P07�ǿ����𶯵���Ƿ��𶯵�
  P0_7=0;
  P0DIR |= 0x30;   // P0.4,P0.5����Ϊ�����P05��PWM  115KHZ�����ߣ�P04�Ǹ��������ߣ�P04�ɲ��ã�
  
}
/****************************************************************************
* ��    ��: TEMP_CMP_SET()
* ��    ��: �ж��Ƿ�����¶����ý���
* ��ڲ���: ��
* ���ڲ���: 
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
             Turnon_main_picture();//�ж�����Ѿ������ý��棬���������˳����ý��������棬��������¶����ý���
             KEY_SET_MARK=0;
           }
           else
           {
             Turnon_set_picture();
             KEY_SET_MARK=1;
           }
           while((!KEY_ADD)&&(!KEY_DEC));
           KeyScan_finsh=1;
                                  //���ý����¶����ñ�־λ,����־λΪON,��OFF       
          */
        }
    }
}
/****************************************************************************
* ��    ��: ShutDown()
* ��    ��: �ػ�
* ��ڲ���: ��
* ���ڲ���: 0Ϊ̧��   1Ϊ��������
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
* ��    ��: KeyScan()
* ��    ��: ��ȡ����״̬�����ư���ָʾ��
* ��ڲ���: ��
* ���ڲ���: 0Ϊ̧��   1Ϊ��������
****************************************************************************/
uchar KeyScan(void)
{
    if (KEY_POWER == 0)         
    {
        Delay_ms(10);      //��ʱ10MSȥ��
        if (KEY_POWER == 0)
        {
            while(!KEY_POWER); //���ּ��
               if(KEY_POWER_MARK==1)
               {
                 ShutDown();
                 
               }
               else
               {
                 RoomTemperature(); //����ǰ�ȼ�������
                 HOT=ON;
                 Turn_on_picture();
                 KEY_POWER_MARK=1;
                 
               } 
                  
         }
            return 1;     //�а�������
        }
    
    
    if (KEY_POWER_MARK&&(KEY_ADD == 0))
    {
        Delay_ms(10);      //��ʱ10MSȥ��
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
            OLED_ShowNum(0,2,TEMP_CMP,4,16);  //�����ý�����趨�¶Ƚ��е���������ʾ������
            Delay_ms(300);
        }
        return 3;
    }
    
    
    if (KEY_POWER_MARK&&(KEY_DEC == 0))
    {
        Delay_ms(10);      //��ʱ10MSȥ��
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
        Delay_ms(10);      //��ʱ10MSȥ��
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
            OLED_ShowNum(0,2,TEMP_CMP,4,16);  //�����ý�����趨�¶Ƚ��е���������ʾ������
        }
        return 3;
    }
    
    
    if ((KEY_SET_MARK==1) &&(KEY_DEC == 0))
    {
        Delay_ms(10);      //��ʱ10MSȥ��
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
        Delay_ms(10);      //��ʱ10MSȥ��
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
              MOTOR=ON;//������
            }
              MOTOR=OFF;
        }
        return 5;
     }
    */
    
    return 0;             //�ް�������
}
/****************************************************************************
* ��    ��: Turn_on_picture()
* ��    ��: ������ʾ����
* ��ڲ���: ��
* ���ڲ���: ��
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
          OLED_ShowCHinese(40,2,0);//��ʾ��
}

/****************************************************************************
* ��    ��: Turnon_main_picture()
* ��    ��: ��������ʾ����
* ��ڲ���: ��
* ���ڲ���: ��
****************************************************************************/
void Turnon_main_picture(void)
{
          OLED_Clear();
          OLED_DrawBMP(0,0,64,2,BMP4);	
          OLED_ShowCHinese(8,2,13);//������
          OLED_ShowCHinese(24,2,14);//
          OLED_ShowCHinese(40,2,15);//
          delay_ms(2000);
          OLED_DrawBMP(0,2,64,4,BMP0);
          OLED_ShowCHinese(40,2,0);//��ʾ��
}

/****************************************************************************
* ��    ��: Turnon_set_picture()
* ��    ��: ����������ʾ����
* ��ڲ���: ��
* ���ڲ���: ��
****************************************************************************/
void Turnon_set_picture(void)
{
          OLED_Clear();
          OLED_ShowCHinese(0,0,16);
          OLED_ShowCHinese(16,0,17);
          OLED_ShowCHinese(40,2,0);//��ʾ��
          OLED_ShowNum(0,2,TEMP_CMP,4,16);
}
/****************************************************************************
* ��    ��: Turn_off_picture()
* ��    ��: �ػ���ʾ����
* ��ڲ���: ��
* ���ڲ���: ��
****************************************************************************/
void Turn_off_picture(void)
{
          OLED_Clear();
          OLED_DrawBMP(0,0,64,4,BMP2);	
          delay_ms(1000);
          OLED_Clear();
}

/****************************************************************************
* ��    ��: InitCLK()
* ��    ��: ����ϵͳʱ��Դ
* ��ڲ���: ��
* ���ڲ���: ��
****************************************************************************/
void InitCLK()
{
    CLKCONCMD &= ~0x40;             //����ϵͳʱ��ԴΪ32MHZ����
    while(CLKCONSTA & 0x40);        //�ȴ������ȶ�Ϊ32M
    CLKCONCMD &= ~0x47;             //����ϵͳ��ʱ��Ƶ��Ϊ32MHZ   
}

/****************************************************************************
* ��    ��: Heating_control()
* ��    ��: ���¿���
* ��ڲ���: ��
* ���ڲ���: ��
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
  //���¶ȴﵽ�򳬹��趨ֵʱ��ֹͣ����
  //���¶ȵ���һ��ƫ��ֵʱ���������ȣ��¶�ƫ��ֵ�ú궨��
}
/****************************************************************************
* ��    ��: Temperature_conversion_shows()
* ��    ��: �¶�ת����ʾ
* ��ڲ���: ��
* ���ڲ���: ��
****************************************************************************/
void Temperature_conversion_shows(void)
{
         if(KEY_POWER_MARK)//&&(KEY_SET_MARK==0))
          {
          Surface_temperature_conversion();
          }
}
/****************************************************************************
* ��    ��: battery_charging()
* ��    ��: ��س��
* ��ڲ���: ��
* ���ڲ���: ��
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

  //���رռ��ȵ�ʱ��������(�����ػ���ʱ�������磩,�������ʱ�رճ��
}
/****************************************************************************
* ��    ��: GetVoltage()
* ��    ��: AD ֵ
* ��ڲ���: ��
* ���ڲ���: ͨ�����㷵��ʵ�ʵ��¶�ֵ
****************************************************************************/
/*
float GetVoltage(void)
{ 
   unsigned int  value; 
   unsigned char VolADCCON3 = ADCCON3;
   
   ADCIF = 0;   
   
   //ѡ��1.25VΪ�ο���ѹ��12λ�ֱ��ʣ���Ƭ���¶ȴ���������
   ADCCON3  = (HAL_ADC_REF_1V25 | ADC_12_BIT | ADC_EMP_SENS);            
   
   while(!ADCIF);                     //�ȴ� AD ת����� 
   value =  ADCL >> 2;                //ADCL �Ĵ����� 2 λ��Ч 
   value |= (((unsigned int)ADCH) << 6);

   ADCCON3 = VolADCCON3;

   return ADC_TO_CELSIUS(value);                               
}
*/
/******************************************************************************
*�� �� ����Read_advalue
*��    �ܣ�ADC��ص�ѹ
*��ڲ������ο���ѹ reference��ת��ͨ�� channel���ֱ���resolution
*���ڲ�����ADCת�����
******************************************************************************/
uint Read_advalue(uchar reference, uchar channel, uchar  resolution)
{
   uint value; 
   uchar tmpADCCON3 = ADCCON3;
   
   APCFG |= 1 << channel ; //����ADC����ͨ��,ģ��I/Oʹ��
/*
   //APCFG (0xF2) �C Analog Peripheral I/O Configuration
Bit Name Reset R/W Description
7:0 APCFG[7:0]
//0x00 R/W Analog Perpheral I/O configuration . APCFG[7:0] select P0.7�CP0.0 as 
//analog I/O.
0: Analog I/O disabled
1: Analog I/O enabled
//ʹ��ģ��IO�ڣ�����ʹ��ADC�ɼ����Ű�
*/
   ADCCON3  = (reference | resolution | channel);
   ADCIF = 0;                         //
 
   while(!ADCIF);               //�ȴ� AD ת����� 
   value =  ADCL >> 2;          //ADCL �Ĵ����� 2 λ��Ч
   value |= ((uint)ADCH << 6);  //����ADת�������λ�͵�λ 
   
 //���ݷֱ��ʻ��ADCת�������Чλ 
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
*�� �� ����
*��    �ܣ�UART��ʼ��
*��ڲ�����
*���ڲ�����
******************************************************************************/
void InitUART(void)
{
  PERCFG &= ~0x01;     //USART0Ϊλ��1
  P2DIR &= ~0xc0;      //���ȼ�USART0 >USART1 >��ʱ��1
  P0SEL |= 0x0c;       //P0_2,P0_3 �������ڣ��ⲿ�豸���ܣ�
  U0CSR |= 0x80;       //����ΪUART ��ʽ
  U0CSR |= 0x40;       //ʹ�ܽ�����
  U0UCR = 0x02;        //��ֹ�����ƣ�8bits,��У��λ��1λֹͣλ
                       //��ʼλ�͵�ƽ��ֹͣλ�ߵ�ƽ
  U0GCR |=8;          //32MHz �µ�BAUD_E:11, 115200    8    9600
  U0BAUD |= 59;       //32MHz �µ�BAUD_M:216 115200    59   9600
  UTX0IF = 0;          //UART0 TX �жϱ�־��ʼ��λ0
  IEN0 |= 0x04;        //ʹ��USART0 RX�ж�
  EA = 1;              //�����ж�
}
  
/******************************************************************************
*�� �� ����UART0_ISR
*��    �ܣ�����0�жϷ������
*��ڲ�����
*���ڲ�����
******************************************************************************/
#pragma vector = URX0_VECTOR 
__interrupt void UART0_ISR(void) 
{
  URX0IF = 0;           //UART0 RX�жϱ�־λ��0
  temp = U0DBUF;        //��ȡU0DBUF��ֵ
}

/****************************************************************************
* ��    ��: UartSendString()
* ��    ��: ���ڷ��ͺ���
* ��ڲ���: Data:���ͻ�����   len:���ͳ���
* ���ڲ���: ��
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
* ��    ��: PowerAD()
* ��    ��: ADת������ص�ѹ����0.1V���ֱ�
* ��ڲ���: 
* ���ڲ���: 
****************************************************************************/
uint PowerAD(unsigned char Pnn)
   {

     uint vddvalue;       //ADCת��ֵ 
 //ADC�ο���ѹAVDD5���ŵ�Դ��ѹ��3.3V���ֱ���12λ���ɼ�ͨ����VDD/3,VDD=3.3V
     vddvalue = Read_advalue(HAL_ADC_REF_1V25, Pnn , ADC_12_BIT);  //0x0FΪVDD/3��01ΪP01  04ΪP04  HAL_ADC_REF_1V25  ADC_REF_AVDD5
  //   vddvalue = (vddvalue*125) >> 11;   //�������һ�����⣬������������ټ���3.3V���ȣ�����USB��ѹ���ԣ�����﮵�ص�ѹ���ԡ������������ʱ��ͦ�õ�
  //   vddvalue = vddvalue*4;
  //  vddvalue = (vddvalue*33)/2048;
#if 0
     buf[0] = vddvalue/10 + '0';
     buf[1] = '.';
     buf[2] =vddvalue%10 + '0';
     buf[3] =0x0;
     UartSendString(buf,strlen(buf));  //�����ϴ�����VDDֵ   //����������⣬strlen��������ĳ��ȱ�ʵ�ʳ��ȶ�һλ��
                                         //�ҵ������ˣ�Ӧ�ý����ַ��Ĵ洢λ�÷ŵ���������棬�������������һ����0�����Խ���������
                                         //֮ǰ������buf[3]����3���������ڸ�Ϊbuf[6]������λbuf[3] =0x0;
     ucTemp=vddvalue;
     OLED_ShowNum(0,2,ucTemp,4,16);
     delay_ms(2000);                  //ÿ��2s�ϴ�һ��ֵ
 #endif
     return vddvalue;
   }
   

/****************************************************************************
* ��    ��: BattleCycle()
* ��    ��: С���ͼ��ѭ����ʾ
* ��ڲ���: 
* ���ڲ���: 
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
  if(P1_7==1)OLED_DrawBMP(47,0,64,1,BMP_Battle_817Finish);  //�������ʱ��ʾС������ͼ��
}

/****************************************************************************
* ��    ��: Error(void) 
* ��    ��: ̽��N�κ����¶�û�б仯,Ҳ�������¶��趨ֵ,����ʾ:������?����ά��
* ��ڲ���: 
* ���ڲ���: 
****************************************************************************/
void Error(void)     
{
  if(ucTemp_on==ucTemp)   //����һ���¶�ucTemp_on�Ƚϣ�����ͬ�Ҳ�����Ŀ���¶������������N�κ�����Ϊ����ʧ�顣
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
      OLED_ShowCHinese(0,0,18);//��
      OLED_ShowCHinese(16,0,19);//��
      OLED_ShowCHinese(32,0,20);//��
      OLED_ShowCHinese(48,0,21);//��
      OLED_ShowCHinese(0,2,22);//��
      OLED_ShowCHinese(16,2,23);//��
      OLED_ShowCHinese(32,2,24);//ά
      OLED_ShowCHinese(48,2,25);//��
      delay_ms(1000);
    }
  }
}

/****************************************************************************
* ��    ��: PWM_Hot()
* ��    ��: PWM�����¶�����
* ��ڲ���: 
* ���ڲ���: 
****************************************************************************/
void PWM_Hot(void)
{
  uint i,j;
  if(KEY_POWER_MARK==1)
  { 
    if(PwmVariable>=255)PwmVariable=255;  //���������������
    
    else 
    {
  //  PwmVariable=PwmVariable*2;
    PwmVariable++;
    }
//    OLED_ShowNum(0,2,PwmVariable,3,16);
    PWM_Set(PwmVariable,0);//����0��׼��������һ��IO�ģ��ȱ���
    PWM_Pulse();
    delay_ms(10);
    i=VoltageMean(PnLi);
    test1=i;
    j=VoltageMean(PnUsb);
    AD_AbnormalTermination;  //��Ϊ��ʵ�ʲ���ʱ���ֵ�ѹ���գ��޵�ѹ״̬ʱ�ɼ������Ľ������4000���Ǵ���ģ�����ʱ��������취���������
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
      PowerLow=1;   //��ʾ��ѹ���㣬׼���ػ���
      PwmVariable=0;
    }
    else PowerLow=0;
    
  }
}
/****************************************************************************
* ��    ��: PowerLow_Shutdown()
* ��    ��: ��ع����ѹ���Զ��ػ�
* ��ڲ���: 
* ���ڲ���: 
****************************************************************************/
void PowerLow_Shutdown(void)
{
  if(PowerLow==1)
  {
    PWM_Set(0,0);  //������Ϊ0,�����Զ�û�磬����ʾ����Ļ��ᶼû����
    PWM_Pulse();
    P0_7=1;                        //��BIһ��
    OLED_DrawBMP(0,0,64,4,BMP_Please);   //��ѹ���޸�Ϊ���磡
    for(char i=0;i<50;i++)
    {
            //OLED_DrawBMP(0,0,64,4,BMP_Please);//OLED_DrawBMP(0,2,64,4,PowerLow_1664);   ��ѹ���޸�Ϊ���磡
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
    PowerLow=0; //�����־λ
  }
}

/****************************************************************************
* ��    ��: Heating_intensity_icon()
* ��    ��: ����ǿ��ͼ��
* ��ڲ���: 
* ���ڲ���: 
****************************************************************************/
void Heating_intensity_icon(void)
{
#if 1
  if(KEY_POWER_MARK==1)                        //Ŀǰ��ʾ�����趨��ǿ��
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
* ��    ��: LoopCheck()
* ��    ��: ѭ���м��
* ��ڲ���: 
* ���ڲ���: 
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
* ��    ��: Charge_complete()
* ��    ��: �����ɴ���
* ��ڲ���: 
* ���ڲ���: 
****************************************************************************/
void Charge_complete(void)
{
  if(P1_7==1)
    {
      delay_ms(10);
      if(P1_7==1)  //ȷ�ϵ��Ѿ�����������ͼ����ʾ��Ȼ��BIһ��
      {
        OLED_DrawBMP(0,0,64,4,BMP_Battle_6432Finish);
        P0_7=1;
        delay_ms(1000);
        P0_7=0;
  //      OLED_Clear();    //�Ȳ������Ļ
        Battle_6432Finish=1;
      }
    }
  
}
/****************************************************************************
* ��    ��: PowerOffCharging()
* ��    ��: �ػ����
* ��ڲ���: 
* ���ڲ���: 
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
      UsbPowerElectricize=0; //USB���״̬��־
      LoopClearOver=0;       //USB���ѭ��ͼ���Ƿ�ִ�й�������Ͽ�����ѹ��Ҫ����ִ�У�����ִ��
      Battle_6432Finish=0;   //�������־����һ��
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
      if(P1_7==1)  //ȷ�ϵ��Ѿ�����������ͼ����ʾ��Ȼ��BIһ��
      {
        OLED_DrawBMP(0,0,64,4,BMP_Battle_6432Finish);
        P0_7=1;
        delay_ms(1000);
        P0_7=0;
  //      OLED_Clear();    //�Ȳ������Ļ
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
 // LoopClearOver=1;//�����ٴ����ȴ����һ������
#endif
}
/****************************************************************************
* ��    ��: RoomTemperature()
* ��    ��: ���������¶�
* ��ڲ���: ��
* ���ڲ���: ��
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
    
  //�ϵ�ǰ����ʮ�Σ���ƽ��ֵ��ÿ�β��Լ��1MS
  //����¶ȳ���30�ȣ���Ӧ��֮ǰ��һ�β����µ�ֵ������Ĭ������25�ȡ�
}

/****************************************************************************
* ��    ��: unsigned char TemperatureMean(void)
* ��    ��: ����ƽ��ֵ
* ��ڲ���: ��
* ���ڲ���: ��
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
* ��    ��: Surface_temperature_conversion()
* ��    ��: ʵ���¶�������¶ȵ�ת������
* ��ڲ���: ��
* ���ڲ���: ��
****************************************************************************/
void Surface_temperature_conversion(void)
{
  int i=0;
   i=(char)DS18B20_ReadMain();
  if(i<=RoomTemperatureValue)
  {
    ucTemp=RoomTemperatureValue;//�����õ��¶�С�����£�����ʾ����

  }
  else
  {
    ucTemp=thermometer[i-1];

  }
    OLED_DrawBMP(0,2,16,4,BMP0F81); //��ʱ����������������������Ż���
    OLED_ShowNum(0,2,ucTemp,4,16);
    delay_ms(10);
#if 1                        //С�����˵ľ�������!
    if(ucTemp>=TEMP_CMP_TipBurn)high_temperature_count++;
    if(high_temperature_count>=TEMP_CMP_TipBurn_count)  //һ�����������ʾ����.
    {
     high_temperature_count=0;
     OLED_DrawBMP(0,2,64,4,BMP_Becareful);
     delay_ms(500);
     OLED_DrawBMP(0,2,64,4,BMP0F41);  //����°ಿ������,����ʾ��ǰֵ�͡�
     OLED_ShowNum(0,2,ucTemp,4,16);
     OLED_ShowCHinese(40,2,0);//��ʾ��
    }
    
#endif    
} 



/****************************************************************************
* ��    ��: IO_Reset()
* ��    ��: IO�ڸ�λ����
* ��ڲ���: Data:���ͻ�����   len:���ͳ���
* ���ڲ���: ��
****************************************************************************/
void IO_Reset(void)
{
  P1_0=0; //���ȿ��ƶ˿ڣ��ϵ���и�λ��
//  P1_7=0; //�������⣬�ϵ���и�λ�������á�
  P1_6=1; //���ʹ�ܿ��ƶˣ��ϵ���ߵ�ƽ����ֱ�Ӹ�ֵ���������ö˿ں��ֵ��
}

/****************************************************************************
* ��    ��: main()
* ��    ��: �������
* ��ڲ���: ��
* ���ڲ���: ��
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
                //����һ���������IO�ڵĺ���

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
               //��Ҫ�Ѷ˿������óɸ���̬����Ȼ������ʵ��Ӧ�������á������س��ʱĪ��LED���������д����
//     P1_0=0;  
     P0_7=0;
    I2CCFG = 0;             
    I2CWC |= 0x83;
    I2CIO |=0x0;
    OLED_Init();	
    OLED_Clear(); 

   InitUART();          //UART0���ڳ�ʼ��
 //  PIDInit ( &spid ); // Initialize Structure 
   
   battery_control=OFF;  //�����س��
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
    PWM_Set(PwmVariable,0);//����0��׼��������һ��IO�ģ��ȱ���
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
    OLED_ShowCHinese(8,2,13);//������
    OLED_ShowCHinese(24,2,14);//
    OLED_ShowCHinese(40,2,15);//
    OLED_DrawBMP(0,4,64,4,line);	
    delay_ms(2000);
    OLED_DrawBMP(0,2,64,4,Hoting);	
    delay_ms(2000);
    //OLED_DrawBMP(0,2,64,4,BMP0);
    OLED_ShowCHinese(40,2,0);//��ʾ��
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
            PowerLow_Shutdown();//��ѹ�����Զ��ػ����ػ�ǰ��ʾ�����BIһ��PowerLow_Shutdown
            if(KEY_POWER_MARK)Error();
            if(!KEY_POWER_MARK)PowerOffCharging();
 //           battery_charging();  //��س�������ʱ���Σ��õ�Դ����һ�߳��һ��ʹ�á�
          }	  
	
  }
          
        //ucTemp = ReadDs18B20();           //�¶ȶ�ȡ����
        //strTemp[0] = ucTemp/10+16;        //ȡ��ʮλ��
        //strTemp[1] = ucTemp%10+16;        //ȡ����λ��
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
          
		OLED_ShowCHinese(8,0,0);//��
		OLED_ShowCHinese(24,0,1);//��
		OLED_ShowCHinese(40,0,2);//԰
		OLED_ShowCHinese(8,2,7);//��
		OLED_ShowCHinese(40,2,8);//��
	  	delay_ms(1000);
			OLED_Clear();
		OLED_DrawBMP(0,0,64,4,BMP1);	
		delay_ms(1000);
			delay_ms(1000);
	  	delay_ms(1000);
			 OLED_Clear();

          
                OLED_ShowCHinese(0,0,9);//��
		OLED_ShowCHinese(16,0,10);//��
		OLED_ShowCHinese(32,0,11);//��
		OLED_ShowCHinese(48,0,12);//��
                    */
#endif