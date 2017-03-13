#include "ioCC2540.h"
#include "PWM.h"  
#ifndef BV  
#define BV(n)      (1 << (n))  
#endif  
static U8 sPWM_P10 = 0;    //P10的pwm  
//static U8 sPWM_P11 = 0;    //P11的pwm  
//******************************************************************************    
//name:             PWM_Init    
//introduce:        PWM的初始化   
//******************************************************************************  
void PWM_Init(void)    
{     
  P1DIR |= BV(0);               //P10定义为输出   
  P1SEL |= BV(0);         //P10设置为外设功能;   
  T4CTL = 0x00;                 //1分频（32M/256=125K）、关timer、自由运行模式  T4CTL = 0x00;
  T4CC0 = 255 - sPWM_P10;       //P10的初始化值  
  //T4CC1 = 255 - sPWM_P11;       //P11的初始化值  
  T4CCTL0 = 0x2C;               //00 101 100无中断、Set output on compare, clear on 0xFF、比较模式、No Capture  
  //T4CCTL1 = 0x2C;               //00 101 100无中断、Set output on compare, clear on 0xFF、比较模式、No Capture    
  T4CTL |= BV(4);               //开始定时器    
}    
//******************************************************************************    
//name:             PWM_SetLed    
//introduce:        PWM的两通道值设置  
//parameter:        nPWM_P10：p10的pwm值   
//                      nPWM_P11：p11的pwm值   
//******************************************************************************  
void PWM_Set(U8 nPWM_P10, U8 nPWM_P11)    
{    
  sPWM_P10 = nPWM_P10;    
 // sPWM_P11 = nPWM_P11;    
}    
//******************************************************************************    
//name:             PWM_GetLed    
//introduce:        PWM的两通道值读取  
//parameter:        nPWM_P10：p10的pwm值   
//                      nPWM_P11：p11的pwm值   
//******************************************************************************  
void PWM_Get(U8 *nPWM_P10, U8 *nPWM_P11)    
{    
  *nPWM_P10 = sPWM_P10;    
  //*nPWM_P11 = sPWM_P11;    
}    
//******************************************************************************    
//name:             PWM_Pulse    
//introduce:        PWM的值更新  
//******************************************************************************  
void PWM_Pulse(void)    
{    
//重新配置PWM值  
  T4CC0 = 255 - sPWM_P10;                 
//  T4CC1 = 255 - sPWM_P11;                 
//复位timer4的counter  
  T4CTL &= ~BV(2);      
//使能定时器4  
  T4CTL |= BV(4);     
}   

/*上述代码中，T4CC0的赋值之所以需要255放在前面减，是因为上面的配置是先低电平后高电平，与实际需要的先高后低的占空比相反了。
因此取补后即可得到正确的占空比。*/