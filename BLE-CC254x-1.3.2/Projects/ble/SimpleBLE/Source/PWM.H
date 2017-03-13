#ifndef PWM_H
#define PWM_H
#ifndef U8
typedef unsigned char U8;  
#endif
#ifndef U16
typedef unsigned short U16;  
#endif
extern void PWM_Init(void);  
extern void PWM_Set(U8 nPWM_P10, U8 nPWM_P11);  
extern void PWM_Get(U8 *nPWM_P10, U8 *nPWM_P11);  
extern void PWM_Pulse(void);  
#endif