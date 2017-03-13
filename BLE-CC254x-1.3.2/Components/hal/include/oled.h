#include "ioCC2540.h"
#ifndef __OLED_H
#define __OLED_H			  	 
//#include "sys.h"
//#include "stdlib.h"	  
#define  u8 unsigned char 
#define  u32 unsigned int 
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据
#define OLED_MODE 0






#define OLED_SCLK_Clr() I2CIO&=0x01;
#define OLED_SCLK_Set() I2CIO|=0x02;

#define OLED_SDIN_Clr() I2CIO&=0x02;
#define OLED_SDIN_Set() I2CIO|=0x01;

/*
#define OLED_SCLK_Clr() P1_5=0
#define OLED_SCLK_Set() P1_5=1

#define OLED_SDIN_Clr() P1_6=0
#define OLED_SDIN_Set() P1_6=1
*/

//OLED模式设置
//0:4线串行模式
//1:并行8080模式

#define SIZE 16
#define XLevelL		0x02
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	    						  
//-----------------OLED端口定义----------------  					   

void delay_ms(unsigned int ms);


//OLED控制用函数
extern void OLED_WR_Byte(unsigned dat,unsigned cmd);  
extern void OLED_Display_On(void);
extern void OLED_Display_Off(void);	   							   		    
extern void OLED_Init(void);
extern void OLED_Clear(void);
extern void OLED_DrawPoint(u8 x,u8 y,u8 t);
extern void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
extern void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size);
extern void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size);
extern void OLED_ShowString(u8 x,u8 y, u8 *p,u8 Char_Size);	 
extern void OLED_Set_Pos(unsigned char x, unsigned char y);
extern void OLED_ShowCHinese(u8 x,u8 y,u8 no);
extern void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
extern void Delay_50ms(unsigned int Del_50ms);
extern void Delay_1ms(unsigned int Del_1ms);
extern void fill_picture(unsigned char fill_Data);
extern void Picture();
extern void IIC_Start();
extern void IIC_Stop();
extern void Write_IIC_Command(unsigned char IIC_Command);
extern void Write_IIC_Data(unsigned char IIC_Data);
extern void Write_IIC_Byte(unsigned char IIC_Byte);
extern void IIC_Wait_Ack();
extern void Lcd_Pixel(unsigned char x, unsigned char y, unsigned char columnLen, unsigned int ByteCount, unsigned char BMP[]);
#endif  
	 



