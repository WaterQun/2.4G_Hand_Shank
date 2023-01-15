#ifndef _OLED_H
#define _OLED_H
#include <sys.h>
#include "delay.h"

//#define OLED_CMD  0	//写命令
//#define OLED_DATA 1	//写数据
//--GPIO重定义
#define OLED_GPIO GPIOA	//重定义OLED使用的GPIO接口
#define OLED_SDA_PIN GPIO_Pin_7 //OLED数据线引脚
#define OLED_SCL_PIN GPIO_Pin_5 //OLED时钟线引脚
//--IO输入输出重定义
#define OLED_IIC_SCL  PAout(5) //SCL   串行时钟
#define OLED_IIC_SDA  PAout(7) //SDA   串行数据

//去掉ADC1检测数据的个位
#define limAdc(x)  (x/10*10)

/*
 * 宏定义显示配置
 */
#define DISPLAY_ADC_VALUE 0	//通过OLED显示AD转换值

//OLED控制用函数
static void IIC_GPIO_init(void);
void OLED_Refresh_Gram(void);
void OLED_IIC_Start(void);
void OLED_IIC_Stop(void);
void Write_IIC_Byte(unsigned char IIC_Byte);
void OLED_WR_Date(unsigned char IIC_Data);
void OLED_WR_Com(unsigned char IIC_Command);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
u32 mypow(u8 m,u8 n);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y,const u8 *p,u8 size);
void OLED_init(void);

void OLED_Display_Init_String(void);
void OLED_Display_Adc_channel_Value(void);
void OLED_Display_Euler_Angle(void);
void OLED_Display_Power_Volt(void);
void OLED_Display_Pid_Frequency(void);

#endif


