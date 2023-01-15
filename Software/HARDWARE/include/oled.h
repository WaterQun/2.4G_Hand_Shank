#ifndef _OLED_H
#define _OLED_H
#include <sys.h>
#include "delay.h"

//#define OLED_CMD  0	//д����
//#define OLED_DATA 1	//д����
//--GPIO�ض���
#define OLED_GPIO GPIOA	//�ض���OLEDʹ�õ�GPIO�ӿ�
#define OLED_SDA_PIN GPIO_Pin_7 //OLED����������
#define OLED_SCL_PIN GPIO_Pin_5 //OLEDʱ��������
//--IO��������ض���
#define OLED_IIC_SCL  PAout(5) //SCL   ����ʱ��
#define OLED_IIC_SDA  PAout(7) //SDA   ��������

//ȥ��ADC1������ݵĸ�λ
#define limAdc(x)  (x/10*10)

/*
 * �궨����ʾ����
 */
#define DISPLAY_ADC_VALUE 0	//ͨ��OLED��ʾADת��ֵ

//OLED�����ú���
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


