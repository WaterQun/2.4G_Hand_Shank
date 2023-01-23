#include "led.h"

 /**************************************************************************
 * 函数  名：LED_init
 * 函数功能：初始化LED
 * 入口参数：无
 * 返回  值：无 
**************************************************************************/
void LED_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 	//使能PC端口时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     	//50M
	
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//配置GPIOB8和GPIOB9
	
	GPIO_SetBits(GPIOB,GPIO_Pin_4|GPIO_Pin_5);				//置高B8和B9
}

/**************************************************************************
 * 函数  名：Light_User_LED
 * 函数功能：设置LED状态
 * 入口参数：User_Led类型变量，需点亮的LED
 * 返回  值：无 
**************************************************************************/
void Light_User_LED(User_Led state)
{
	if(state == OnlyLED0)		//LED0点亮，LED1熄灭
	{
		LED0 = 0;
		LED1 = 1;
	}
	else if(state == OnlyLED1)	//LED0熄灭，LED1点亮
	{
		LED0 = 1;
		LED1 = 0;
	}
	else if(state == BothOfThem)//LED0和LED1都点亮
	{
		LED0 = 0;
		LED1 = 0;
	}
	else if(state == BothOff)	//LED0和LED1都熄灭
	{
		LED0 = 1;
		LED1 = 1;
	}
}


