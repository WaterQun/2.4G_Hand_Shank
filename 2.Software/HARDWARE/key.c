#include "key.h"

/**************************************************************************
 * 函数  名：KEY_init
 * 函数功能：按键模块初始化
 * 入口参数：无
 * 返回  值：无 
**************************************************************************/
void KEY_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC , ENABLE); //使能PC端口时钟
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE); 	 // JTAG失能
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_8|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_15;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;      //上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);		//配置GPIOA的按键端口
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;      //上拉输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);		//配置GPIOA的按键端口
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;      //上拉输入
	GPIO_Init(GPIOC, &GPIO_InitStructure);		//配置GPIOA的按键端口
}




