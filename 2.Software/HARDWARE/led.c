#include "led.h"

 /**************************************************************************
 * ����  ����LED_init
 * �������ܣ���ʼ��LED
 * ��ڲ�������
 * ����  ֵ���� 
**************************************************************************/
void LED_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 	//ʹ��PC�˿�ʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;	//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      	//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     	//50M
	
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//����GPIOB8��GPIOB9
	
	GPIO_SetBits(GPIOB,GPIO_Pin_4|GPIO_Pin_5);				//�ø�B8��B9
}

/**************************************************************************
 * ����  ����Light_User_LED
 * �������ܣ�����LED״̬
 * ��ڲ�����User_Led���ͱ������������LED
 * ����  ֵ���� 
**************************************************************************/
void Light_User_LED(User_Led state)
{
	if(state == OnlyLED0)		//LED0������LED1Ϩ��
	{
		LED0 = 0;
		LED1 = 1;
	}
	else if(state == OnlyLED1)	//LED0Ϩ��LED1����
	{
		LED0 = 1;
		LED1 = 0;
	}
	else if(state == BothOfThem)//LED0��LED1������
	{
		LED0 = 0;
		LED1 = 0;
	}
	else if(state == BothOff)	//LED0��LED1��Ϩ��
	{
		LED0 = 1;
		LED1 = 1;
	}
}


