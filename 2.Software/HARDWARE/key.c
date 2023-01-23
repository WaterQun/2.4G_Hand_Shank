#include "key.h"

/**************************************************************************
 * ����  ����KEY_init
 * �������ܣ�����ģ���ʼ��
 * ��ڲ�������
 * ����  ֵ���� 
**************************************************************************/
void KEY_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC , ENABLE); //ʹ��PC�˿�ʱ��
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE); 	 // JTAGʧ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_8|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_15;	//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;      //��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);		//����GPIOA�İ����˿�
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;	//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;      //��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);		//����GPIOA�İ����˿�
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;	//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;      //��������
	GPIO_Init(GPIOC, &GPIO_InitStructure);		//����GPIOA�İ����˿�
}




