/*
 * adc.h
 *
 * Created on: 2022��04��26��
 *     Author: @����
 */
#ifndef _ADC_H
#define _ADC_H

#include "sys.h"

/*
 * �ض���ADC1ͨ������ӿ�
 */
#define ADC1_Channel_1 		GPIO_Pin_1	
#define ADC1_Channel_1_Port	GPIOA

#define ADC1_Channel_2 		GPIO_Pin_2	
#define ADC1_Channel_2_Port	GPIOA

#define ADC1_Channel_3 		GPIO_Pin_3	
#define ADC1_Channel_3_Port	GPIOA

#define ADC1_Channel_4 		GPIO_Pin_6	
#define ADC1_Channel_4_Port	GPIOA

//����ȫ�ֱ���
extern u16 ADC1_value[4];

//����ADC��غ���
void ADC_ADC1_init(void);			//��ʼ��ADC1
void ADC_DMA1Channel1_init(void);	//��ʼ��ADC1��DMAͨ��
void ADC_Throttle_Protection(u8 *state);	//���ű���

#endif




