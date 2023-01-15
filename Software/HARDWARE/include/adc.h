/*
 * adc.h
 *
 * Created on: 2022年04月26日
 *     Author: @假人
 */
#ifndef _ADC_H
#define _ADC_H

#include "sys.h"

/*
 * 重定义ADC1通道输出接口
 */
#define ADC1_Channel_1 		GPIO_Pin_1	
#define ADC1_Channel_1_Port	GPIOA

#define ADC1_Channel_2 		GPIO_Pin_2	
#define ADC1_Channel_2_Port	GPIOA

#define ADC1_Channel_3 		GPIO_Pin_3	
#define ADC1_Channel_3_Port	GPIOA

#define ADC1_Channel_4 		GPIO_Pin_6	
#define ADC1_Channel_4_Port	GPIOA

//声明全局变量
extern u16 ADC1_value[4];

//声明ADC相关函数
void ADC_ADC1_init(void);			//初始化ADC1
void ADC_DMA1Channel1_init(void);	//初始化ADC1的DMA通道
void ADC_Throttle_Protection(u8 *state);	//油门保护

#endif




