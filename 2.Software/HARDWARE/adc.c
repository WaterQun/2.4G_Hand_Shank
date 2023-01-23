#include "adc.h"
#include "delay.h"
#include "oled.h"

u16 ADC1_value[4];

/**************************************************************************
 * 函数  名：ADC_ADC1_init
 * 函数功能：ADC1初始化,ADC1通道1\2\3\4软件启动扫描转换模式
 * 入口参数：无
 * 返回  值：无 
**************************************************************************/
void ADC_ADC1_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//使能GPIOC时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);	//使能ADC1时钟
	
	GPIO_InitStructure.GPIO_Pin = ADC1_Channel_1;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;  //IO口模式配置为模拟输入
	GPIO_Init(ADC1_Channel_1_Port, &GPIO_InitStructure);	//通道1初始化
	
	GPIO_InitStructure.GPIO_Pin = ADC1_Channel_2;
	GPIO_Init(ADC1_Channel_2_Port, &GPIO_InitStructure);	//通道2初始化
	
	GPIO_InitStructure.GPIO_Pin = ADC1_Channel_3;
	GPIO_Init(ADC1_Channel_3_Port, &GPIO_InitStructure);	//通道3初始化
	
	GPIO_InitStructure.GPIO_Pin = ADC1_Channel_4;
	GPIO_Init(ADC1_Channel_4_Port, &GPIO_InitStructure);	//通道4初始化
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M
	ADC_DeInit(ADC1);  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//模数转换工作在连续转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 4;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器 
	
	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	
	ADC_ResetCalibration(ADC1);	//使能复位校准   
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束
	
	ADC_StartCalibration(ADC1);	 //开启AD校准
	while(ADC_GetCalibrationStatus(ADC1));	 //等待校准结束
	
	ADC_DMA1Channel1_init();
	//设置通道的转换顺序以及转换时间
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5);	//ADC1,ADC通道1第一个转换,采样时间为239.5周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_239Cycles5);	//ADC1,ADC通道2第二个转换,采样时间为239.5周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_239Cycles5);	//ADC1,ADC通道3第三个转换,采样时间为239.5周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 4, ADC_SampleTime_239Cycles5);	//ADC1,ADC通道4第四个转换,采样时间为239.5周期
	
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);	//开启ADC1转换
}

/**************************************************************************
 * 函数  名：ADC_DMA1Channel1_init
 * 函数功能：ADC1的DMA通道初始化,从外设到内存，将ADC转换的数据存放至ADC1_value[]
 * 入口参数：无
 * 返回  值：无 
**************************************************************************/
void ADC_DMA1Channel1_init(void)
{
	DMA_InitTypeDef DMA_InitStruct;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	
	DMA_InitStruct.DMA_BufferSize=4;								//DMA传输数据数目
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralSRC;					//DMA方向 外设是源
	DMA_InitStruct.DMA_M2M=DMA_M2M_Disable;							//关闭内存到内存
	DMA_InitStruct.DMA_MemoryBaseAddr=(uint32_t)ADC1_value;			//内存数据地址
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;	//内存数据大小 半字
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;				//内存地址自加
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;						//DMA模式是循环模式
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&(ADC1->DR);	//外设地址
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;	//外设数据大小 半字
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;		//外设地址不自加
	DMA_InitStruct.DMA_Priority=DMA_Priority_High;					//DMA优先级
	DMA_Init(DMA1_Channel1, &DMA_InitStruct);						//配置DMA1通道1
	
	ADC_DMACmd(ADC1,ENABLE);										//使能ADC1的DMA
	DMA_Cmd(DMA1_Channel1,ENABLE);									//使能DMA1通道1
}

/**************************************************************************
 * 函数  名：ADC_Throttle_Protection_delay
 * 函数功能：油门解锁延时，共3s
 * 入口参数：是否第一次调用(pro_bit)
 * 返回  值：0 -> 延时完成	其它 -> 延时未完成 
**************************************************************************/
u8 ADC_Throttle_Protection_delay(u8 pro_bit)
{
	static short delay_i;
	
	if(pro_bit == 1 || delay_i == 0)
		delay_i = 100;
	
	for(;delay_i != 0;)
	{
		delay_i = delay_i - 1;
		delay_ms(10);
		return delay_i;
	}
	
	return 0;
}

/**************************************************************************
 * 函数  名：ADC_Throttle_Protection
 * 函数功能：油门保护，使用油门需要先给最低油门连续三秒，否则油门不起作用。
 * 入口参数：油门解锁标志(*state)
 * 返回  值：无
**************************************************************************/
void ADC_Throttle_Protection(u8 *state)
{
	u8 adc_pro = 1;
	
	if(*state == 0)
	{	
		while(ADC_Throttle_Protection_delay(adc_pro))
		{
			adc_pro = 0;
			
			if(ADC1_value[0] < 10)	//判断是不是最小油门
				*state = 1;
			else
			{
				*state = 0;
				break;
			}
		}
		if(*state == 1)
		{
			OLED_ShowString(106,36,"off",12);
			OLED_Refresh_Gram();
		}
	}
}
