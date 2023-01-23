#include "adc.h"
#include "delay.h"
#include "oled.h"

u16 ADC1_value[4];

/**************************************************************************
 * ����  ����ADC_ADC1_init
 * �������ܣ�ADC1��ʼ��,ADC1ͨ��1\2\3\4�������ɨ��ת��ģʽ
 * ��ڲ�������
 * ����  ֵ���� 
**************************************************************************/
void ADC_ADC1_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��GPIOCʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);	//ʹ��ADC1ʱ��
	
	GPIO_InitStructure.GPIO_Pin = ADC1_Channel_1;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;  //IO��ģʽ����Ϊģ������
	GPIO_Init(ADC1_Channel_1_Port, &GPIO_InitStructure);	//ͨ��1��ʼ��
	
	GPIO_InitStructure.GPIO_Pin = ADC1_Channel_2;
	GPIO_Init(ADC1_Channel_2_Port, &GPIO_InitStructure);	//ͨ��2��ʼ��
	
	GPIO_InitStructure.GPIO_Pin = ADC1_Channel_3;
	GPIO_Init(ADC1_Channel_3_Port, &GPIO_InitStructure);	//ͨ��3��ʼ��
	
	GPIO_InitStructure.GPIO_Pin = ADC1_Channel_4;
	GPIO_Init(ADC1_Channel_4_Port, &GPIO_InitStructure);	//ͨ��4��ʼ��
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M
	ADC_DeInit(ADC1);  //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//ģ��ת�������ڵ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//ģ��ת������������ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 4;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ��� 
	
	ADC_Cmd(ADC1, ENABLE);	//ʹ��ָ����ADC1
	
	ADC_ResetCalibration(ADC1);	//ʹ�ܸ�λУ׼   
	while(ADC_GetResetCalibrationStatus(ADC1));	//�ȴ���λУ׼����
	
	ADC_StartCalibration(ADC1);	 //����ADУ׼
	while(ADC_GetCalibrationStatus(ADC1));	 //�ȴ�У׼����
	
	ADC_DMA1Channel1_init();
	//����ͨ����ת��˳���Լ�ת��ʱ��
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5);	//ADC1,ADCͨ��1��һ��ת��,����ʱ��Ϊ239.5����
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_239Cycles5);	//ADC1,ADCͨ��2�ڶ���ת��,����ʱ��Ϊ239.5����
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_239Cycles5);	//ADC1,ADCͨ��3������ת��,����ʱ��Ϊ239.5����
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 4, ADC_SampleTime_239Cycles5);	//ADC1,ADCͨ��4���ĸ�ת��,����ʱ��Ϊ239.5����
	
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);	//����ADC1ת��
}

/**************************************************************************
 * ����  ����ADC_DMA1Channel1_init
 * �������ܣ�ADC1��DMAͨ����ʼ��,�����赽�ڴ棬��ADCת�������ݴ����ADC1_value[]
 * ��ڲ�������
 * ����  ֵ���� 
**************************************************************************/
void ADC_DMA1Channel1_init(void)
{
	DMA_InitTypeDef DMA_InitStruct;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	
	DMA_InitStruct.DMA_BufferSize=4;								//DMA����������Ŀ
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralSRC;					//DMA���� ������Դ
	DMA_InitStruct.DMA_M2M=DMA_M2M_Disable;							//�ر��ڴ浽�ڴ�
	DMA_InitStruct.DMA_MemoryBaseAddr=(uint32_t)ADC1_value;			//�ڴ����ݵ�ַ
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;	//�ڴ����ݴ�С ����
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;				//�ڴ��ַ�Լ�
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;						//DMAģʽ��ѭ��ģʽ
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&(ADC1->DR);	//�����ַ
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;	//�������ݴ�С ����
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;		//�����ַ���Լ�
	DMA_InitStruct.DMA_Priority=DMA_Priority_High;					//DMA���ȼ�
	DMA_Init(DMA1_Channel1, &DMA_InitStruct);						//����DMA1ͨ��1
	
	ADC_DMACmd(ADC1,ENABLE);										//ʹ��ADC1��DMA
	DMA_Cmd(DMA1_Channel1,ENABLE);									//ʹ��DMA1ͨ��1
}

/**************************************************************************
 * ����  ����ADC_Throttle_Protection_delay
 * �������ܣ����Ž�����ʱ����3s
 * ��ڲ������Ƿ��һ�ε���(pro_bit)
 * ����  ֵ��0 -> ��ʱ���	���� -> ��ʱδ��� 
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
 * ����  ����ADC_Throttle_Protection
 * �������ܣ����ű�����ʹ��������Ҫ�ȸ���������������룬�������Ų������á�
 * ��ڲ��������Ž�����־(*state)
 * ����  ֵ����
**************************************************************************/
void ADC_Throttle_Protection(u8 *state)
{
	u8 adc_pro = 1;
	
	if(*state == 0)
	{	
		while(ADC_Throttle_Protection_delay(adc_pro))
		{
			adc_pro = 0;
			
			if(ADC1_value[0] < 10)	//�ж��ǲ�����С����
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
