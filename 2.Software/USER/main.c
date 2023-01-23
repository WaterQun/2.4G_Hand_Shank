#include "delay.h"	
#include "oled.h"
#include "nrf.h"
#include "led.h"
#include "key.h"
#include "adc.h"

void Hardware_Initialize(void);	//���������ʼ������

int main()
{
	u8 throttle_mode = 0;	//����ģʽ��0��������1������
	u8 throttle_h = 0,throttle_l = 0;
	//����ϵͳ�ж����ȼ�����2
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//��ʼ������
	Hardware_Initialize();
	//�����ʾ
	OLED_Clear();
	OLED_Display_Init_String();	//��ʾ��ʼ�ַ�
	//������ѭ��
	while(1)
	{
		#if DISPLAY_ADC_VALUE	//�ж��Ƿ���ʾ�ĸ�ͨ����ADCֵ	
			OLED_Display_Adc_channel_Value();
		#endif
		
		ADC_Throttle_Protection(&throttle_mode);	//���Ž���
		Nrf_Check_Event();	//����Ƿ���յ�����
		if(KEY_BB == 0)		//����B+����
		{
			delay_ms(20);
			if(KEY_BB == 0)
				tx_buf[0] = 1;	//��������ź�
			while(!KEY_BB);
		}
		else tx_buf[0] = 0;
		
		if(KEY_B == 0)		//����B����
		{
			delay_ms(20);
			if(KEY_B == 0)
				tx_buf[1] = 1;	//���ֹͣ�ź�
			while(!KEY_B);
		}
		else tx_buf[1] = 0;
		
		/* �Է��͵����ݽ��д��� */
		if(throttle_mode == 1)
		{
			if(ADC1_value[0] > 4000)
				ADC1_value[0] = 4000;
			throttle_h = (u8)((limAdc(ADC1_value[0])/10)/256);
			throttle_l = (u8)((limAdc(ADC1_value[0])/10)%256);
		}
		tx_buf[2] = throttle_h;		//����ֵ
		tx_buf[3] = throttle_l;
		tx_buf[4] = (u8)(limAdc(ADC1_value[2])/100);	//ǰ��ƫ��
		tx_buf[5] = (u8)(limAdc(ADC1_value[3])/100);	//����ƫ��
		
		if(KEY_AA == 0)		//����A+����
		{
			delay_ms(20);
			if(KEY_AA == 0)
				tx_buf[6] = 1;	//����ģʽ�����ź�
			while(!KEY_AA);
		}
		else tx_buf[6] = 0;
		
		if(KEY_A == 0)		//����A����
		{
			delay_ms(20);
			if(KEY_A == 0)
				tx_buf[7] = 1;	//����ģʽֹͣ�ź�
			while(!KEY_A);
		}
		else tx_buf[7] = 0;
		
		if(KEY_DOWN == 0)		//����DOWN����
		{
			delay_ms(20);
			if(KEY_DOWN == 0)
				tx_buf[8] = 1;	//����ģʽ�����ź�
			while(!KEY_DOWN);
		}
		else tx_buf[8] = 0;
		
		if(KEY_UP == 0)		//����UP����
		{
			delay_ms(20);
			if(KEY_UP == 0)
				tx_buf[9] = 1;	//����ģʽ�ر��ź�
			while(!KEY_UP);
		}
		else tx_buf[9] = 0;
		
		/*��������*/
		Nrf_TxPacket(tx_buf,32);	//����һ������
		OLED_Display_Euler_Angle();
		OLED_Display_Power_Volt();
		OLED_Display_Pid_Frequency();
		OLED_Refresh_Gram();
	}
}

/**************************************************************************
 * ��������Hardware_Initialize
 * �������ܣ������ʼ������
 * ��ڲ�������
 * ����  ֵ���� 
**************************************************************************/
void Hardware_Initialize(void)
{
	delay_init();		//��ʱ������ʼ��
	LED_init();			//LED�Ƴ�ʼ��
	KEY_init();			//������ʼ��
	OLED_init();		//OLED��ʼ��
	ADC_ADC1_init();	//ADC1��ʼ��
	Nrf24l01_init();	//��ʼ��nrf24l01
}
