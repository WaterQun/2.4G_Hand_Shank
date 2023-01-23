#include "delay.h"	
#include "oled.h"
#include "nrf.h"
#include "led.h"
#include "key.h"
#include "adc.h"

void Hardware_Initialize(void);	//声明外设初始化函数

int main()
{
	u8 throttle_mode = 0;	//油门模式，0：上锁，1：解锁
	u8 throttle_h = 0,throttle_l = 0;
	//设置系统中断优先级分组2
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//初始化外设
	Hardware_Initialize();
	//清除显示
	OLED_Clear();
	OLED_Display_Init_String();	//显示初始字符
	//进入死循环
	while(1)
	{
		#if DISPLAY_ADC_VALUE	//判断是否显示四个通道的ADC值	
			OLED_Display_Adc_channel_Value();
		#endif
		
		ADC_Throttle_Protection(&throttle_mode);	//油门解锁
		Nrf_Check_Event();	//检测是否接收到数据
		if(KEY_BB == 0)		//按键B+按下
		{
			delay_ms(20);
			if(KEY_BB == 0)
				tx_buf[0] = 1;	//电机开启信号
			while(!KEY_BB);
		}
		else tx_buf[0] = 0;
		
		if(KEY_B == 0)		//按键B按下
		{
			delay_ms(20);
			if(KEY_B == 0)
				tx_buf[1] = 1;	//电机停止信号
			while(!KEY_B);
		}
		else tx_buf[1] = 0;
		
		/* 对发送的数据进行处理 */
		if(throttle_mode == 1)
		{
			if(ADC1_value[0] > 4000)
				ADC1_value[0] = 4000;
			throttle_h = (u8)((limAdc(ADC1_value[0])/10)/256);
			throttle_l = (u8)((limAdc(ADC1_value[0])/10)%256);
		}
		tx_buf[2] = throttle_h;		//油门值
		tx_buf[3] = throttle_l;
		tx_buf[4] = (u8)(limAdc(ADC1_value[2])/100);	//前后偏移
		tx_buf[5] = (u8)(limAdc(ADC1_value[3])/100);	//左右偏移
		
		if(KEY_AA == 0)		//按键A+按下
		{
			delay_ms(20);
			if(KEY_AA == 0)
				tx_buf[6] = 1;	//定高模式开启信号
			while(!KEY_AA);
		}
		else tx_buf[6] = 0;
		
		if(KEY_A == 0)		//按键A按下
		{
			delay_ms(20);
			if(KEY_A == 0)
				tx_buf[7] = 1;	//定高模式停止信号
			while(!KEY_A);
		}
		else tx_buf[7] = 0;
		
		if(KEY_DOWN == 0)		//按键DOWN按下
		{
			delay_ms(20);
			if(KEY_DOWN == 0)
				tx_buf[8] = 1;	//降落模式开启信号
			while(!KEY_DOWN);
		}
		else tx_buf[8] = 0;
		
		if(KEY_UP == 0)		//按键UP按下
		{
			delay_ms(20);
			if(KEY_UP == 0)
				tx_buf[9] = 1;	//降落模式关闭信号
			while(!KEY_UP);
		}
		else tx_buf[9] = 0;
		
		/*发送数据*/
		Nrf_TxPacket(tx_buf,32);	//发送一次数据
		OLED_Display_Euler_Angle();
		OLED_Display_Power_Volt();
		OLED_Display_Pid_Frequency();
		OLED_Refresh_Gram();
	}
}

/**************************************************************************
 * 函数名：Hardware_Initialize
 * 函数功能：外设初始化函数
 * 入口参数：无
 * 返回  值：无 
**************************************************************************/
void Hardware_Initialize(void)
{
	delay_init();		//延时函数初始化
	LED_init();			//LED灯初始化
	KEY_init();			//按键初始化
	OLED_init();		//OLED初始化
	ADC_ADC1_init();	//ADC1初始化
	Nrf24l01_init();	//初始化nrf24l01
}
