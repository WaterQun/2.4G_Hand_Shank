#include "oled.h"
#include "oledfont.h" 
//OLED的显存
//存放格式如下.
//[0]0 1 2 3 ... 127	
//[1]0 1 2 3 ... 127	
//[2]0 1 2 3 ... 127	
//[3]0 1 2 3 ... 127	
//[4]0 1 2 3 ... 127	
//[5]0 1 2 3 ... 127	
//[6]0 1 2 3 ... 127	
//[7]0 1 2 3 ... 127 		   
u8 OLED_GRAM[128][8];

static void IIC_GPIO_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能PC端口时钟
	
	GPIO_InitStructure.GPIO_Pin = OLED_SDA_PIN|OLED_SCL_PIN;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	
	GPIO_Init(OLED_GPIO, &GPIO_InitStructure);
}

//更新显存到LCD		 
void OLED_Refresh_Gram(void)
{
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Com (0xb0+i);    //设置页地址（0~7）
		OLED_WR_Com (0x00);      //设置显示位置—列低地址
		OLED_WR_Com (0x10);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)OLED_WR_Date(OLED_GRAM[n][i]); 
	}   
}

void OLED_IIC_Start(void)
{
	OLED_IIC_SDA=1;
	OLED_IIC_SCL=1;
	delay_us(1);
 	OLED_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(1);
	OLED_IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}

void  OLED_IIC_Stop(void)
{

	OLED_IIC_SCL=0;
	OLED_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(1);
	OLED_IIC_SCL=1; 
	OLED_IIC_SDA=1;//发送I2C总线结束信号
	delay_us(1);							   	
}

void Write_IIC_Byte(unsigned char IIC_Byte)
{
	unsigned char i;
	for(i=0;i<8;i++)
	{
		if(IIC_Byte & 0x80)
			OLED_IIC_SDA=1;
		else
			OLED_IIC_SDA=0;
		OLED_IIC_SCL=1;
		delay_us(1);  //必须有保持SCL脉冲的延时
		OLED_IIC_SCL=0;
		IIC_Byte<<=1;
	}
		OLED_IIC_SDA = 1;//原程序这里有一个拉高SDA，根据OLED的DATASHEET，此句必须去掉。
		OLED_IIC_SCL=1;
		delay_us(1);
		OLED_IIC_SCL=0;
}

void OLED_WR_Date(unsigned char IIC_Data)
{
	OLED_IIC_Start();
	Write_IIC_Byte(0x78);
	Write_IIC_Byte(0x40);			//write data
	Write_IIC_Byte(IIC_Data);
	OLED_IIC_Stop();
}

void OLED_WR_Com(unsigned char IIC_Command)
{
	OLED_IIC_Start();
	Write_IIC_Byte(0x78);            //Slave address,SA0=0
	Write_IIC_Byte(0x00);			//write command
	Write_IIC_Byte(IIC_Command);
	OLED_IIC_Stop();
}

//开启OLED显示    
void OLED_Display_On(void)
{
	OLED_WR_Com(0X8D);  //SET DCDC命令
	OLED_WR_Com(0X14);  //DCDC ON
	OLED_WR_Com(0XAF);  //DISPLAY ON
}

//关闭OLED显示     
void OLED_Display_Off(void)
{
	OLED_WR_Com(0X8D);  //SET DCDC命令
	OLED_WR_Com(0X10);  //DCDC OFF
	OLED_WR_Com(0XAE);  //DISPLAY OFF
}

//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!	  
void OLED_Clear(void)  
{  
	u8 i,n;  
	for(i=0;i<8;i++)for(n=0;n<128;n++)OLED_GRAM[n][i]=0X00;  
	OLED_Refresh_Gram();//更新显示
}

//画点 
//x:0~127
//y:0~63
//t:1 填充 0,清空				   
void OLED_DrawPoint(u8 x,u8 y,u8 t)
{
	u8 pos,bx,temp=0;
	if(x>127||y>63)return;//超出范围了.
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;	    
}

//x1,y1,x2,y2 填充区域的对角坐标
//确保x1<=x2;y1<=y2 0<=x1<=127 0<=y1<=63	 	 
//dot:0,清空;1,填充	  
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot)  
{  
	u8 x,y;  
	for(x=x1;x<=x2;x++)
	{
		for(y=y1;y<=y2;y++)OLED_DrawPoint(x,y,dot);
	}													    
	OLED_Refresh_Gram();//更新显示
}

//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示				 
//size:选择字体 16/12 
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode)
{      			    
	u8 temp,t,t1;
	u8 y0=y;
	u8 csize=(size/8+((size%8)?1:0))*(size/2);		//得到字体一个字符对应点阵集所占的字节数
	chr=chr-' ';//得到偏移后的值		 
    for(t=0;t<csize;t++)
    {   
		if(size==12)temp=asc2_1206[chr][t]; 	 	//调用1206字体
		else if(size==16)temp=asc2_1608[chr][t];	//调用1608字体
		else if(size==24)temp=asc2_2412[chr][t];	//调用2412字体
		else return;								//没有的字库
        for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp<<=1;
			y++;
			if((y-y0)==size)
			{
				y=y0;
				x++;
				break;
			}
		}  	 
    }          
}

//m^n函数
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}

//显示2个数字
//x,y :起点坐标	 
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);	 		  
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ',size,1);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0',size,1); 
	}
} 

//显示字符串
//x,y:起点坐标  
//size:字体大小 
//*p:字符串起始地址 
void OLED_ShowString(u8 x,u8 y,const u8 *p,u8 size)
{	
    while((*p<='~')&&(*p>=' '))//判断是不是非法字符!
    {       
        if(x>(128-(size/2))){x=0;y+=size;}
        if(y>(64-size)){y=x=0;OLED_Clear();}
        OLED_ShowChar(x,y,*p,size,1);	 
        x+=size/2;
        p++;
    }  
	
}

//初始化SSD1306	
void OLED_init(void)
{
	IIC_GPIO_init();
	
	delay_ms(500);
	
	OLED_WR_Com(0xAE); //关闭显示
	OLED_WR_Com(0xD5); //设置时钟分频因子,震荡频率
	OLED_WR_Com(80);   //[3:0],分频因子;[7:4],震荡频率
	OLED_WR_Com(0xA8); //设置驱动路数
	OLED_WR_Com(0X3F); //默认0X3F(1/64) 
	OLED_WR_Com(0xD3); //设置显示偏移
	OLED_WR_Com(0X00); //默认为0

	OLED_WR_Com(0x40); //设置显示开始行 [5:0],行数.
													    
	OLED_WR_Com(0x8D); //电荷泵设置
	OLED_WR_Com(0x14); //bit2，开启/关闭
	OLED_WR_Com(0x20); //设置内存地址模式
	OLED_WR_Com(0x02); //[1:0],00，列地址模式;01，行地址模式;10,页地址模式;默认10;
	OLED_WR_Com(0xA1); //段重定义设置,bit0:0,0->0;1,0->127;
	OLED_WR_Com(0xC0); //设置COM扫描方向;bit3:0,普通模式;1,重定义模式 COM[N-1]->COM0;N:驱动路数
	OLED_WR_Com(0xDA); //设置COM硬件引脚配置
	OLED_WR_Com(0x12); //[5:4]配置
		 
	OLED_WR_Com(0x81); //对比度设置
	OLED_WR_Com(0xEF); //1~255;默认0X7F (亮度设置,越大越亮)
	OLED_WR_Com(0xD9); //设置预充电周期
	OLED_WR_Com(0xf1); //[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Com(0xDB); //设置VCOMH 电压倍率
	OLED_WR_Com(0x30); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	OLED_WR_Com(0xA4); //全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏)
	OLED_WR_Com(0xA6); //设置显示方式;bit0:1,反相显示;0,正常显示	    						   
	OLED_WR_Com(0xAF); //开启显示	
	
	OLED_Clear();
}

/**************************************************************************
 * 函数  名：OLED_Display_Init_String
 * 函数功能：显示初始字符
 * 入口参数：无
 * 返回  值：无
**************************************************************************/
void OLED_Display_Init_String(void)
{
	OLED_ShowString(0,0 ,"pitch:",12);
	OLED_ShowString(0,12," roll:",12);
	OLED_ShowString(0,24," yaw:",12);
	OLED_ShowString(0,36,"thro_lock:",12);
	OLED_ShowString(112,36,"on",12);
	OLED_ShowString(0,50,"Power_v:",12);
	OLED_ShowString(114,50,"Hz",12);
}

#if DISPLAY_ADC_VALUE==1	//判断是否定义DISPLAY_ADC_VALUE

/*
 *	如果定义DISPLAY_ADC_VALUE为1则定义以下函数
 *	OLED_Display_Adc_channel_Value(void)
 */

#include "adc.h"

//去掉ADC1检测数据的个位
#define limAdc(x)  (x/10*10)

/*
 * 函数名：OLED_Display_Adc_channel_Value
 * 功  能：通过OLED显示四路ADC通道转换的值
 * 返回值：无
 */
void OLED_Display_Adc_channel_Value(void)
{
	OLED_ShowNum(0, 0,limAdc(ADC1_value[0]),4,16);
	OLED_ShowNum(0,16,limAdc(ADC1_value[1]),4,16);
	OLED_ShowNum(0,32,limAdc(ADC1_value[2]),4,16);
	OLED_ShowNum(0,48,limAdc(ADC1_value[3]),4,16);
	OLED_Refresh_Gram();
}

#endif

#include "nrf.h"

/**************************************************************************
 * 函数  名：OLED_Display_Euler_Angle
 * 函数功能：显示接收到的欧拉角的数据
 * 入口参数：无
 * 返回  值：无
**************************************************************************/
void OLED_Display_Euler_Angle(void)
{
	short pitch_v = To_int(rx_buf[2],rx_buf[3]);
	short roll_v = To_int(rx_buf[4],rx_buf[5]);
	short yaw_v = To_int(rx_buf[6],rx_buf[7]);
	//OLED显示pitch角度
	if(pitch_v < 0)
	{
		OLED_ShowChar(36,0,'-',12,1);
		pitch_v = -pitch_v;
	}
	else
		OLED_ShowChar(36,0,' ',12,1);
	OLED_ShowNum(42,0,pitch_v/100,3,12);
	OLED_ShowChar(60,0,'.',12,1);
	OLED_ShowNum(66,0,pitch_v%100,2,12);
	//OLED显示roll角度
	if(roll_v < 0)
	{
		OLED_ShowChar(36,12,'-',12,1);
		roll_v = -roll_v;
	}
	else
		OLED_ShowChar(36,12,' ',12,1);
	OLED_ShowNum(42,12,roll_v/100,3,12);
	OLED_ShowChar(60,12,'.',12,1);
	OLED_ShowNum(66,12,roll_v%100,2,12);
	//OLED显示yaw角度
	if(yaw_v < 0)
	{
		OLED_ShowChar(36,24,'-',12,1);
		yaw_v = -yaw_v;
	}
	else
		OLED_ShowChar(36,24,' ',12,1);
	OLED_ShowNum(42,24,yaw_v/100,3,12);
	OLED_ShowChar(60,24,'.',12,1);
	OLED_ShowNum(66,24,yaw_v%100,2,12);
}

/**************************************************************************
 * 函数  名：OLED_Display_Power_Volt
 * 函数功能：显示飞机反馈的电压数据
 * 入口参数：无
 * 返回  值：无
**************************************************************************/
void OLED_Display_Power_Volt(void)
{
	short volt_v;
	float Power_Volt = 0;
	static float Power_last = 0;
	
	volt_v = To_int(rx_buf[0],rx_buf[1]);	//获取反馈的电压数值
	
	Power_Volt = volt_v*0.0091;		//转换为模拟电压
	
	if(Power_last != 0)
		Power_Volt = 0.15*Power_Volt + 0.85*Power_last;	//一阶低通滤波
	
	Power_last = Power_Volt;

	if(Power_Volt != 0)
		Power_Volt = Power_Volt + 0.82;		//线性拟合
	
	volt_v = (uint16_t)(Power_Volt*100);
	
	OLED_ShowNum(48,50,volt_v/100,2,12);
	OLED_ShowChar(60,50,'.',12,1);
	OLED_ShowNum(66,50,volt_v%100,2,12);
}

/**************************************************************************
 * 函数  名：OLED_Display_Pid_Frequency
 * 函数功能：显示PID运行频率
 * 入口参数：无
 * 返回  值：无
**************************************************************************/
void OLED_Display_Pid_Frequency(void)
{
	OLED_ShowNum(96,50,rx_buf[8],3,12);
}
