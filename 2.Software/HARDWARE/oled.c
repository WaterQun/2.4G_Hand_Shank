#include "oled.h"
#include "oledfont.h" 
//OLED���Դ�
//��Ÿ�ʽ����.
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
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //ʹ��PC�˿�ʱ��
	
	GPIO_InitStructure.GPIO_Pin = OLED_SDA_PIN|OLED_SCL_PIN;	//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	
	GPIO_Init(OLED_GPIO, &GPIO_InitStructure);
}

//�����Դ浽LCD		 
void OLED_Refresh_Gram(void)
{
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Com (0xb0+i);    //����ҳ��ַ��0~7��
		OLED_WR_Com (0x00);      //������ʾλ�á��е͵�ַ
		OLED_WR_Com (0x10);      //������ʾλ�á��иߵ�ַ   
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
	OLED_IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}

void  OLED_IIC_Stop(void)
{

	OLED_IIC_SCL=0;
	OLED_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(1);
	OLED_IIC_SCL=1; 
	OLED_IIC_SDA=1;//����I2C���߽����ź�
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
		delay_us(1);  //�����б���SCL�������ʱ
		OLED_IIC_SCL=0;
		IIC_Byte<<=1;
	}
		OLED_IIC_SDA = 1;//ԭ����������һ������SDA������OLED��DATASHEET���˾����ȥ����
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

//����OLED��ʾ    
void OLED_Display_On(void)
{
	OLED_WR_Com(0X8D);  //SET DCDC����
	OLED_WR_Com(0X14);  //DCDC ON
	OLED_WR_Com(0XAF);  //DISPLAY ON
}

//�ر�OLED��ʾ     
void OLED_Display_Off(void)
{
	OLED_WR_Com(0X8D);  //SET DCDC����
	OLED_WR_Com(0X10);  //DCDC OFF
	OLED_WR_Com(0XAE);  //DISPLAY OFF
}

//��������,������,������Ļ�Ǻ�ɫ��!��û����һ��!!!	  
void OLED_Clear(void)  
{  
	u8 i,n;  
	for(i=0;i<8;i++)for(n=0;n<128;n++)OLED_GRAM[n][i]=0X00;  
	OLED_Refresh_Gram();//������ʾ
}

//���� 
//x:0~127
//y:0~63
//t:1 ��� 0,���				   
void OLED_DrawPoint(u8 x,u8 y,u8 t)
{
	u8 pos,bx,temp=0;
	if(x>127||y>63)return;//������Χ��.
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;	    
}

//x1,y1,x2,y2 �������ĶԽ�����
//ȷ��x1<=x2;y1<=y2 0<=x1<=127 0<=y1<=63	 	 
//dot:0,���;1,���	  
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot)  
{  
	u8 x,y;  
	for(x=x1;x<=x2;x++)
	{
		for(y=y1;y<=y2;y++)OLED_DrawPoint(x,y,dot);
	}													    
	OLED_Refresh_Gram();//������ʾ
}

//��ָ��λ����ʾһ���ַ�,���������ַ�
//x:0~127
//y:0~63
//mode:0,������ʾ;1,������ʾ				 
//size:ѡ������ 16/12 
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode)
{      			    
	u8 temp,t,t1;
	u8 y0=y;
	u8 csize=(size/8+((size%8)?1:0))*(size/2);		//�õ�����һ���ַ���Ӧ������ռ���ֽ���
	chr=chr-' ';//�õ�ƫ�ƺ��ֵ		 
    for(t=0;t<csize;t++)
    {   
		if(size==12)temp=asc2_1206[chr][t]; 	 	//����1206����
		else if(size==16)temp=asc2_1608[chr][t];	//����1608����
		else if(size==24)temp=asc2_2412[chr][t];	//����2412����
		else return;								//û�е��ֿ�
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

//m^n����
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}

//��ʾ2������
//x,y :�������	 
//len :���ֵ�λ��
//size:�����С
//mode:ģʽ	0,���ģʽ;1,����ģʽ
//num:��ֵ(0~4294967295);	 		  
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

//��ʾ�ַ���
//x,y:�������  
//size:�����С 
//*p:�ַ�����ʼ��ַ 
void OLED_ShowString(u8 x,u8 y,const u8 *p,u8 size)
{	
    while((*p<='~')&&(*p>=' '))//�ж��ǲ��ǷǷ��ַ�!
    {       
        if(x>(128-(size/2))){x=0;y+=size;}
        if(y>(64-size)){y=x=0;OLED_Clear();}
        OLED_ShowChar(x,y,*p,size,1);	 
        x+=size/2;
        p++;
    }  
	
}

//��ʼ��SSD1306	
void OLED_init(void)
{
	IIC_GPIO_init();
	
	delay_ms(500);
	
	OLED_WR_Com(0xAE); //�ر���ʾ
	OLED_WR_Com(0xD5); //����ʱ�ӷ�Ƶ����,��Ƶ��
	OLED_WR_Com(80);   //[3:0],��Ƶ����;[7:4],��Ƶ��
	OLED_WR_Com(0xA8); //��������·��
	OLED_WR_Com(0X3F); //Ĭ��0X3F(1/64) 
	OLED_WR_Com(0xD3); //������ʾƫ��
	OLED_WR_Com(0X00); //Ĭ��Ϊ0

	OLED_WR_Com(0x40); //������ʾ��ʼ�� [5:0],����.
													    
	OLED_WR_Com(0x8D); //��ɱ�����
	OLED_WR_Com(0x14); //bit2������/�ر�
	OLED_WR_Com(0x20); //�����ڴ��ַģʽ
	OLED_WR_Com(0x02); //[1:0],00���е�ַģʽ;01���е�ַģʽ;10,ҳ��ַģʽ;Ĭ��10;
	OLED_WR_Com(0xA1); //���ض�������,bit0:0,0->0;1,0->127;
	OLED_WR_Com(0xC0); //����COMɨ�跽��;bit3:0,��ͨģʽ;1,�ض���ģʽ COM[N-1]->COM0;N:����·��
	OLED_WR_Com(0xDA); //����COMӲ����������
	OLED_WR_Com(0x12); //[5:4]����
		 
	OLED_WR_Com(0x81); //�Աȶ�����
	OLED_WR_Com(0xEF); //1~255;Ĭ��0X7F (��������,Խ��Խ��)
	OLED_WR_Com(0xD9); //����Ԥ�������
	OLED_WR_Com(0xf1); //[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Com(0xDB); //����VCOMH ��ѹ����
	OLED_WR_Com(0x30); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	OLED_WR_Com(0xA4); //ȫ����ʾ����;bit0:1,����;0,�ر�;(����/����)
	OLED_WR_Com(0xA6); //������ʾ��ʽ;bit0:1,������ʾ;0,������ʾ	    						   
	OLED_WR_Com(0xAF); //������ʾ	
	
	OLED_Clear();
}

/**************************************************************************
 * ����  ����OLED_Display_Init_String
 * �������ܣ���ʾ��ʼ�ַ�
 * ��ڲ�������
 * ����  ֵ����
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

#if DISPLAY_ADC_VALUE==1	//�ж��Ƿ���DISPLAY_ADC_VALUE

/*
 *	�������DISPLAY_ADC_VALUEΪ1�������º���
 *	OLED_Display_Adc_channel_Value(void)
 */

#include "adc.h"

//ȥ��ADC1������ݵĸ�λ
#define limAdc(x)  (x/10*10)

/*
 * ��������OLED_Display_Adc_channel_Value
 * ��  �ܣ�ͨ��OLED��ʾ��·ADCͨ��ת����ֵ
 * ����ֵ����
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
 * ����  ����OLED_Display_Euler_Angle
 * �������ܣ���ʾ���յ���ŷ���ǵ�����
 * ��ڲ�������
 * ����  ֵ����
**************************************************************************/
void OLED_Display_Euler_Angle(void)
{
	short pitch_v = To_int(rx_buf[2],rx_buf[3]);
	short roll_v = To_int(rx_buf[4],rx_buf[5]);
	short yaw_v = To_int(rx_buf[6],rx_buf[7]);
	//OLED��ʾpitch�Ƕ�
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
	//OLED��ʾroll�Ƕ�
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
	//OLED��ʾyaw�Ƕ�
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
 * ����  ����OLED_Display_Power_Volt
 * �������ܣ���ʾ�ɻ������ĵ�ѹ����
 * ��ڲ�������
 * ����  ֵ����
**************************************************************************/
void OLED_Display_Power_Volt(void)
{
	short volt_v;
	float Power_Volt = 0;
	static float Power_last = 0;
	
	volt_v = To_int(rx_buf[0],rx_buf[1]);	//��ȡ�����ĵ�ѹ��ֵ
	
	Power_Volt = volt_v*0.0091;		//ת��Ϊģ���ѹ
	
	if(Power_last != 0)
		Power_Volt = 0.15*Power_Volt + 0.85*Power_last;	//һ�׵�ͨ�˲�
	
	Power_last = Power_Volt;

	if(Power_Volt != 0)
		Power_Volt = Power_Volt + 0.82;		//�������
	
	volt_v = (uint16_t)(Power_Volt*100);
	
	OLED_ShowNum(48,50,volt_v/100,2,12);
	OLED_ShowChar(60,50,'.',12,1);
	OLED_ShowNum(66,50,volt_v%100,2,12);
}

/**************************************************************************
 * ����  ����OLED_Display_Pid_Frequency
 * �������ܣ���ʾPID����Ƶ��
 * ��ڲ�������
 * ����  ֵ����
**************************************************************************/
void OLED_Display_Pid_Frequency(void)
{
	OLED_ShowNum(96,50,rx_buf[8],3,12);
}
