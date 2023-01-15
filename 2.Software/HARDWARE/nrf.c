#include "nrf.h"
#include "oled.h"

const u8 TX_ADDRESS[TX_ADR_WIDTH]={0x66,0x66,0x66,0x66,0x66};	//发送地址
const u8 RX_ADDRESS[RX_ADR_WIDTH]={0x66,0x66,0x66,0x66,0x66};	//接收地址

u8 rx_buf[RX_PLOAD_WIDTH];	//定义接收数据数组
u8 tx_buf[TX_PLOAD_WIDTH];	//定义发送数据数组

//初始化SPI2，用于NRF的通信
void Nrf_SPI2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//使能GPIOB的端口时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 , ENABLE);	//使能SPI2时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  	//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//输出速度
	GPIO_Init(GPIOB, &GPIO_InitStructure);				//初始化PB相应引脚
	
	SPI_Cmd(SPI2, DISABLE);	//关闭SPI2

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//选择了串行时钟的稳态:空闲时钟低
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//数据捕获于第一个时钟沿
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
	SPI_Init(SPI2, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
	
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8);	//设置SPI2通信波特率
	
	SPI_Cmd(SPI2, ENABLE); //使能SPI外设
	
//	SPI1_ReadWriteByte(0xff);//启动传输		
}

//SPI 速度设置函数
//SpeedSet:
//SPI_BaudRatePrescaler_2   2分频   (SPI 36M@sys 72M)
//SPI_BaudRatePrescaler_8   8分频   (SPI 9M@sys 72M)
//SPI_BaudRatePrescaler_16  16分频  (SPI 4.5M@sys 72M)
//SPI_BaudRatePrescaler_256 256分频 (SPI 281.25K@sys 72M)
void SPI2_SetSpeed(u8 SpeedSet)
{
	//SPI_InitStructure.SPI_BaudRatePrescaler = SpeedSet ;
  	//SPI_Init(SPI1, &SPI_InitStructure);
//	SPI_Cmd(SPI1,ENABLE);
	SPI2->CR1&=0XFFC7; 
	SPI2->CR1|=SpeedSet;	//设置SPI速度  
	SPI2->CR1|=1<<6; 		//SPI设备使能 
} 

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI2_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
	{
		retry++;
		if(retry>200)return 0;
	}			  
	SPI_I2S_SendData(SPI2, TxData); //通过外设SPIx发送一个数据
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)//检查指定的SPI标志位设置与否:接受缓存非空标志位
	{
		retry++;
		if(retry>200)return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPI2); //返回通过SPIx最近接收的数据					    
}

//NRF所用到IO口初始化
void Nrf_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = Nrf_CS_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //输出速度50M
	GPIO_Init(Nrf_CS_Port, &GPIO_InitStructure); //初始化IO口
	
	GPIO_InitStructure.GPIO_Pin = Nrf_CE_Pin;
	GPIO_Init(Nrf_CE_Port, &GPIO_InitStructure); //初始化IO口
	
	GPIO_InitStructure.GPIO_Pin = Nrf_IRQ_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //推挽输出
	GPIO_Init(Nrf_IRQ_Port, &GPIO_InitStructure); //初始化IO口
}

//检测24L01是否存在
//返回值:0，成功;1，失败	
u8 Nrf_Check(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 i; 
	Nrf_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	Nrf_Read_Buf(TX_ADDR,buf,5); //读出写入的地址  
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 1;//检测24L01错误	
	return 0;		 //检测到24L01
}

//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
u8 Nrf_Write_Reg(u8 reg,u8 value)
{
	u8 status;	
   	NRF24L01_CSN=0;                 //使能SPI传输
  	status =Nrf_SPI_ReadWriteByte(reg);//发送寄存器号 
  	Nrf_SPI_ReadWriteByte(value);      //写入寄存器的值
  	NRF24L01_CSN=1;                 //禁止SPI传输	   
  	return(status);       			//返回状态值
}

//读取SPI寄存器值
//reg:要读的寄存器
u8 Nrf_Read_Reg(u8 reg)
{
	u8 reg_val;	    
 	NRF24L01_CSN = 0;          //使能SPI传输		
  	Nrf_SPI_ReadWriteByte(reg);   //发送寄存器号
  	reg_val=Nrf_SPI_ReadWriteByte(0XFF);//读取寄存器内容
  	NRF24L01_CSN = 1;          //禁止SPI传输		    
  	return(reg_val);           //返回状态值
}

//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值 
u8 Nrf_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	u8 status,u8_ctr;	       
  	NRF24L01_CSN = 0;           //使能SPI传输
  	status=Nrf_SPI_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)pBuf[u8_ctr]=Nrf_SPI_ReadWriteByte(0XFF);//读出数据
  	NRF24L01_CSN=1;       //关闭SPI传输
  	return status;        //返回读到的状态值
}

//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
u8 Nrf_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;	    
 	NRF24L01_CSN = 0;          //使能SPI传输
  	status = Nrf_SPI_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)Nrf_SPI_ReadWriteByte(*pBuf++); //写入数据	 
  	NRF24L01_CSN = 1;       //关闭SPI传输
  	return status;          //返回读到的状态值
}	

//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
void Nrf_TxPacket_AP(u8 *pBuf, u8 len)		//自动应答时使用
{
	NRF24L01_CE=0;	
	Nrf_Write_Buf(0xa8, pBuf, len); 			 // 装载数据
	NRF24L01_CE=1;		 //置高CE
}

//初始化Nrf24l01
//根据定义初始化Nrf为相应的模式
void Nrf24l01_init()
{
	Nrf_SPI2_Init();	//SPI2初始化
	Nrf_GPIO_Init();	//Nrf所用IO初始化
	
	while(Nrf_Check())	//检查nrf是否到位
	{
		OLED_ShowString(0,0,"Not find Nrf",16);	
		OLED_Refresh_Gram();
	}
	//作为发送端时检测需要在模式配置之前
	
	NRF24L01_CE = 0;

    Nrf_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//载入接收地址
    Nrf_Write_Buf(NRF_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);//载入发送地址
    Nrf_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);		//开启通道0自动应答
    Nrf_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);	//使能通道0接收地址
    Nrf_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);	//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
    Nrf_Write_Reg(NRF_WRITE_REG+RF_CH,40);			//设置RF通信频率
    Nrf_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);		//设置TX发射参数,0db增益,2Mbps,低噪声增益开启 
#ifdef TX_MODE_NOACK	//设置为发射模式，不带自动应答	
    Nrf_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);	//设置通道0接收数据宽度
    Nrf_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);
	#endif
	
#ifdef RX_MODE_NOACK	//设置为接收模式，不带自动应答
    Nrf_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);	//设置通道0接收数据宽度
    Nrf_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);			//配置基本工作模式
	#endif
	
#ifdef TX_MODE_ACK		//设置Nrf为发送模式，带自动应答
    Nrf_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);         // IRQ收发完成中断开启,16位CRC,主发送
    Nrf_Write_Reg(FLUSH_TX,0xff);		//清除发送缓冲区
    Nrf_Write_Reg(FLUSH_RX,0xff);		//清除接收缓冲区
    Nrf_SPI_ReadWriteByte(0x50);
    Nrf_SPI_ReadWriteByte(0x73);
    Nrf_Write_Reg(NRF_WRITE_REG+0x1c,0x01);	//使能通道0的动态载荷长度
    Nrf_Write_Reg(NRF_WRITE_REG+0x1d,0x07);	//启用动态载荷长度，允许载荷ACK
	#endif
	
#ifdef RX_MODE_ACK		//设置Nrf为接收模式，带自动应答
    Nrf_Write_Reg(FLUSH_TX,0xff);		//清除发送缓冲区
    Nrf_Write_Reg(FLUSH_RX,0xff);		//清除接收缓冲区
    Nrf_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);         // IRQ收发完成中断开启,16位CRC,主接收
    Nrf_SPI_ReadWriteByte(0x50);
    Nrf_SPI_ReadWriteByte(0x73);
    Nrf_Write_Reg(NRF_WRITE_REG+0x1c,0x01);	//使能通道0的动态载荷长度
    Nrf_Write_Reg(NRF_WRITE_REG+0x1d,0x07); //启用动态载荷长度，允许载荷ACK
	#endif
	
    NRF24L01_CE = 1;	//拉高CE引脚，开启发送
	NRF24L01_CSN = 0;	//SPI3片选引脚拉低
}

//检查Nrf是否接收到数据
void Nrf_Check_Event(void)
{
    u8 sta = Nrf_Read_Reg(NRF_READ_REG + STATUS);
    if(sta & (1 << RX_DR))
    {
        Nrf_Read_Buf(RD_RX_PLOAD,rx_buf,RX_PLOAD_WIDTH);//将接收到的数据存入rx_buf
		OLED_ShowString(120,0,"1",12);
		OLED_Refresh_Gram();
    }
    else
	{
        Nrf_Write_Reg(FLUSH_RX,0xff);					//清除缓存
		OLED_ShowString(120,0,"2",12);
		OLED_Refresh_Gram();
	}
    Nrf_Write_Reg(NRF_WRITE_REG + STATUS, sta);
}

//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
void Nrf_TxPacket(uint8_t * pBuf, uint8_t len)		//带ACK应答数据包时用
{
	NRF24L01_CE=0;		 //StandBy I模式		
	Nrf_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (u8*)RX_ADDRESS, TX_ADR_WIDTH); // 装载接收端地址
	Nrf_Write_Buf(WR_TX_PLOAD, pBuf, len); 			 // 装载数据	
	NRF24L01_CE=1;		 //置高CE，激发数据发送
}

/*
 * 函数名：Nrf_Clear_Fifo
 * 参  数：*buf -> 要清空的数据地址		len -> 清空数据长度
 * 返回值：无
 */
void Nrf_Clear_Fifo(u8 *buf, u8 len)
{
	u8 i;
	for(i=0; i<len; i++)
		buf[i] = 0;
}





