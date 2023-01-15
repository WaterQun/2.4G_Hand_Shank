#include "nrf.h"
#include "oled.h"

const u8 TX_ADDRESS[TX_ADR_WIDTH]={0x66,0x66,0x66,0x66,0x66};	//���͵�ַ
const u8 RX_ADDRESS[RX_ADR_WIDTH]={0x66,0x66,0x66,0x66,0x66};	//���յ�ַ

u8 rx_buf[RX_PLOAD_WIDTH];	//���������������
u8 tx_buf[TX_PLOAD_WIDTH];	//���巢����������

//��ʼ��SPI2������NRF��ͨ��
void Nrf_SPI2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//ʹ��GPIOB�Ķ˿�ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 , ENABLE);	//ʹ��SPI2ʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  	//�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//����ٶ�
	GPIO_Init(GPIOB, &GPIO_InitStructure);				//��ʼ��PB��Ӧ����
	
	SPI_Cmd(SPI2, DISABLE);	//�ر�SPI2

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//ѡ���˴���ʱ�ӵ���̬:����ʱ�ӵ�
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//���ݲ����ڵ�һ��ʱ����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
	SPI_Init(SPI2, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
	
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8);	//����SPI2ͨ�Ų�����
	
	SPI_Cmd(SPI2, ENABLE); //ʹ��SPI����
	
//	SPI1_ReadWriteByte(0xff);//��������		
}

//SPI �ٶ����ú���
//SpeedSet:
//SPI_BaudRatePrescaler_2   2��Ƶ   (SPI 36M@sys 72M)
//SPI_BaudRatePrescaler_8   8��Ƶ   (SPI 9M@sys 72M)
//SPI_BaudRatePrescaler_16  16��Ƶ  (SPI 4.5M@sys 72M)
//SPI_BaudRatePrescaler_256 256��Ƶ (SPI 281.25K@sys 72M)
void SPI2_SetSpeed(u8 SpeedSet)
{
	//SPI_InitStructure.SPI_BaudRatePrescaler = SpeedSet ;
  	//SPI_Init(SPI1, &SPI_InitStructure);
//	SPI_Cmd(SPI1,ENABLE);
	SPI2->CR1&=0XFFC7; 
	SPI2->CR1|=SpeedSet;	//����SPI�ٶ�  
	SPI2->CR1|=1<<6; 		//SPI�豸ʹ�� 
} 

//SPIx ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI2_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
	{
		retry++;
		if(retry>200)return 0;
	}			  
	SPI_I2S_SendData(SPI2, TxData); //ͨ������SPIx����һ������
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)//���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
	{
		retry++;
		if(retry>200)return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPI2); //����ͨ��SPIx������յ�����					    
}

//NRF���õ�IO�ڳ�ʼ��
void Nrf_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = Nrf_CS_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //����ٶ�50M
	GPIO_Init(Nrf_CS_Port, &GPIO_InitStructure); //��ʼ��IO��
	
	GPIO_InitStructure.GPIO_Pin = Nrf_CE_Pin;
	GPIO_Init(Nrf_CE_Port, &GPIO_InitStructure); //��ʼ��IO��
	
	GPIO_InitStructure.GPIO_Pin = Nrf_IRQ_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //�������
	GPIO_Init(Nrf_IRQ_Port, &GPIO_InitStructure); //��ʼ��IO��
}

//���24L01�Ƿ����
//����ֵ:0���ɹ�;1��ʧ��	
u8 Nrf_Check(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 i; 
	Nrf_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
	Nrf_Read_Buf(TX_ADDR,buf,5); //����д��ĵ�ַ  
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 1;//���24L01����	
	return 0;		 //��⵽24L01
}

//SPIд�Ĵ���
//reg:ָ���Ĵ�����ַ
//value:д���ֵ
u8 Nrf_Write_Reg(u8 reg,u8 value)
{
	u8 status;	
   	NRF24L01_CSN=0;                 //ʹ��SPI����
  	status =Nrf_SPI_ReadWriteByte(reg);//���ͼĴ����� 
  	Nrf_SPI_ReadWriteByte(value);      //д��Ĵ�����ֵ
  	NRF24L01_CSN=1;                 //��ֹSPI����	   
  	return(status);       			//����״ֵ̬
}

//��ȡSPI�Ĵ���ֵ
//reg:Ҫ���ļĴ���
u8 Nrf_Read_Reg(u8 reg)
{
	u8 reg_val;	    
 	NRF24L01_CSN = 0;          //ʹ��SPI����		
  	Nrf_SPI_ReadWriteByte(reg);   //���ͼĴ�����
  	reg_val=Nrf_SPI_ReadWriteByte(0XFF);//��ȡ�Ĵ�������
  	NRF24L01_CSN = 1;          //��ֹSPI����		    
  	return(reg_val);           //����״ֵ̬
}

//��ָ��λ�ö���ָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
u8 Nrf_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	u8 status,u8_ctr;	       
  	NRF24L01_CSN = 0;           //ʹ��SPI����
  	status=Nrf_SPI_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)pBuf[u8_ctr]=Nrf_SPI_ReadWriteByte(0XFF);//��������
  	NRF24L01_CSN=1;       //�ر�SPI����
  	return status;        //���ض�����״ֵ̬
}

//��ָ��λ��дָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
u8 Nrf_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;	    
 	NRF24L01_CSN = 0;          //ʹ��SPI����
  	status = Nrf_SPI_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)Nrf_SPI_ReadWriteByte(*pBuf++); //д������	 
  	NRF24L01_CSN = 1;       //�ر�SPI����
  	return status;          //���ض�����״ֵ̬
}	

//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:�������״��
void Nrf_TxPacket_AP(u8 *pBuf, u8 len)		//�Զ�Ӧ��ʱʹ��
{
	NRF24L01_CE=0;	
	Nrf_Write_Buf(0xa8, pBuf, len); 			 // װ������
	NRF24L01_CE=1;		 //�ø�CE
}

//��ʼ��Nrf24l01
//���ݶ����ʼ��NrfΪ��Ӧ��ģʽ
void Nrf24l01_init()
{
	Nrf_SPI2_Init();	//SPI2��ʼ��
	Nrf_GPIO_Init();	//Nrf����IO��ʼ��
	
	while(Nrf_Check())	//���nrf�Ƿ�λ
	{
		OLED_ShowString(0,0,"Not find Nrf",16);	
		OLED_Refresh_Gram();
	}
	//��Ϊ���Ͷ�ʱ�����Ҫ��ģʽ����֮ǰ
	
	NRF24L01_CE = 0;

    Nrf_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//������յ�ַ
    Nrf_Write_Buf(NRF_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);//���뷢�͵�ַ
    Nrf_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);		//����ͨ��0�Զ�Ӧ��
    Nrf_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);	//ʹ��ͨ��0���յ�ַ
    Nrf_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);	//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
    Nrf_Write_Reg(NRF_WRITE_REG+RF_CH,40);			//����RFͨ��Ƶ��
    Nrf_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);		//����TX�������,0db����,2Mbps,���������濪�� 
#ifdef TX_MODE_NOACK	//����Ϊ����ģʽ�������Զ�Ӧ��	
    Nrf_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);	//����ͨ��0�������ݿ��
    Nrf_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);
	#endif
	
#ifdef RX_MODE_NOACK	//����Ϊ����ģʽ�������Զ�Ӧ��
    Nrf_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);	//����ͨ��0�������ݿ��
    Nrf_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);			//���û�������ģʽ
	#endif
	
#ifdef TX_MODE_ACK		//����NrfΪ����ģʽ�����Զ�Ӧ��
    Nrf_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);         // IRQ�շ�����жϿ���,16λCRC,������
    Nrf_Write_Reg(FLUSH_TX,0xff);		//������ͻ�����
    Nrf_Write_Reg(FLUSH_RX,0xff);		//������ջ�����
    Nrf_SPI_ReadWriteByte(0x50);
    Nrf_SPI_ReadWriteByte(0x73);
    Nrf_Write_Reg(NRF_WRITE_REG+0x1c,0x01);	//ʹ��ͨ��0�Ķ�̬�غɳ���
    Nrf_Write_Reg(NRF_WRITE_REG+0x1d,0x07);	//���ö�̬�غɳ��ȣ������غ�ACK
	#endif
	
#ifdef RX_MODE_ACK		//����NrfΪ����ģʽ�����Զ�Ӧ��
    Nrf_Write_Reg(FLUSH_TX,0xff);		//������ͻ�����
    Nrf_Write_Reg(FLUSH_RX,0xff);		//������ջ�����
    Nrf_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);         // IRQ�շ�����жϿ���,16λCRC,������
    Nrf_SPI_ReadWriteByte(0x50);
    Nrf_SPI_ReadWriteByte(0x73);
    Nrf_Write_Reg(NRF_WRITE_REG+0x1c,0x01);	//ʹ��ͨ��0�Ķ�̬�غɳ���
    Nrf_Write_Reg(NRF_WRITE_REG+0x1d,0x07); //���ö�̬�غɳ��ȣ������غ�ACK
	#endif
	
    NRF24L01_CE = 1;	//����CE���ţ���������
	NRF24L01_CSN = 0;	//SPI3Ƭѡ��������
}

//���Nrf�Ƿ���յ�����
void Nrf_Check_Event(void)
{
    u8 sta = Nrf_Read_Reg(NRF_READ_REG + STATUS);
    if(sta & (1 << RX_DR))
    {
        Nrf_Read_Buf(RD_RX_PLOAD,rx_buf,RX_PLOAD_WIDTH);//�����յ������ݴ���rx_buf
		OLED_ShowString(120,0,"1",12);
		OLED_Refresh_Gram();
    }
    else
	{
        Nrf_Write_Reg(FLUSH_RX,0xff);					//�������
		OLED_ShowString(120,0,"2",12);
		OLED_Refresh_Gram();
	}
    Nrf_Write_Reg(NRF_WRITE_REG + STATUS, sta);
}

//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:�������״��
void Nrf_TxPacket(uint8_t * pBuf, uint8_t len)		//��ACKӦ�����ݰ�ʱ��
{
	NRF24L01_CE=0;		 //StandBy Iģʽ		
	Nrf_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (u8*)RX_ADDRESS, TX_ADR_WIDTH); // װ�ؽ��ն˵�ַ
	Nrf_Write_Buf(WR_TX_PLOAD, pBuf, len); 			 // װ������	
	NRF24L01_CE=1;		 //�ø�CE���������ݷ���
}

/*
 * ��������Nrf_Clear_Fifo
 * ��  ����*buf -> Ҫ��յ����ݵ�ַ		len -> ������ݳ���
 * ����ֵ����
 */
void Nrf_Clear_Fifo(u8 *buf, u8 len)
{
	u8 i;
	for(i=0; i<len; i++)
		buf[i] = 0;
}





