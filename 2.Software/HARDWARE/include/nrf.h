#ifndef _NRF_H
#define _NRF_H

#include "sys.h"

void Nrf_SPI2_Init(void);	//2.4GӲ��SPI�ӿڳ�ʼ��
void SPI2_SetSpeed(u8 SpeedSet);	//����SPIͨ������
u8 SPI2_ReadWriteByte(u8 TxData);	//��дһ���ֽ�

/*
 * �궨��SPI��д����
 */
#define Nrf_SPI_ReadWriteByte(TxData)	SPI2_ReadWriteByte((u8)TxData)

/*
 * NRF���õ�IO���ض���
 */
#define Nrf_CS_Pin  GPIO_Pin_10	//Nrf24l01 SPIͨ��CSѡͨ�˿�
#define Nrf_CS_Port GPIOB		//Nrf24l01 SPIͨ��CSѡͨ�˿�

#define Nrf_CE_Pin  GPIO_Pin_12	//Nrf24l01 CEѡͨ����
#define Nrf_CE_Port GPIOB		//Nrf24l01 CEѡͨ�˿�

#define Nrf_IRQ_Pin  GPIO_Pin_11 //Nrf24l01 IRQ�ж�����
#define Nrf_IRQ_Port GPIOB		//Nrf24l01 IRQ�ж϶˿�

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//NRF24L01�Ĵ�����������
#define NRF_READ_REG    0x00  //�����üĴ���,��5λΪ�Ĵ�����ַ
#define NRF_WRITE_REG   0x20  //д���üĴ���,��5λΪ�Ĵ�����ַ
#define RD_RX_PLOAD     0x61  //��RX��Ч����,1~32�ֽ�
#define WR_TX_PLOAD     0xA0  //дTX��Ч����,1~32�ֽ�
#define FLUSH_TX        0xE1  //���TX FIFO�Ĵ���.����ģʽ����
#define FLUSH_RX        0xE2  //���RX FIFO�Ĵ���.����ģʽ����
#define REUSE_TX_PL     0xE3  //����ʹ����һ������,CEΪ��,���ݰ������Ϸ���.
#define NOP             0xFF  //�ղ���,����������״̬�Ĵ���
//SPI(NRF24L01)�Ĵ�����ַ
#define CONFIG          0x00  //���üĴ�����ַ;bit0:1����ģʽ,0����ģʽ;bit1:��ѡ��;bit2:CRCģʽ;bit3:CRCʹ��;
                              //bit4:�ж�MAX_RT(�ﵽ����ط������ж�)ʹ��;bit5:�ж�TX_DSʹ��;bit6:�ж�RX_DRʹ��
#define EN_AA           0x01  //ʹ���Զ�Ӧ����  bit0~5,��Ӧͨ��0~5
#define EN_RXADDR       0x02  //���յ�ַ����,bit0~5,��Ӧͨ��0~5
#define SETUP_AW        0x03  //���õ�ַ���(��������ͨ��):bit1,0:00,3�ֽ�;01,4�ֽ�;02,5�ֽ�;
#define SETUP_RETR      0x04  //�����Զ��ط�;bit3:0,�Զ��ط�������;bit7:4,�Զ��ط���ʱ 250*x+86us
#define RF_CH           0x05  //RFͨ��,bit6:0,����ͨ��Ƶ��;
#define RF_SETUP        0x06  //RF�Ĵ���;bit3:��������(0:1Mbps,1:2Mbps);bit2:1,���书��;bit0:�������Ŵ�������
#define STATUS          0x07  //״̬�Ĵ���;bit0:TX FIFO����־;bit3:1,��������ͨ����(���:6);bit4,�ﵽ�����ط�
                              //bit5:���ݷ�������ж�;bit6:���������ж�;
#define MAX_TX          0x10   //�ﵽ����ʹ����ж�
#define TX_OK           0x20  //TX��������ж�
#define RX_OK           0x40  //���յ������ж�

#define RX_DR           6       //�жϱ�־
#define TX_DS           5
#define MAX_RT          4

#define OBSERVE_TX      0x08  //���ͼ��Ĵ���,bit7:4,���ݰ���ʧ������;bit3:0,�ط�������
#define CD              0x09  //�ز����Ĵ���,bit0,�ز����;
#define RX_ADDR_P0      0x0A  //����ͨ��0���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P1      0x0B  //����ͨ��1���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P2      0x0C  //����ͨ��2���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P3      0x0D  //����ͨ��3���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P4      0x0E  //����ͨ��4���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P5      0x0F  //����ͨ��5���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define TX_ADDR         0x10  //���͵�ַ(���ֽ���ǰ),ShockBurstTMģʽ��,RX_ADDR_P0��˵�ַ���
#define RX_PW_P0        0x11  //��������ͨ��0��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P1        0x12  //��������ͨ��1��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P2        0x13  //��������ͨ��2��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P3        0x14  //��������ͨ��3��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P4        0x15  //��������ͨ��4��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P5        0x16  //��������ͨ��5��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define NRF_FIFO_STATUS 0x17  //FIFO״̬�Ĵ���;bit0,RX FIFO�Ĵ����ձ�־;bit1,RX FIFO����־;bit2,3,����
                              //bit4,TX FIFO�ձ�־;bit5,TX FIFO����־;bit6,1,ѭ��������һ���ݰ�.0,��ѭ��;
//////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * ����NRF2401����ģʽ��
 * 
 *			TX_MODE_NOACK	//����ģʽ�����Զ�Ӧ��
 * 			RX_MODE_NOACK	//����ģʽ�����Զ�Ӧ��
 *			TX_MODE_ACK		//����ģʽ���Զ�Ӧ��
 *			RX_MODE_ACK		//����ģʽ���Զ�Ӧ��
 */
#define TX_MODE_ACK		//�ض���Nrf����ģʽ
//24L01������
#define NRF24L01_CE   PBout(12) //24L01Ƭѡ�ź�
#define NRF24L01_CSN  PBout(10) //SPIƬѡ�ź�	   
#define NRF24L01_IRQ  PBin(11)  //IRQ������������
//24L01���ͽ������ݿ�ȶ���
#define TX_ADR_WIDTH    5   	//5�ֽڵĵ�ַ���
#define RX_ADR_WIDTH    5   	//5�ֽڵĵ�ַ���
#define TX_PLOAD_WIDTH  32  	//32�ֽڵ��û����ݿ��
#define RX_PLOAD_WIDTH  32  	//32�ֽڵ��û����ݿ��

//����ȫ�ֱ���
extern u8 rx_buf[RX_PLOAD_WIDTH];	//���������������
extern u8 tx_buf[TX_PLOAD_WIDTH];	//���巢����������

//�������Ϻ궨��
#define To_int(Data_H,Data_L) (((short)((char)Data_H)<<8)|Data_L)

//����Nrf���ú���
void Nrf_GPIO_Init(void);	//��ʼ��Nrf����GPIO
u8 Nrf_Check(void);			//���Nrf�Ƿ����
u8 Nrf_Write_Reg(u8 reg,u8 value);//дNrf�Ĵ���
u8 Nrf_Read_Reg(u8 reg);	//��Nrf�Ĵ���
u8 Nrf_Read_Buf(u8 reg,u8 *pBuf,u8 len);	//��ȡNrf����
u8 Nrf_Write_Buf(u8 reg, u8 *pBuf, u8 len);	//Nrf��������
void Nrf_TxPacket_AP(u8 *pBuf, u8 len);	//�������ݣ��Զ�Ӧ��ʱʹ��
void Nrf24l01_init(void);	//��ʼ��Nrf����ģʽ
void Nrf_Check_Event(void);	//���Nrf�Ƿ���յ�����
void Nrf_TxPacket(uint8_t * pBuf, uint8_t len);	//Nrf����һ������
void Nrf_Clear_Fifo(u8 *buf, u8 len);	//���������

#endif

