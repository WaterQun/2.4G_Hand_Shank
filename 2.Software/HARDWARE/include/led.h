#ifndef _LED_H
#define _LED_H

#include "sys.h"

#define LED0 PBout(4)	//�ض���LED0
#define LED1 PBout(5)	//�ض���LED1

/*
 * ����LED����
 * OnlyLED0��LED0������LED1Ϩ��
 * OnlyLED0��LED0Ϩ��LED1����
 * BothOfThem��LED0��LED1������
 * BothOff��LED0��LED1��Ϩ��
 */
typedef enum { OnlyLED0 = 1, OnlyLED1, BothOfThem, BothOff } User_Led;

//led.c ��������
void LED_init(void);	//LED0��LED1 GPIO��ʼ��
void Light_User_LED(User_Led state);	//����LED״̬

#endif


