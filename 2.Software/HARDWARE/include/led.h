#ifndef _LED_H
#define _LED_H

#include "sys.h"

#define LED0 PBout(4)	//重定义LED0
#define LED1 PBout(5)	//重定义LED1

/*
 * 定义LED变量
 * OnlyLED0：LED0点亮，LED1熄灭
 * OnlyLED0：LED0熄灭，LED1点亮
 * BothOfThem：LED0和LED1都点亮
 * BothOff：LED0和LED1都熄灭
 */
typedef enum { OnlyLED0 = 1, OnlyLED1, BothOfThem, BothOff } User_Led;

//led.c 函数声明
void LED_init(void);	//LED0和LED1 GPIO初始化
void Light_User_LED(User_Led state);	//设置LED状态

#endif


