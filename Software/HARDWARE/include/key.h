#ifndef _KEY_H
#define _KEY_H

#include "sys.h"

// 按键输入宏定义
#define KEY_1 	    PCin(14)    //Key_1
#define KEY_WKUP 	  PAin(0)     //WKUP

#define KEY_UP 	    PAin(11)    //UP
#define KEY_DOWN  	PAin(12)    //DOWN
#define KEY_LEFT   	PAin(15)    //LEFT
#define KEY_RIGHT 	PBin(3)		  //RIGHT

#define KEY_A 	    PBin(6)		  //A
#define KEY_AA    	PBin(7)		  //A+
#define KEY_B     	PBin(8)		  //B
#define KEY_BB    	PBin(9)		  //B+

#define YG1KEY 	    PAin(8)		  //左手遥感按键
#define YG2KEY 	    PCin(15)		//右手遥感按键

// 按键初始化
void KEY_init(void);	//按键配置初始化

#endif


