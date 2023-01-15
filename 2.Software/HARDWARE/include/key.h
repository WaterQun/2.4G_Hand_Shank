#ifndef _KEY_H
#define _KEY_H

#include "sys.h"

// ��������궨��
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

#define YG1KEY 	    PAin(8)		  //����ң�а���
#define YG2KEY 	    PCin(15)		//����ң�а���

// ������ʼ��
void KEY_init(void);	//�������ó�ʼ��

#endif


