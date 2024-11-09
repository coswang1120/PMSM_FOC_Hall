//############################################################
// FILE:  Fuzzy_Cale.h
// Created on: 2017年1月11日
// Author: XQ
// summary: Header file  and definition
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：314306105
//############################################################

#ifndef  _Fuzzy_Cale_H
#define  _Fuzzy_Cale_H

#include "IQ_math.h"
#include "stm32f10x.h"

typedef struct {
       u16  ke;
	     u16  kv;
	     u16  ku;
       int32_t  a1;
	     int32_t  b1;
	     int32_t  I;
	     u32  set;
       u16  condition;
       float	r;
	     float  grade_reg[2];
	     float  grade[49];
	     int32_t  p1;
	     int32_t  p2;
		   int32_t  ph;
	   } Fuzzy_Control, *p_Fuzzy_Control ;	 
		 
		 

#define Fuzzy_Control_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0}  // 初始化参数

void Fuzzy_Controller(p_Fuzzy_Control  pV);

#endif /* _Fuzzy_Cale_H*/
