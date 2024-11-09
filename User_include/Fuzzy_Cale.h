//############################################################
// FILE:  Fuzzy_Cale.h
// Created on: 2017��1��11��
// Author: XQ
// summary: Header file  and definition
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP/STM32������ƿ�����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��314306105
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
		 
		 

#define Fuzzy_Control_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0}  // ��ʼ������

void Fuzzy_Controller(p_Fuzzy_Control  pV);

#endif /* _Fuzzy_Cale_H*/
