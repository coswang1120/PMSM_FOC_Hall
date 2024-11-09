//############################################################
// FILE:  Axis_transform.c
// Created on: 2017��1��11��
// Author: XQ
// summary:Axis_transform
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP/STM32������ƿ�����
//˶������  
//��ַ: https://shuolidianzi.taobao.com 
//Author-QQ: 616264123  
//�������QQȺ��314306105  
//############################################################

#include "Axis_transform.h"

//Alpha = Iu
//Beta = (��3/3*2^15*Iu + 2*��3/3*2^15*Iw)/(2^15)
// ������仯��IQ15��ʽ  ����ϵ�� С����ĸ�������*2^15
//�����˱仯����ʱ�������  �����������ɼ�-4095��4096��IQ12��ʽ
void CLARKE_Cale(p_CLARKE pV) {
    pV->Alpha = pV->As;
    pV->Beta = _IQmpy((pV->As +_IQmpy2(pV->Bs)), 18918);
    // 0.57735*2��^15=0.57735*32768    _IQ(0.57735026918963)   [sqrt(3)/3*32768=18918]
}

// Parking Id,Iq
// Id = Ialpha*cos+Ibeta*sin
// Iq = Ibeta*cos-Ialpha*sin
// �������ǵȷ�ֵ�仯����  ���� Alpha��Beta��IQ12��ʽ�������IQ12 ��-4095��4096
//Sine��Cosine���90��256�� ������-1��1��IQ15  -32767��32768
void PARK_Cale(p_PARK pV) {
    pV->Ds = _IQmpy(pV->Alpha, pV->Cosine) + _IQmpy(pV->Beta, pV->Sine);
    pV->Qs = _IQmpy(pV->Beta, pV->Cosine) - _IQmpy(pV->Alpha, pV->Sine);
}

//IParking Ia,Ib
// Ialpha = Id*cos-Iq*sin
// Ibeta = Iq*cos+Id*sin
void IPARK_Cale(p_IPARK pV) {
    pV->Alpha = _IQmpy(pV->Ds, pV->Cosine) - _IQmpy(pV->Qs, pV->Sine);
    pV->Beta = _IQmpy(pV->Qs, pV->Cosine) + _IQmpy(pV->Ds, pV->Sine);
}

//===========================================================================
// No more.
//===========================================================================
