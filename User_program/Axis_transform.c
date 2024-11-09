//############################################################
// FILE:  Axis_transform.c
// Created on: 2017年1月11日
// Author: XQ
// summary:Axis_transform
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子  
//网址: https://shuolidianzi.taobao.com 
//Author-QQ: 616264123  
//电机控制QQ群：314306105  
//############################################################

#include "Axis_transform.h"

//Alpha = Iu
//Beta = (√3/3*2^15*Iu + 2*√3/3*2^15*Iw)/(2^15)
// 此坐标变化是IQ15格式  计算系数 小数点的浮点数据*2^15
//克拉克变化输入时三相电流  电流传感器采集-4095到4096的IQ12格式
void CLARKE_Cale(p_CLARKE pV) {
    pV->Alpha = pV->As;
    pV->Beta = _IQmpy((pV->As +_IQmpy2(pV->Bs)), 18918);
    // 0.57735*2…^15=0.57735*32768    _IQ(0.57735026918963)   [sqrt(3)/3*32768=18918]
}

// Parking Id,Iq
// Id = Ialpha*cos+Ibeta*sin
// Iq = Ibeta*cos-Ialpha*sin
// 卡拉卡是等幅值变化所以  输入 Alpha和Beta是IQ12格式输出还是IQ12 从-4095到4096
//Sine和Cosine表格90度256个 参数是-1到1的IQ15  -32767到32768
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
