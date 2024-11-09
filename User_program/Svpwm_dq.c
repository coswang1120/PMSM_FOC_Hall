//############################################################
// FILE:  Svpwm_dq.c
// Created on: 2017年1月18日
// Author: XQ
// summary: Svpwm_dq
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com       
//Author-QQ: 616264123
//电机控制QQ群：314306105
//############################################################

#include "Svpwm_dq.h"

// SVPWM是7段式矢量调试，扇区与输出电压占空比经过简化后为对称输出，1和4，2和5，3和6对称
// 文档给出是按照7段式SVPWM一点点按照矢量调制原理推到，中间推到有Uabc的三相电压
// T1和T1作用时间，Txyz时间最终等效Tabc三相输入占空比,
// Udc/√3=1， 是调制最大电1 是等效为IQ15是32768  电压利用率是100%，算死区计算2*1.8=3.6/80us=4.5% 
// 调制利用率等于(1-0.045)=0.955=95.5%   约等于31000
// Ualpha和Ubeta是和uq ud等幅值变换所以都是-32767到32768
void SVPWM_Cale(p_SVPWM pV) {
    pV->tmp1 = pV->Ubeta; // 相当于二相静止坐标--到三相静止变换出Uabc
    pV->tmp2 = _IQdiv2(pV->Ubeta) + _IQmpy(28377, pV->Ualpha); //0.866*32768=sqrt(3)/2*2^15
    pV->tmp3 = pV->tmp2 - pV->tmp1;

    pV->VecSector = 3; // 根据三相电压符号计算矢量扇区
    pV->VecSector = (pV->tmp2 > 0) ? (pV->VecSector - 1) : pV->VecSector;
    pV->VecSector = (pV->tmp3 > 0) ? (pV->VecSector - 1) : pV->VecSector;
    pV->VecSector = (pV->tmp1 < 0) ? (7 - pV->VecSector) : pV->VecSector;

    if (pV->VecSector == 1 || pV->VecSector == 4) // 根据矢量扇区计算矢量占空比Tabc
    {
        pV->Ta = pV->tmp2;
        pV->Tb = pV->tmp1 - pV->tmp3;
        pV->Tc = -pV->tmp2;
    } else if (pV->VecSector == 2 || pV->VecSector == 5) {
        pV->Ta = pV->tmp3 + pV->tmp2;
        pV->Tb = pV->tmp1;
        pV->Tc = -pV->tmp1;
    } else {
        pV->Ta = pV->tmp3;
        pV->Tb = -pV->tmp3;
        pV->Tc = -(pV->tmp1 + pV->tmp2);
    }
}

//===========================================================================
// No more.
//===========================================================================
