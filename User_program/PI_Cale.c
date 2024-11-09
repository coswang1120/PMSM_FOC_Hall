//############################################################
// FILE:  PI_Cale.c
// Created on: 2017年1月4日
// Author: XQ
// summary: PI_Cale
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：314306105
//############################################################

#include "PI_Cale.h"

extern PI_Control pi_spd;
extern PI_Control pi_id;
extern PI_Control pi_iq;


void PI_Controller(p_PI_Control pV) {
    /* proportional term */
    pV->up = pV->Ref - pV->Fbk;

    /* integral term */
    //pV->ui = (pV->Out == pV->v1)?(_IQmpy(pV->Ki, pV->up)+ pV->i1) : pV->i1; // 判断积分饱和
    pV->ui = (pV->Out == pV->v1) ? (_IQmpy(pV->Ki, pV->up) + pV->i1) : pV->i1; // 判断积分饱和

    pV->i1 = pV->ui; // 输出值达到保护输出值就锁定积分值不在累加

    /* control output */ // _IQmpy 是移位2…^15次方 所以Kp/32768

    pV->v1 = _IQmpy(pV->Kp, (pV->up )) + pV->ui;
    pV->Out = IQsat(pV->v1, pV->Umax, pV->Umin); //限制输出
}

void PI_Pare_init(void) {
    //_IQmpy 是移位2…^15次方 所以Kp/32768   Ki/32768
    pi_spd.Kp = 9800; //  PID参数是用
    //pi_spd.Ki=760;  //    T* 环路周期 /0.2
    pi_spd.Ki = 760; //    T* 环路周期 /0.2
    pi_spd.Umax = 30000; //
    pi_spd.Umin = 0;

    pi_id.Kp = 810; //
    pi_id.Ki = 310; //
    pi_id.Umax = 8000; //
    pi_id.Umin = -8000; //

    pi_iq.Kp = 980;
    //pi_iq.Ki=386 ;
    pi_iq.Ki = 386;
    pi_iq.Umax = 30000;
    pi_iq.Umin = 0;
}

//===========================================================================
// No more.
//===========================================================================
