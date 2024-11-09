//############################################################
// FILE:  PI_Cale.c
// Created on: 2017��1��4��
// Author: XQ
// summary: PI_Cale
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP/STM32������ƿ�����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��314306105
//############################################################

#include "PI_Cale.h"

extern PI_Control pi_spd;
extern PI_Control pi_id;
extern PI_Control pi_iq;


void PI_Controller(p_PI_Control pV) {
    /* proportional term */
    pV->up = pV->Ref - pV->Fbk;

    /* integral term */
    //pV->ui = (pV->Out == pV->v1)?(_IQmpy(pV->Ki, pV->up)+ pV->i1) : pV->i1; // �жϻ��ֱ���
    pV->ui = (pV->Out == pV->v1) ? (_IQmpy(pV->Ki, pV->up) + pV->i1) : pV->i1; // �жϻ��ֱ���

    pV->i1 = pV->ui; // ���ֵ�ﵽ�������ֵ����������ֵ�����ۼ�

    /* control output */ // _IQmpy ����λ2��^15�η� ����Kp/32768

    pV->v1 = _IQmpy(pV->Kp, (pV->up )) + pV->ui;
    pV->Out = IQsat(pV->v1, pV->Umax, pV->Umin); //�������
}

void PI_Pare_init(void) {
    //_IQmpy ����λ2��^15�η� ����Kp/32768   Ki/32768
    pi_spd.Kp = 9800; //  PID��������
    //pi_spd.Ki=760;  //    T* ��·���� /0.2
    pi_spd.Ki = 760; //    T* ��·���� /0.2
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
