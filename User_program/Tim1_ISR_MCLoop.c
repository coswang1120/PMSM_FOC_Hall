//############################################################
// FILE: Tim1_ISR_MCLoop.c
// Created on: 2017��1��15��
// Author: XQ
// summary: Tim1_ISR_MCLoop
// ��ʱ��1������ƣ� �жϻ�·�ջ�����  
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP/STM32������ƿ�����          
//˶������                     
//��ַ: https://shuolidianzi.taobao.com
//�޸�����:2018/7/12
//�汾��V17_3                                          
//Author-QQ: 616264123
//�������QQȺ��314306105
//############################################################
#include "Tim1_ISR_MCLoop.h"
#include "ADC_int.h"
#include "Tim1_PWM.h"
#include "GPIO_int.h"
#include "ThreeHall.h"
#include "Axis_transform.h"
#include "Svpwm_dq.h"
#include "Task_function.h"
#include "PI_Cale.h"
#include "Timer.h"
//#include "Usart_RS232.h"
#include "Tim4_Encoder_PWMDAC.h"
#include "Fuzzy_Cale.h"
#include "printf_uart.h"
#include "stdio.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "IQ_math.h"


extern PI_Control pi_spd;
extern PI_Control pi_id;
extern PI_Control pi_iq;
extern ADCSamp ADCSampPare;
extern uint16_t PWM_DUTY[3];
extern Hall Hall_Three;
extern logic logicContr;
extern TaskTime TaskTimePare;
extern CLARKE ClarkeI;
extern PARK ParkI;
extern IPARK IparkU;
extern SVPWM Svpwmdq;
extern IQSin_Cos AngleSin_Cos;
extern Test TestPare;
extern IQAtan IQAtan_Pare;
extern uint16_t ADC_ConvertedValue[5];
extern Fuzzy_Control Fuzzy;

uint16_t FilK1 = 328;
uint16_t FilK2 = 696;
// һ�����ֵ�ͨ�˲���   328+696=1024  a=328/1024
// https://wenku.baidu.com/view/2d022b6b7375a417866f8f7e.html?from=search

void TIM1_UP_IRQHandler(void) // ����ADC�жϲ����͵����·����
{
    int8_t rotconst;
    //�˳�������ж�һֱִ��


    if (logicContr.Run_mode == 1) {
        rotconst = -1;
    } else {
        rotconst = 1;
    }

    ADC_Sample(); //�����ĸ�ߵ�����ѹ��
    ThreeHallanglecale(); //����״̬�ͼ���λ�ýǶ�
    if (logicContr.drive_car == 0) //
    {
        Offset_CurrentReading(); // ��������ƫִ��ѹ��ȡ
    }
    TaskTimePare.PWMZD_count++;
    if (TaskTimePare.PWMZD_count > 24) // 2ms���¼�����
    {
        TaskTimePare.PWMZD_count = 0;
        Hall_Three_Speedcale(); //���ݻ����Ƕȼ����ٶ�

        knob_control(); //ת�ѵ�λ�����ٵĿ���

        //ͨѶ���Բ������
        //���Բ������������Ϊ������Բ���������ѹ����ת�ٵȲ������Ա�ͨѶ�����������
        TestPare.id_test = pi_id.Fbk; // IQ15��ʽ
        TestPare.iq_test = pi_iq.Fbk;
        TestPare.ud_test = IparkU.Ds;
        TestPare.uq_test = IparkU.Qs;

        TestPare.fact_BUS_Voil = ADCSampPare.BUS_Voltage;
        TestPare.fact_BUS_Curr = ADCSampPare.BUS_Curr;
        TestPare.Speed_fact = pi_spd.Fbk;
        TestPare.Speed_target = pi_spd.Ref;

        pi_spd.Fbk = Hall_Three.Speed_RPM; //   0---4096RPM      max  -> 2200rpm  ~ 2700 rpm
        PI_Controller((p_PI_Control) &pi_spd); // �ٶȻ�PI����
        pi_spd.OutF = _IQ10mpy(FilK1, pi_spd.OutF) + _IQ10mpy(FilK2, pi_spd.Out);
        //            ini : 0  med: 1000~9000    stable: 0


        // printf("%d \r\n",pi_spd.OutF);  // can't add because it will cause motor rotate fail
    }

    ClarkeI.As = ADCSampPare.PhaseU_Curr;
    ClarkeI.Bs = ADCSampPare.PhaseV_Curr;

    CLARKE_Cale((p_CLARKE) &ClarkeI); // CLARKE�任

    ParkI.Alpha = ClarkeI.Alpha;
    ParkI.Beta = ClarkeI.Beta;

    AngleSin_Cos.IQAngle = Hall_Three.ele_angleIQ; //  0~65536   random

    IQSin_Cos_Cale((p_IQSin_Cos) &AngleSin_Cos); //�ż�λ�ýǶȣ������Ҽ��㺯��

    ParkI.Sine = AngleSin_Cos.IQSin;
    ParkI.Cosine = AngleSin_Cos.IQCos;

    PARK_Cale((p_PARK) &ParkI); // park�任

    pi_id.Ref = 0;
    pi_id.Fbk = ParkI.Ds;
    PI_Controller((p_PI_Control) &pi_id); // id�� PI����
    pi_id.OutF = _IQ10mpy(FilK1, pi_id.OutF) + _IQ10mpy(FilK2, pi_id.Out);

    pi_iq.Fbk = ParkI.Qs;
    pi_iq.Ref = pi_spd.OutF;
    PI_Controller((p_PI_Control) &pi_iq); // iq�� PI����
    pi_iq.OutF = _IQ10mpy(FilK1, pi_iq.OutF) + _IQ10mpy(FilK2, pi_iq.Out);


    if (logicContr.Control_Mode == 1) {
        //IparkU.Ds=pi_id.OutF;   // ���ÿ���

        IparkU.Ds = 0;
        IparkU.Qs = 9 * pi_spd.Ref; //  open loop     0< 12*pi_spd.Ref <30000    don't use any PI controller

        IparkU.Qs = IQsat(IparkU.Qs, 15000, 0);

        //IparkU.Qs= 13000;  // max < 25000
    }

    if (logicContr.Control_Mode == 2) {
        IparkU.Ds = pi_id.OutF; // ����id�����ջ�     feedback  id  speed  close loop     and    don't use pid(iq)
        IparkU.Qs = rotconst * pi_spd.OutF; // �����ٶȱջ�     0~ 24000
    }
    if (logicContr.Control_Mode == 3) {
        IparkU.Ds = pi_id.OutF; //��ת���� ��id�����ջ�   feedback  id  -speed  close loop
        IparkU.Qs = rotconst * pi_spd.OutF;
    }
    if (logicContr.Control_Mode == 4) {
        // pi_spd.Umax =1500;  // ���ʹ�ܸ���Iqֵ����ֵ      feedback  id  iq[speed + park.Q]  close loop

        IparkU.Ds = pi_id.OutF; // �����ٶȱջ�  id�����ջ�
        IparkU.Qs = rotconst * (pi_iq.OutF / 2.0); // ����ѹ������Ϊ1=2^15=32768 ���ֵ 100%
    }

    IparkU.Sine = AngleSin_Cos.IQSin; //   - 32000  ~ 32000 random jump
    IparkU.Cosine = AngleSin_Cos.IQCos;
    IPARK_Cale((p_IPARK) &IparkU); // ��park�任

    Svpwmdq.Ualpha = IparkU.Alpha;
    Svpwmdq.Ubeta = IparkU.Beta;

    SVPWM_Cale((p_SVPWM) &Svpwmdq); //SVPWM����ռ�ձ�

    Svpwm_Outpwm(); // PWM���    // PWM_DUTY[0] 1440 ~2880 PWM_DUTY[1] 1440 ~2880  PWM_DUTY[2] 1440 ~2880

    Protection_software(); // ĸ�ߵ�������ֱ�ӹع��� AD��600��3A���ҿ��޸ĵ��� Ӳ����׼����3A

    //��ʱ��4�ڳ�����������������ͬ�����FOC����������
    //���������������PWM���������ͨ��RC�����ͨ�˲���Ϳ��Բ⵽���ݲ��Σ�������ʸ����
    TIM_SetCompare3(TIM4, ADC_ConvertedValue[2]); //  U����� DACͨ��1     ( demo )
    TIM_SetCompare4(TIM4, (Hall_Three.ele_angleIQ >> 5)); // ������Ƕ� DACͨ��2   ( demo )


    TIM_ClearFlag(TIM1, TIM_FLAG_Update); // �����־λ
}


//===========================================================================
// No more.
//===========================================================================
