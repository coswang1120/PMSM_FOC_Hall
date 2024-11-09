//############################################################
// FILE: Tim1_ISR_MCLoop.c
// Created on: 2017年1月15日
// Author: XQ
// summary: Tim1_ISR_MCLoop
// 定时器1电机控制， 中断环路闭环控制  
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板          
//硕历电子                     
//网址: https://shuolidianzi.taobao.com
//修改日期:2018/7/12
//版本：V17_3                                          
//Author-QQ: 616264123
//电机控制QQ群：314306105
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
// 一阶数字低通滤波器   328+696=1024  a=328/1024
// https://wenku.baidu.com/view/2d022b6b7375a417866f8f7e.html?from=search

void TIM1_UP_IRQHandler(void) // 触发ADC中断采样和电机环路控制
{
    int8_t rotconst;
    //此程序更新中断一直执行


    if (logicContr.Run_mode == 1) {
        rotconst = -1;
    } else {
        rotconst = 1;
    }

    ADC_Sample(); //相电流母线电流电压采
    ThreeHallanglecale(); //霍尔状态和计算位置角度
    if (logicContr.drive_car == 0) //
    {
        Offset_CurrentReading(); // 电流采样偏执电压读取
    }
    TaskTimePare.PWMZD_count++;
    if (TaskTimePare.PWMZD_count > 24) // 2ms的事件任务
    {
        TaskTimePare.PWMZD_count = 0;
        Hall_Three_Speedcale(); //根据霍尔角度计算速度

        knob_control(); //转把电位器调速的控制

        //通讯测试参数输出
        //测试参数输出，以下为打包测试参数包括电压电流转速等参数，以便通讯发送输出参数
        TestPare.id_test = pi_id.Fbk; // IQ15格式
        TestPare.iq_test = pi_iq.Fbk;
        TestPare.ud_test = IparkU.Ds;
        TestPare.uq_test = IparkU.Qs;

        TestPare.fact_BUS_Voil = ADCSampPare.BUS_Voltage;
        TestPare.fact_BUS_Curr = ADCSampPare.BUS_Curr;
        TestPare.Speed_fact = pi_spd.Fbk;
        TestPare.Speed_target = pi_spd.Ref;

        pi_spd.Fbk = Hall_Three.Speed_RPM; //   0---4096RPM      max  -> 2200rpm  ~ 2700 rpm
        PI_Controller((p_PI_Control) &pi_spd); // 速度环PI控制
        pi_spd.OutF = _IQ10mpy(FilK1, pi_spd.OutF) + _IQ10mpy(FilK2, pi_spd.Out);
        //            ini : 0  med: 1000~9000    stable: 0


        // printf("%d \r\n",pi_spd.OutF);  // can't add because it will cause motor rotate fail
    }

    ClarkeI.As = ADCSampPare.PhaseU_Curr;
    ClarkeI.Bs = ADCSampPare.PhaseV_Curr;

    CLARKE_Cale((p_CLARKE) &ClarkeI); // CLARKE变换

    ParkI.Alpha = ClarkeI.Alpha;
    ParkI.Beta = ClarkeI.Beta;

    AngleSin_Cos.IQAngle = Hall_Three.ele_angleIQ; //  0~65536   random

    IQSin_Cos_Cale((p_IQSin_Cos) &AngleSin_Cos); //磁极位置角度，正余弦计算函数

    ParkI.Sine = AngleSin_Cos.IQSin;
    ParkI.Cosine = AngleSin_Cos.IQCos;

    PARK_Cale((p_PARK) &ParkI); // park变换

    pi_id.Ref = 0;
    pi_id.Fbk = ParkI.Ds;
    PI_Controller((p_PI_Control) &pi_id); // id轴 PI控制
    pi_id.OutF = _IQ10mpy(FilK1, pi_id.OutF) + _IQ10mpy(FilK2, pi_id.Out);

    pi_iq.Fbk = ParkI.Qs;
    pi_iq.Ref = pi_spd.OutF;
    PI_Controller((p_PI_Control) &pi_iq); // iq轴 PI控制
    pi_iq.OutF = _IQ10mpy(FilK1, pi_iq.OutF) + _IQ10mpy(FilK2, pi_iq.Out);


    if (logicContr.Control_Mode == 1) {
        //IparkU.Ds=pi_id.OutF;   // 采用开环

        IparkU.Ds = 0;
        IparkU.Qs = 9 * pi_spd.Ref; //  open loop     0< 12*pi_spd.Ref <30000    don't use any PI controller

        IparkU.Qs = IQsat(IparkU.Qs, 15000, 0);

        //IparkU.Qs= 13000;  // max < 25000
    }

    if (logicContr.Control_Mode == 2) {
        IparkU.Ds = pi_id.OutF; // 采用id电流闭环     feedback  id  speed  close loop     and    don't use pid(iq)
        IparkU.Qs = rotconst * pi_spd.OutF; // 采用速度闭环     0~ 24000
    }
    if (logicContr.Control_Mode == 3) {
        IparkU.Ds = pi_id.OutF; //反转控制 ，id电流闭环   feedback  id  -speed  close loop
        IparkU.Qs = rotconst * pi_spd.OutF;
    }
    if (logicContr.Control_Mode == 4) {
        // pi_spd.Umax =1500;  // 输出使能给定Iq值限制值      feedback  id  iq[speed + park.Q]  close loop

        IparkU.Ds = pi_id.OutF; // 采用速度闭环  id电流闭环
        IparkU.Qs = rotconst * (pi_iq.OutF / 2.0); // 最大电压利用率为1=2^15=32768 最大值 100%
    }

    IparkU.Sine = AngleSin_Cos.IQSin; //   - 32000  ~ 32000 random jump
    IparkU.Cosine = AngleSin_Cos.IQCos;
    IPARK_Cale((p_IPARK) &IparkU); // 反park变换

    Svpwmdq.Ualpha = IparkU.Alpha;
    Svpwmdq.Ubeta = IparkU.Beta;

    SVPWM_Cale((p_SVPWM) &Svpwmdq); //SVPWM计算占空比

    Svpwm_Outpwm(); // PWM输出    // PWM_DUTY[0] 1440 ~2880 PWM_DUTY[1] 1440 ~2880  PWM_DUTY[2] 1440 ~2880

    Protection_software(); // 母线电流过流直接关管子 AD，600是3A左右可修改调试 硬件不准许超过3A

    //定时器4在除正交编码器的永磁同步电机FOC控制中运用
    //其他程序代码做用PWM输出数字量通过RC电阻低通滤波后就可以测到数据波形，例如马鞍矢量波
    TIM_SetCompare3(TIM4, ADC_ConvertedValue[2]); //  U相电流 DAC通道1     ( demo )
    TIM_SetCompare4(TIM4, (Hall_Three.ele_angleIQ >> 5)); // 霍尔电角度 DAC通道2   ( demo )


    TIM_ClearFlag(TIM1, TIM_FLAG_Update); // 清除标志位
}


//===========================================================================
// No more.
//===========================================================================
