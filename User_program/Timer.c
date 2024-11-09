//############################################################
// FILE: Timer.c
// Created on: 2017年1月11日
// Author: XQ    
// summary: Timer    
// 定时器1电机控制，
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究    
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com   
//Author-QQ: 616264123
//电机控制QQ群：314306105
// printf 放在這邊 可以有時間觀念
//############################################################
#include "Timer.h"
#include "GPIO_int.h"
#include "Tim1_PWM.h"
#include "stm32f10x.h"
#include "ADC_int.h"
//#include "ThreeHall.h"
#include "IQ_math.h"
#include "Tim4_Encoder_PWMDAC.h"
#include "PI_Cale.h"
#include "Task_function.h"
#include "printf_uart.h"
#include <stdio.h>
#include <math.h>

//extern  Hall           Hall_Three;

//extern int16_t IQSin_Cos_Table[256];
//#include "printf_uart.h"
extern  TaskTime       TaskTimePare;
extern PI_Control pi_spd;
//extern Hall         Hall_Three;
extern logic logicContr;

void SysTickConfig(void) {
    /* Setup SysTick Timer for 1ms interrupts  */
    if (SysTick_Config(SystemCoreClock / 100))  //  10ms
    {
        /* Capture error */
        while (1);
    }
    /* Configure the SysTick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x0);
}

void RunSystimer(void)  //  every loop clear flag but not count
{
    if (TaskTimePare.IntClock_10ms == 1)  // 10ms     stm32f10x_it.c   enable TaskTimePare.IntClock_10ms
    {
        TaskTimePare.IntClock_10ms = 0;

        TaskTimePare.Tim10ms_flag = 1;

        // printf("Start_order %d \r\n", logicContr.Start_order);
        // printf("drive_car %d \r\n", logicContr.drive_car);
        // printf("pi_spd.OutF %d \r\n", pi_spd.OutF);


        if (++TaskTimePare.Tim100ms_count >= 10)  // 100ms
        {
            TaskTimePare.Tim100ms_count = 0;
            TaskTimePare.Tim100ms_flag = 1;
        }

        if (++TaskTimePare.Tim200ms_count >= 20)  // 200ms
        {
            TaskTimePare.Tim200ms_count = 0;
            TaskTimePare.Tim200ms_flag = 1;
        }

        if (++TaskTimePare.Tim300ms_count >= 30)  // 300ms
        {
            TaskTimePare.Tim300ms_count = 0;
            TaskTimePare.Tim300ms_flag = 1;
        }

        if (++TaskTimePare.Tim400ms_count >= 40)  // 400ms
        {
            TaskTimePare.Tim400ms_count = 0;
            TaskTimePare.Tim400ms_flag = 1;
        }

        if (++TaskTimePare.Tim500ms_count >= 50)  // 500ms
        {
            TaskTimePare.Tim500ms_count = 0;
            TaskTimePare.Tim500ms_flag = 1;
        }

        if (++TaskTimePare.Tim1s_count >= 100)  // 1s
        {
            LED1_Toggle();
            TaskTimePare.Tim1s_count = 0;
            TaskTimePare.Tim1s_flag = 1;
            // printf("%d \r\n", IQSin_Cos_Table[29]);
        }

        if (++TaskTimePare.Tim10s_count >= 1000)  // 10s
        {
            TaskTimePare.Tim10s_count = 0;
            TaskTimePare.Tim10s_flag = 1;
            //printf("Hello\r\n");
            //printf("%d \r\n",pi_spd.Ref);
        }

        if (++TaskTimePare.Tim1min_count >= 6000)  // 1min
        {
            TaskTimePare.Tim1min_count = 0;
            TaskTimePare.Tim1min_flag = 1;
        }
    }
}

void CLEAR_flag(void)  // 清除事件标志位
{
    if (TaskTimePare.Tim10ms_flag == 1) {
        TaskTimePare.Tim10ms_flag = 0;
    }

    if (TaskTimePare.Tim100ms_flag == 1) {
        TaskTimePare.Tim100ms_flag = 0;
    }
    if (TaskTimePare.Tim200ms_flag == 1) {
        TaskTimePare.Tim200ms_flag = 0;
    }
    if (TaskTimePare.Tim300ms_flag == 1) {
        TaskTimePare.Tim300ms_flag = 0;
    }

    if (TaskTimePare.Tim400ms_flag == 1) {
        TaskTimePare.Tim400ms_flag = 0;
    }
    if (TaskTimePare.Tim500ms_flag == 1) {
        TaskTimePare.Tim500ms_flag = 0;
    }
    if (TaskTimePare.Tim1s_flag == 1) {
        TaskTimePare.Tim1s_flag = 0;
    }
    if (TaskTimePare.Tim10s_flag == 1) {
        TaskTimePare.Tim10s_flag = 0;

    }
    if (TaskTimePare.Tim1min_flag == 1) {
        TaskTimePare.Tim1min_flag = 0;
    }
}

void Timer2Config(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 啟用 Timer2 時鐘
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // Timer2 基礎配置
    // 72MHz / 7200 = 10KHz, 10KHz / 100 = 100Hz (10ms)
    TIM_TimeBaseStructure.TIM_Period = 99;  // 100-1
    TIM_TimeBaseStructure.TIM_Prescaler = 7199;  // 7200-1
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // 配置 Timer2 中斷
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 啟用 Timer2 中斷
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    
    // 啟動 Timer2
    TIM_Cmd(TIM2, ENABLE);
}

// Timer2 中斷處理函數
void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        
        // 設置 10ms 標誌位，與原 SysTick 相同的功能
        TaskTimePare.IntClock_10ms = 1;
    }
}

//===========================================================================
// No more.
//===========================================================================
