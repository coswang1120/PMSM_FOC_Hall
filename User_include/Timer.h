//############################################################
// FILE: Timer.h
// Created on: 2017��1��18��
// Author: XQ
// summary: Timer_
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP/STM32������ƿ�����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��314306105
//############################################################

#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f10x.h"

typedef struct {
    uint8_t PWMZD_count;     // ��PWM�м���  1/12.5K *25=2ms
    uint8_t IntClock_10ms;   // 10msʱ�ӱ�־
    uint8_t Tim10ms_flag;    // 10ms��־
    uint8_t Tim100ms_count;  // 100ms����
    uint8_t Tim100ms_flag;   // 100ms�¼���־λ
    uint8_t Tim200ms_count;  // 200ms cont
    uint8_t Tim200ms_flag;   // 200ms flag
    uint8_t Tim300ms_count;  // 300ms����
    uint8_t Tim300ms_flag;   // 300ms�¼���־λ
    uint8_t Tim400ms_count;  // 400ms����
    uint8_t Tim400ms_flag;   // 400ms�¼���־λ
    uint8_t Tim500ms_count;  // 500ms����
    uint8_t Tim500ms_flag;   // 500ms�¼���־λ
    uint8_t Tim1s_count;    // 1s����
    uint8_t Tim1s_flag;     // 1s�¼���־λ
    uint16_t Tim10s_count;   // 10s����
    uint8_t Tim10s_flag;    // 10s�¼���־λ
    uint16_t Tim1min_count;  // 1����
    uint8_t Tim1min_flag;   // 1�����¼���־λ
} TaskTime;

#define  TaskTime_DEFAULTS  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  // ��ʼ������

void SysTickConfig(void);  // �δ�ʱ������
void RunSystimer(void);    // ���м����¼���־
void CLEAR_flag(void);     // ����¼���־λ

void Timer2Config(void);  // ���� Timer2 ���ú�������
void Timer2_IRQHandler(void);  // Timer2 �Д�̎��������

#endif


//===========================================================================
// No more.
//===========================================================================
