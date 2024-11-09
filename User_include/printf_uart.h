#ifndef __PRINTF_UART_H
#define __PRINTF_UART_H

#include <stdio.h>
#include "stm32f10x.h"


#include "IQ_math.h"
#include "stm32f10x.h"

typedef struct {
    uint8_t Uart_txdr[8];     // 串口发送数据
    uint8_t Uart_rxdr[8];     // 串口接收数据
    uint16_t fact_BUS_Curr;   // 实际母线电流
    uint16_t fact_BUS_Voil;   // 实际母线电压
    uint16_t iq_test;         // q轴测试电流
    uint16_t id_test;         // d轴测试电流
    uint16_t uq_test;         // q轴测试电压
    uint16_t ud_test;         // d轴测试电压
    int16_t IqRef_test;       // d轴给定参数电流
    uint16_t Speed_target;    // 目标转速
    uint16_t Speed_fact;      // 实际转速
    uint8_t Rece_flag;        // 接收标志
    uint8_t Rxdr_byte;        // 接收字节数
    uint8_t chaoshi_jishu;    // 超时判断计数
    uint8_t chaoshi_pand;     // 超时判断时间
    uint8_t chaoshi_pandold;  // 历史超时判断时间
    uint8_t chaoshi_shu;      // 超时判断数
} Test;

#define  Test_DEFAULTS  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  // 初始化参数

void Uart3Init(u32 bound);
void PrintfInit(USART_TypeDef *uarTx);
int _isatty(int fd);
int _write(int fd, char *ptr, int len);
int _close(int fd);
int _lseek(int fd, int ptr, int dir);
int _read(int fd, char *ptr, int len);

#endif  // #ifndef _PRINT_UART_H__
