//############################################################
// FILE:  ADC_int.c
// Created on: 2017年1月5日
// Author: XQ
// summary: ADCSampPare
// ADC采样 ，使用定时器1中断函数出发ADC中断采样 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：314306105
//############################################################
#include "ADC_int.h"
#include "GPIO_int.h"
#include "Task_function.h"
#include "Tim1_PWM.h"

#define ADC1_DR_Address    0x4001244C

extern logic logicContr;
extern ADCSamp ADCSampPare;
uint16_t ADC_ConvertedValue[5] = {0};
uint16_t BUS_CurrProtection = 600; //通过母线电流值保护硬件 软件处理 3A左右为600峰值

void ADC1_Configuration(void) {
    ADC_InitTypeDef ADC_InitStructure;
    /* ADC1 Periph clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    /* ADC1 DeInit */
    ADC_DeInit(ADC1);

    /* Initialize ADC structure */
    ADC_StructInit(&ADC_InitStructure);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //连续转换开启
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 5; //设置转换序列长度为2
    ADC_Init(ADC1, &ADC_InitStructure);

    RCC_ADCCLKConfig(RCC_PCLK2_Div2); // 72/2

    //采样时间为1us左右,(7cycles)  保证在PWM中断后进来采样后为在PWM中间采集相电流
    //常规转换序列1：通道0
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_7Cycles5);
    //常规转换序列2：通道1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_7Cycles5);
    //常规转换序列1：通道2
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_7Cycles5);
    //常规转换序列2：通道3
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_7Cycles5);
    //常规转换序列2：通道8
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 5, ADC_SampleTime_7Cycles5);

    // Enable ADC1
    ADC_Cmd(ADC1, ENABLE);
    // 开启ADC的DMA支持（要实现DMA功能，还需独立配置DMA通道等参数）
    ADC_DMACmd(ADC1, ENABLE);

    // 下面是ADC自动校准，开机后需执行一次，保证精度
    // Enable ADC1 reset calibaration register
    ADC_ResetCalibration(ADC1);
    // Check the end of ADC1 reset calibration register
    while (ADC_GetResetCalibrationStatus(ADC1));

    // Start ADC1 calibaration
    ADC_StartCalibration(ADC1);
    // Check the end of ADC1 calibration
    while (ADC_GetCalibrationStatus(ADC1));
    // ADC自动校准结束---------------
    //启动第一次AD转换
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    //因为已经配置好了DMA，接下来AD自动连续转换，结果自动保存在RegularConvData_Tab处
}


void DMA_Configuration(void) {
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) ADC1_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ADC_ConvertedValue;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    //BufferSize=2，因为ADC转换序列有2个通道
    //如此设置，使序列1结果放在RegularConvData_Tab[0]，序列2结果放在RegularConvData_Tab[1]
    DMA_InitStructure.DMA_BufferSize = 5;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    //循环模式开启，Buffer写满后，自动回到初始地址开始传输
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    //配置完成后，启动DMA通道
    DMA_Cmd(DMA1_Channel1, ENABLE);
}


//校准作用,电流传感器的偏移值为1.65V
void Offset_CurrentReading(void) //  没有PWM输出是调用
{
    static uint16_t ADC_PhaseU_Curr[64];
    static uint16_t ADC_PhaseV_Curr[64];
    static uint16_t ADC_BUS_Curr[64];
    static uint8_t i = 0;

    /* ADC Channel used for current reading are read  in order to get zero currents ADC values*/
    //64次采样求平均值，电流传感器初始校准
    ADC_PhaseU_Curr[i] = ADC_ConvertedValue[2];
    ADC_PhaseV_Curr[i] = ADC_ConvertedValue[1];
    ADC_BUS_Curr[i] = ADC_ConvertedValue[0];;

    i++;
    if (i >= 64) { i = 0; }
    //对于相电流和母线电流的电阻法测量电流,需要上电读取初始偏执电压
    if (logicContr.drive_car == 0) //
    {
        uint32_t sum_U = 0;
        uint32_t sum_V = 0;
        uint32_t sum_BUS = 0;
        //uint8_t i;
        for (u8 j = 0; j < 64; j++) {
            sum_U += ADC_PhaseU_Curr[j];
            sum_V += ADC_PhaseV_Curr[j];
            sum_BUS += ADC_BUS_Curr[j];
        }
        ADCSampPare.OffsetPhaseU_Curr = sum_U / 64;
        ADCSampPare.OffsetPhaseV_Curr = sum_V / 64;
        ADCSampPare.OffsetBUS_Curr = sum_BUS / 64;
    }
}

// 相电流是-2048到2048

void ADC_Sample(void) // 放在PWM中断进来后，采样时间为1us左右,(7cycles)  保证在PWM中断后进来采样后为在PWM中间采集相电流
{
    // 把电流采集运算偏执后，左移1为，乘2倍后将电流传感器AD采集值转换IQ12格式 -4095到4096
    // 此电流乘2倍与硬件差分放大电流2K/1K电阻放大2倍没有关系
    // 此传感器量程，100毫欧 运算放大器2倍，硬件0到3.3等效-1.6到1.6，100mr电阻传感器量程大约±8A
    //  若硬件修改，按照此算法比例计算
    ADCSampPare.BUS_Curr = (ADC_ConvertedValue[0] - ADCSampPare.OffsetBUS_Curr) + 25; // 补偿一阶低通滤波后的小的插值25经验值
    ADCSampPare.PhaseV_Curr = -(ADC_ConvertedValue[1] - ADCSampPare.OffsetPhaseV_Curr);
    ADCSampPare.PhaseU_Curr = -(ADC_ConvertedValue[2] - ADCSampPare.OffsetPhaseU_Curr);
    ADCSampPare.RP_speed_Voltage = ADC_ConvertedValue[3];
    ADCSampPare.BUS_Voltage = ADC_ConvertedValue[4];
    // 一阶低通数字滤波器 公式查文献百度  历史量0.96  采集新值0.04  (24/1024) / (2*3.14*0.000083)=46 hz  cutfreq
    ADCSampPare.BUS_CurrF = _IQ10mpy(ADCSampPare.Coeff_filterK1, ADCSampPare.BUS_CurrF) + _IQ10mpy(
                                ADCSampPare.Coeff_filterK2, ADCSampPare.BUS_Curr);
}


void Protection_software(void) //通过母线电流值保护硬件 软件处理 3A左右为600
{
    if (Abs(ADCSampPare.BUS_CurrF) > BUS_CurrProtection) // 大约为3A
    {
        Stop_Motor(); // 关闭电机停止
        logicContr.Start_order = 0;
    }
}

//===========================================================================
// No more.
//===========================================================================
