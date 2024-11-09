//############################################################
// FILE:  ADC_int.c
// Created on: 2017��1��5��
// Author: XQ
// summary: ADCSampPare
// ADC���� ��ʹ�ö�ʱ��1�жϺ�������ADC�жϲ��� 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP/STM32������ƿ�����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//�������QQȺ��314306105
//############################################################
#include "ADC_int.h"
#include "GPIO_int.h"
#include "Task_function.h"
#include "Tim1_PWM.h"

#define ADC1_DR_Address    0x4001244C

extern logic logicContr;
extern ADCSamp ADCSampPare;
uint16_t ADC_ConvertedValue[5] = {0};
uint16_t BUS_CurrProtection = 600; //ͨ��ĸ�ߵ���ֵ����Ӳ�� ������� 3A����Ϊ600��ֵ

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
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //����ת������
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 5; //����ת�����г���Ϊ2
    ADC_Init(ADC1, &ADC_InitStructure);

    RCC_ADCCLKConfig(RCC_PCLK2_Div2); // 72/2

    //����ʱ��Ϊ1us����,(7cycles)  ��֤��PWM�жϺ����������Ϊ��PWM�м�ɼ������
    //����ת������1��ͨ��0
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_7Cycles5);
    //����ת������2��ͨ��1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_7Cycles5);
    //����ת������1��ͨ��2
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_7Cycles5);
    //����ת������2��ͨ��3
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_7Cycles5);
    //����ת������2��ͨ��8
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 5, ADC_SampleTime_7Cycles5);

    // Enable ADC1
    ADC_Cmd(ADC1, ENABLE);
    // ����ADC��DMA֧�֣�Ҫʵ��DMA���ܣ������������DMAͨ���Ȳ�����
    ADC_DMACmd(ADC1, ENABLE);

    // ������ADC�Զ�У׼����������ִ��һ�Σ���֤����
    // Enable ADC1 reset calibaration register
    ADC_ResetCalibration(ADC1);
    // Check the end of ADC1 reset calibration register
    while (ADC_GetResetCalibrationStatus(ADC1));

    // Start ADC1 calibaration
    ADC_StartCalibration(ADC1);
    // Check the end of ADC1 calibration
    while (ADC_GetCalibrationStatus(ADC1));
    // ADC�Զ�У׼����---------------
    //������һ��ADת��
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    //��Ϊ�Ѿ����ú���DMA��������AD�Զ�����ת��������Զ�������RegularConvData_Tab��
}


void DMA_Configuration(void) {
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) ADC1_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ADC_ConvertedValue;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    //BufferSize=2����ΪADCת��������2��ͨ��
    //������ã�ʹ����1�������RegularConvData_Tab[0]������2�������RegularConvData_Tab[1]
    DMA_InitStructure.DMA_BufferSize = 5;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    //ѭ��ģʽ������Bufferд�����Զ��ص���ʼ��ַ��ʼ����
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    //������ɺ�����DMAͨ��
    DMA_Cmd(DMA1_Channel1, ENABLE);
}


//У׼����,������������ƫ��ֵΪ1.65V
void Offset_CurrentReading(void) //  û��PWM����ǵ���
{
    static uint16_t ADC_PhaseU_Curr[64];
    static uint16_t ADC_PhaseV_Curr[64];
    static uint16_t ADC_BUS_Curr[64];
    static uint8_t i = 0;

    /* ADC Channel used for current reading are read  in order to get zero currents ADC values*/
    //64�β�����ƽ��ֵ��������������ʼУ׼
    ADC_PhaseU_Curr[i] = ADC_ConvertedValue[2];
    ADC_PhaseV_Curr[i] = ADC_ConvertedValue[1];
    ADC_BUS_Curr[i] = ADC_ConvertedValue[0];;

    i++;
    if (i >= 64) { i = 0; }
    //�����������ĸ�ߵ����ĵ��跨��������,��Ҫ�ϵ��ȡ��ʼƫִ��ѹ
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

// �������-2048��2048

void ADC_Sample(void) // ����PWM�жϽ����󣬲���ʱ��Ϊ1us����,(7cycles)  ��֤��PWM�жϺ����������Ϊ��PWM�м�ɼ������
{
    // �ѵ����ɼ�����ƫִ������1Ϊ����2���󽫵���������AD�ɼ�ֵת��IQ12��ʽ -4095��4096
    // �˵�����2����Ӳ����ַŴ����2K/1K����Ŵ�2��û�й�ϵ
    // �˴��������̣�100��ŷ ����Ŵ���2����Ӳ��0��3.3��Ч-1.6��1.6��100mr���贫�������̴�Լ��8A
    //  ��Ӳ���޸ģ����մ��㷨��������
    ADCSampPare.BUS_Curr = (ADC_ConvertedValue[0] - ADCSampPare.OffsetBUS_Curr) + 25; // ����һ�׵�ͨ�˲����С�Ĳ�ֵ25����ֵ
    ADCSampPare.PhaseV_Curr = -(ADC_ConvertedValue[1] - ADCSampPare.OffsetPhaseV_Curr);
    ADCSampPare.PhaseU_Curr = -(ADC_ConvertedValue[2] - ADCSampPare.OffsetPhaseU_Curr);
    ADCSampPare.RP_speed_Voltage = ADC_ConvertedValue[3];
    ADCSampPare.BUS_Voltage = ADC_ConvertedValue[4];
    // һ�׵�ͨ�����˲��� ��ʽ�����װٶ�  ��ʷ��0.96  �ɼ���ֵ0.04  (24/1024) / (2*3.14*0.000083)=46 hz  cutfreq
    ADCSampPare.BUS_CurrF = _IQ10mpy(ADCSampPare.Coeff_filterK1, ADCSampPare.BUS_CurrF) + _IQ10mpy(
                                ADCSampPare.Coeff_filterK2, ADCSampPare.BUS_Curr);
}


void Protection_software(void) //ͨ��ĸ�ߵ���ֵ����Ӳ�� ������� 3A����Ϊ600
{
    if (Abs(ADCSampPare.BUS_CurrF) > BUS_CurrProtection) // ��ԼΪ3A
    {
        Stop_Motor(); // �رյ��ֹͣ
        logicContr.Start_order = 0;
    }
}

//===========================================================================
// No more.
//===========================================================================
