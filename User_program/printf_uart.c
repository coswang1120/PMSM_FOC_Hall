#include "printf_uart.h"
#include "main.h"

#if !defined(OS_USE_SEMIHOSTING)

USART_TypeDef *G_uart;
#define STDIN_FILE_NO 0
#define STDOUT_FILE_NO 1
#define STDERR_FILE_NO 2

// 實作 fputc
int fputc(int ch, FILE *f) {
    USART_SendData(USART3, (uint8_t)ch);
    while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET) {
    }
    return ch;
}



void PrintfInit(USART_TypeDef *uarTx) {
    G_uart = uarTx;
    /* Disable I/O buffering for STDOUT stream, so that
     * chars are sent out as soon as they are printed. */
    setvbuf(stdout, NULL, _IONBF, 0);
}

void Uart3Init(u32 bound) {
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);   //使能USART1，GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB ,ENABLE);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; /* USART3 Tx (PB.10)*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /******************************************************************/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; /* USART3 Rx (PB.11)*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//抢占优先级2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        //子优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);    //根据指定的参数初始化VIC寄存器

    //USART 初始化设置
    USART_InitStructure.USART_BaudRate = bound;//串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    //收发模式

    USART_Init(USART3, &USART_InitStructure); //初始化串口1
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
    USART_Cmd(USART3, ENABLE);                                        //使能串口1
}


int _isatty(int fd) {
    if (fd >= STDIN_FILE_NO && fd <= STDERR_FILE_NO)
        return 1;
    return 0;
}

int _write(int fd, char *ptr, int len) {
    if (fd == STDOUT_FILE_NO || fd == STDERR_FILE_NO) {
        uint32_t i;
        for (i = 0; i < len; i++) {
            USART_SendData(G_uart, ptr[i]);

            while (USART_GetFlagStatus(G_uart, USART_FLAG_TXE) == RESET);
        }
        while (USART_GetFlagStatus(G_uart, USART_FLAG_TC) == RESET);

        return len;

    }
    return -1;
}


int _close(int fd) {
    if (fd >= STDIN_FILE_NO && fd <= STDERR_FILE_NO)
        return 0;
    return -1;
}


int _lseek(int fd, int ptr, int dir) {
    (void) fd;
    (void) ptr;
    (void) dir;
    return -1;
}


int _read(int fd, char *ptr, int len) {
    if (fd == STDIN_FILE_NO) {
        while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET) {
        }
        *ptr = USART_ReceiveData(USART2);
        return 1;
    }
    return -1;
}

#endif //#if !defined(OS_USE_SEMIHOSTING)


void USART3_IRQHandler(void) {
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        char rx_data = USART_ReceiveData(USART3);
        
        // 檢查緩衝區是否還有空間
        if (uart_rx_write_ptr < UART_RX_BUFFER_SIZE - 1) {
            uart_rx_buffer[uart_rx_write_ptr++] = rx_data;
            
            // 檢查是否收到換行符
            if (rx_data == '\n' && uart_rx_write_ptr >= 2) {
                if (uart_rx_buffer[uart_rx_write_ptr-2] == '\r') {
                    uart_rx_line_complete = 1;
                }
            }
        } else {
            // 緩衝區滿，重置
            uart_rx_write_ptr = 0;
        }
    }
}
