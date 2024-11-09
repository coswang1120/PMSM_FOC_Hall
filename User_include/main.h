#ifndef __MAIN_H
#define __MAIN_H
#include "stm32f10x.h"
#include <stdio.h>
#define UART_RX_BUFFER_SIZE 128
extern char uart_rx_buffer[UART_RX_BUFFER_SIZE];
extern volatile uint16_t uart_rx_write_ptr;
extern volatile uint8_t uart_rx_line_complete;

int fputc( int ch , FILE *f);

void process_uart_command(void);




#endif /* __MAIN_H */
