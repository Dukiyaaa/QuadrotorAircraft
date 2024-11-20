#ifndef __MYSERIAL_H
#define __MYSERIAL_H
#include "main.h"
#include "stdio.h"

void MySerial_Init(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
int fputc(int ch ,FILE *f);
void MySerial_ReceiveData(void);
void UART_RX_PROC(void);
#endif
