#ifndef UART_H
#define UART_H
#include "alldef.h"

extern bit busy;

/*----------------------------
发送串口数据
----------------------------*/
void SendData(BYTE dat);

/*----------------------------
发送字符串
----------------------------*/
void UartSendStr(char *s);
void UartInit(void);

#endif