#ifndef UART_H
#define UART_H
#include "alldef.h"

extern bit busy;

/*----------------------------
���ʹ�������
----------------------------*/
void SendData(BYTE dat);

/*----------------------------
�����ַ���
----------------------------*/
void UartSendStr(char *s);
void UartInit(void);

#endif