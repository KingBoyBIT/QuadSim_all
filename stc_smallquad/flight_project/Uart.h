#ifndef UART_H
#define UART_H
#include "alldef.h"

extern bit busy;

/**
 * 串口初始化：
 * 默认波特率115200
 * 默认系统主频24M
 *
 * @author KingBoy (2018/10/15)
 *
 * @param baud 波特率
 * @param fosc 主频
 */
void UartInit(unsigned int baud,unsigned int fosc);
/**
 * 向串口发送一个字节
 *
 * @author KingBoy (2018/10/15)
 *
 * @param dat 待发送字节
 */
void SendData(BYTE dat);
/**
 * 发送字符串
 *
 * @author KingBoy (2018/10/15)
 *
 * @param s 以\0结束
 */
void UartSendStr(char *s);


#endif