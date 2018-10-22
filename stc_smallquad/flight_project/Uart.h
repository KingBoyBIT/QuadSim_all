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
void UartInit(unsigned long baud,unsigned long fosc);
/**
 * 向串口发送一个字节
 *
 * @author KingBoy (2018/10/15)
 *
 * @param dat 待发送字节
 * @param flag 1为发送字符串
 */
void UartSendByte(BYTE dat, BYTE flag);
/**
 * 发送字符串
 *
 * @author KingBoy (2018/10/15)
 *
 * @param s 以\0结束
 */
void UartSendStr(char *s);


#endif