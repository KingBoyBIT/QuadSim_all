#include "Uart.h"

bit busy;

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
void UartInit(unsigned int baud,unsigned int fosc)
{
	SCON = 0x50; //8位可变波特率
	T2L = (65536 - (fosc / 4 / baud));
	T2H = (65536 - (fosc / 4 / baud)) >> 8;
	AUXR |= 0X14;
	AUXR |= 0X01;
}

/**
 * 向串口发送一个字节
 *
 * @author KingBoy (2018/10/15)
 *
 * @param dat 待发送字节
 */
void UartSendByte(BYTE dat)
{
	while (busy); //等待前面的数据发送完成
	ACC = dat; //获取校验位P (PSW.0)
	busy = 1;
	SBUF = ACC; //写数据到UART数据寄存器
}

/**
 * 发送字符串
 *
 * @author KingBoy (2018/10/15)
 *
 * @param s 以\0结束
 */
void UartSendStr(char *s)
{
	while (*s) //检测字符串结束标志
	{
		UartSendByte(*s++); //发送当前字符
	}
}

