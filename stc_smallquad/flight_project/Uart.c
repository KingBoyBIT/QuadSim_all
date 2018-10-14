#include "Uart.h"

bit busy;

/*----------------------------
发送串口数据
----------------------------*/
void SendData(BYTE dat)
{
	while (busy); //等待前面的数据发送完成
	ACC = dat; //获取校验位P (PSW.0)
	busy = 1;
	SBUF = ACC; //写数据到UART数据寄存器
}

/*----------------------------
发送字符串
----------------------------*/
void UartSendStr(char *s)
{
	while (*s) //检测字符串结束标志
	{
		SendData(*s++); //发送当前字符
	}
}
void UartInit(void)
{
	SCON = 0x50; //8位可变波特率
	T2L = (65536 - (FOSC / 4 / BAUD));
	T2H = (65536 - (FOSC / 4 / BAUD)) >> 8;
	AUXR |= 0X14;
	AUXR |= 0X01;

}
