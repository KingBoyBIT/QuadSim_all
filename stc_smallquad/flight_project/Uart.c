#include "Uart.h"

bit busy;

/**
 * ���ڳ�ʼ����
 * Ĭ�ϲ�����115200
 * Ĭ��ϵͳ��Ƶ24M
 *
 * @author KingBoy (2018/10/15)
 *
 * @param baud ������
 * @param fosc ��Ƶ
 */
void UartInit(unsigned int baud,unsigned int fosc)
{
	SCON = 0x50; //8λ�ɱ䲨����
	T2L = (65536 - (fosc / 4 / baud));
	T2H = (65536 - (fosc / 4 / baud)) >> 8;
	AUXR |= 0X14;
	AUXR |= 0X01;
}

/**
 * �򴮿ڷ���һ���ֽ�
 *
 * @author KingBoy (2018/10/15)
 *
 * @param dat �������ֽ�
 */
void UartSendByte(BYTE dat)
{
	while (busy); //�ȴ�ǰ������ݷ������
	ACC = dat; //��ȡУ��λP (PSW.0)
	busy = 1;
	SBUF = ACC; //д���ݵ�UART���ݼĴ���
}

/**
 * �����ַ���
 *
 * @author KingBoy (2018/10/15)
 *
 * @param s ��\0����
 */
void UartSendStr(char *s)
{
	while (*s) //����ַ���������־
	{
		UartSendByte(*s++); //���͵�ǰ�ַ�
	}
}

