#include "Uart.h"

bit busy;

/*----------------------------
���ʹ�������
----------------------------*/
void SendData(BYTE dat)
{
	while (busy); //�ȴ�ǰ������ݷ������
	ACC = dat; //��ȡУ��λP (PSW.0)
	busy = 1;
	SBUF = ACC; //д���ݵ�UART���ݼĴ���
}

/*----------------------------
�����ַ���
----------------------------*/
void UartSendStr(char *s)
{
	while (*s) //����ַ���������־
	{
		SendData(*s++); //���͵�ǰ�ַ�
	}
}
void UartInit(void)
{
	SCON = 0x50; //8λ�ɱ䲨����
	T2L = (65536 - (FOSC / 4 / BAUD));
	T2H = (65536 - (FOSC / 4 / BAUD)) >> 8;
	AUXR |= 0X14;
	AUXR |= 0X01;

}
