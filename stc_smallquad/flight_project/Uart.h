#ifndef UART_H
#define UART_H
#include "alldef.h"

extern bit busy;

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
void UartInit(unsigned long baud,unsigned long fosc);
/**
 * �򴮿ڷ���һ���ֽ�
 *
 * @author KingBoy (2018/10/15)
 *
 * @param dat �������ֽ�
 * @param flag 1Ϊ�����ַ���
 */
void UartSendByte(BYTE dat, BYTE flag);
/**
 * �����ַ���
 *
 * @author KingBoy (2018/10/15)
 *
 * @param s ��\0����
 */
void UartSendStr(char *s);


#endif