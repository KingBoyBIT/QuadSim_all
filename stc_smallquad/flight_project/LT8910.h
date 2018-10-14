#ifndef _LT8910_H
#define _LT8910_H

#include "alldef.h"
extern uchar RegH;
extern uchar RegL;


#include "alldef.h"
/**
 * us����ʱ����
 *
 * @author KingBoy (2018/5/20)
 *
 * @param n us
 */
void Delay_us(uint n);
/**
 * д�Ĵ���
 *
 * @author KingBoy (2018/5/20)
 *
 * @param addr д��ĵ�ַ
 * @param H ���ֽ�
 * @param L ���ֽ�
 */
void SPI_WriteReg(uchar addr, uchar H, uchar L);
/**
 * ���Ĵ���
 *
 * @author KingBoy (2018/5/20)
 *
 * @param addr Ҫ��ȡ�ĵ�ַ
 */
void SPI_ReadReg(uchar addr);
/**
 * LT8910��ʼ��
 *
 * @author KingBoy (2018/5/20)
 *
 * @param void
 */
void LT8910_Init(void);
/**
 * 2.4G����ģ��LT8910����Ϊ����ģʽ
 * ���˻�
 *
 * @author KingBoy (2018/5/20)
 *
 * @param void
 */
void RX_model(void);
/**
 * ��������
 *
 * @author KingBoy (2018/5/20)
 *
 * @param rx_buf ���ջ���
 * @param Num ���ݸ���(ushort?)
 */
void ReceiveRC_Data(uchar *rx_buf, uchar Num);
/**
 * ͨѶУ�麯��
 *
 * @author KingBoy (2018/10/8)
 *
 * @param buf ������У�������
 * @param len ������У������鳤��
 * @param mac ��Ҫ�ȶԵ�У��ֵ
 *
 * @return int 0 Ϊһ�£���0Ϊ�����У����
 */
int MAC_calc(unsigned char * buff,int len, unsigned char mac);
#endif

