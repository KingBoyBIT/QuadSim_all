#ifndef _MPU_6050_H
#define _MPU_6050_H

#include "alldef.h"
/**
 * ��ʱ2us 
 *
 * @author KingBoy (2018/5/20)
 */
void Delay2us(void);
/**
 * I2C��ʼ�ź�
 *
 * @author KingBoy (2018/5/20)
 *
 * @param void
 */
void I2C_Start(void);
/**
 * I2Cֹͣ�ź�
 *
 * @author KingBoy (2018/5/20)
 *
 * @param void
 */
void I2C_Stop(void);
/**
 * I2C����Ӧ���ź�
 *
 * @author KingBoy (2018/5/20)
 *
 * @param void
 *
 * @return bit ACK
 */
bit I2C_RecvACK(void);
/**
 * ��I2C���߷���һ���ֽ�����
 *
 * @author KingBoy (2018/5/20)
 *
 * @param dat ��������1�ֽ�
 */
void I2C_SendByte(unsigned char dat);
/**
 * ��I2C���߽���һ���ֽ�����
 *
 * @author KingBoy (2018/5/20)
 *
 * @return unsigned char �������ȡ�����ֽ�
 */
unsigned char I2C_RecvByte(void);
/**
 * ��I2C�豸д��һ���ֽ�����
 *
 * @author KingBoy (2018/5/20)
 *
 * @param REG_Address ���룺��ַ
 * @param REG_data ���룺����
 */
void Single_WriteI2C(unsigned char REG_Address, unsigned char REG_data);
/**
 * ��I2C�豸��ȡһ���ֽ�����
 *
 * @author KingBoy (2018/5/20)
 *
 * @param REG_Address ���룺��ַ
 *
 * @return unsigned char ���������������
 */
unsigned char Single_ReadI2C(unsigned char REG_Address);
/**
 * ��ʼ��MPU6050
 * @brief �������״̬
 * ������125hz 21HZ�˲���ʱA8.5ms G8.3ms
 * �˴�ȡֵӦ�൱ע�⣬��ʱ��ϵͳ�������Ϊ��
 * ������500��/S 65.5LSB/g
 * ���ٶ�+-16g  8192LSB/g
 *
 * @author KingBoy (2018/5/20)
 *
 * @param void
 */
void Init_MPU6050(void);
/**
 * �ϳ�����
 *
 * @author KingBoy (2018/5/20)
 *
 * @param REG_Address �������ַ������
 *
 * @return int uint8�ϳ�Ϊint16
 */
int GetData(unsigned char REG_Address);
/**
 * ֱ�Ӵ�6050��ȡ����
 *
 * @author KingBoy (2018/5/20)
 *
 * @param buf
 */
void Read_MPU6050(unsigned char *buf);

#endif