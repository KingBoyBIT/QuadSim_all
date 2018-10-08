#ifndef _MPU_6050_H
#define _MPU_6050_H

#include "alldef.h"
/**
 * 延时2us 
 *
 * @author KingBoy (2018/5/20)
 */
void Delay2us(void);
/**
 * I2C起始信号
 *
 * @author KingBoy (2018/5/20)
 *
 * @param void
 */
void I2C_Start(void);
/**
 * I2C停止信号
 *
 * @author KingBoy (2018/5/20)
 *
 * @param void
 */
void I2C_Stop(void);
/**
 * I2C接收应答信号
 *
 * @author KingBoy (2018/5/20)
 *
 * @param void
 *
 * @return bit ACK
 */
bit I2C_RecvACK(void);
/**
 * 向I2C总线发送一个字节数据
 *
 * @author KingBoy (2018/5/20)
 *
 * @param dat 输入数据1字节
 */
void I2C_SendByte(unsigned char dat);
/**
 * 从I2C总线接收一个字节数据
 *
 * @author KingBoy (2018/5/20)
 *
 * @return unsigned char 输出，读取到的字节
 */
unsigned char I2C_RecvByte(void);
/**
 * 向I2C设备写入一个字节数据
 *
 * @author KingBoy (2018/5/20)
 *
 * @param REG_Address 输入：地址
 * @param REG_data 输入：数据
 */
void Single_WriteI2C(unsigned char REG_Address, unsigned char REG_data);
/**
 * 从I2C设备读取一个字节数据
 *
 * @author KingBoy (2018/5/20)
 *
 * @param REG_Address 输入：地址
 *
 * @return unsigned char 输出：读到的数据
 */
unsigned char Single_ReadI2C(unsigned char REG_Address);
/**
 * 初始化MPU6050
 * @brief 解除休眠状态
 * 陀螺仪125hz 21HZ滤波延时A8.5ms G8.3ms
 * 此处取值应相当注意，延时与系统周期相近为宜
 * 陀螺仪500度/S 65.5LSB/g
 * 加速度+-16g  8192LSB/g
 *
 * @author KingBoy (2018/5/20)
 *
 * @param void
 */
void Init_MPU6050(void);
/**
 * 合成数据
 *
 * @author KingBoy (2018/5/20)
 *
 * @param REG_Address 从这个地址读数据
 *
 * @return int uint8合成为int16
 */
int GetData(unsigned char REG_Address);
/**
 * 直接从6050读取数据
 *
 * @author KingBoy (2018/5/20)
 *
 * @param buf
 */
void Read_MPU6050(unsigned char *buf);

#endif
