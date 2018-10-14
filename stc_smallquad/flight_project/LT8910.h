#ifndef _LT8910_H
#define _LT8910_H

#include "alldef.h"
extern uchar RegH;
extern uchar RegL;


#include "alldef.h"
/**
 * us级延时函数
 *
 * @author KingBoy (2018/5/20)
 *
 * @param n us
 */
void Delay_us(uint n);
/**
 * 写寄存器
 *
 * @author KingBoy (2018/5/20)
 *
 * @param addr 写入的地址
 * @param H 高字节
 * @param L 低字节
 */
void SPI_WriteReg(uchar addr, uchar H, uchar L);
/**
 * 读寄存器
 *
 * @author KingBoy (2018/5/20)
 *
 * @param addr 要读取的地址
 */
void SPI_ReadReg(uchar addr);
/**
 * LT8910初始化
 *
 * @author KingBoy (2018/5/20)
 *
 * @param void
 */
void LT8910_Init(void);
/**
 * 2.4G无线模块LT8910设置为接收模式
 * 便宜货
 *
 * @author KingBoy (2018/5/20)
 *
 * @param void
 */
void RX_model(void);
/**
 * 接收数据
 *
 * @author KingBoy (2018/5/20)
 *
 * @param rx_buf 接收缓存
 * @param Num 数据个数(ushort?)
 */
void ReceiveRC_Data(uchar *rx_buf, uchar Num);
/**
 * 通讯校验函数
 *
 * @author KingBoy (2018/10/8)
 *
 * @param buf 待计算校验的数组
 * @param len 待计算校验的数组长度
 * @param mac 需要比对的校验值
 *
 * @return int 0 为一致，非0为计算的校验结果
 */
int MAC_calc(unsigned char * buff,int len, unsigned char mac);
#endif

