#ifndef _IMU_H_
#define _IMU_H_

extern int data AngleX,AngleY;
/**
 * IMU���º���
 *
 * @author KingBoy (2018/5/20)
 *
 * @param gx ����x
 * @param gy ����y
 * @param gz ����z
 * @param ax ���ٶȼ�x
 * @param ay ���ٶȼ�y
 * @param az ���ٶȼ�z
 */
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
/**
 * ���ټ���1/Sqrt(x)
 *
 * @author KingBoy (2018/5/20)
 *
 * @param x ����x
 *
 * @return float ���1/sqrt(x)
 */
float invSqrt(float x);
#endif  