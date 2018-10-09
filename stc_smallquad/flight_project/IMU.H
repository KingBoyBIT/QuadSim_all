#ifndef _IMU_H_
#define _IMU_H_

extern int data AngleXest,AngleY;
/**
 * IMU更新函数
 *
 * @author KingBoy (2018/5/20)
 *
 * @param gx 陀螺x
 * @param gy 陀螺y
 * @param gz 陀螺z
 * @param ax 加速度计x
 * @param ay 加速度计y
 * @param az 加速度计z
 */
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
/**
 * 快速计算1/Sqrt(x)
 *
 * @author KingBoy (2018/5/20)
 *
 * @param x 输入x
 *
 * @return float 输出1/sqrt(x)
 */
float invSqrt(float x);
#endif  