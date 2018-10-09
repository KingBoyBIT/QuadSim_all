//#include <STC15W4K48S4.h>	//STC15W4K48S4 专用头文件
#include <math.H>
#include "IMU.H"
#include "alldef.h"


float idata q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float idata exInt = 0, eyInt = 0, ezInt = 0;
float idata exDif = 0, eyDif = 0, ezDif = 0;

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
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
	float idata norm;
	float idata vx, vy, vz;
	float idata ex, ey, ez;
	static float idata last_ex, last_ey, last_ez;

#if 0
	norm = sqrt(ax * ax + ay * ay + az * az); //把加速度计的三维向量转成单维向量
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;
#endif
	//使用快速计算，后续考虑避免乘除运算
	norm = invSqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	/*
dcm(1,3,:) = 2.*(qin(:,2).*qin(:,4) - qin(:,1).*qin(:,3));
dcm(2,3,:) = 2.*(qin(:,3).*qin(:,4) + qin(:,1).*qin(:,2));
dcm(3,3,:) = qin(:,1).^2 - qin(:,2).^2 - qin(:,3).^2 + qin(:,4).^2;
	 */
	//DCM的第三列
	vx = 2 * (q1 * q3 - q0 * q2);
	vy = 2 * (q0 * q1 + q2 * q3);
	vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

	ex = (ay * vz - az * vy);
	ey = (az * vx - ax * vz);
	ez = (ax * vy - ay * vx);

	exInt = exInt + ex * NAV_PARA_EST_KI;
	eyInt = eyInt + ey * NAV_PARA_EST_KI;
	ezInt = ezInt + ez * NAV_PARA_EST_KI;


	exDif = ex - last_ex;
	eyDif = ey - last_ey;
	ezDif = ez - last_ez;

	last_ex = ex;
	last_ey = ey;
	last_ez = ez;

	gx = gx + NAV_PARA_EST_KP * ex + exInt + NAV_PARA_EST_KD * exDif;
	gy = gy + NAV_PARA_EST_KP * ey + eyInt + NAV_PARA_EST_KD * eyDif;
	gz = gz + NAV_PARA_EST_KP * ez + ezInt + NAV_PARA_EST_KD * ezDif;


	q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * NAV_PARA_EST_HALF_T;
	q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * NAV_PARA_EST_HALF_T;
	q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * NAV_PARA_EST_HALF_T;
	q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * NAV_PARA_EST_HALF_T; 

#if 0
	norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
#endif

	//快速计算
	norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
	AngleY = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * MATH_180_PI_10;
	AngleXest = asin(2 * (q0 * q2 - q1 * q3)) * MATH_180_PI_10;
	//AngleZ = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1)* MATH_180_PI_10;

}

/**
 * 快速计算1/Sqrt(x)
 *
 * @author KingBoy (2018/5/20)
 *
 * @param x 输入x
 *
 * @return float 输出1/sqrt(x)
 */
float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	return (y);
}
