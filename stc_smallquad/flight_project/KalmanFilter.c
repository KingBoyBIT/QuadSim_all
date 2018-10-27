#include <STC15W4K48S4.h>	//STC15W4K48S4 专用头文件
#include "KalmanFilter.h"  //卡尔曼滤波算法

/**
 * 一阶卡尔曼滤波，退化为惯性滤波
 *
 * @author KingBoy (2018/10/27)
 *
 * @param FilteredData 输出：滤波后的值
 * @param ResrcData 输入：待滤波值
 * @param Q 输入：噪声矩阵Q
 * @param R 输入：噪声矩阵R
 * @param axis 输入：0-x,1-y,2-z
 *
 * @return int 0-success
 */
int RCLowPassFilter_Acc(int *FilteredData, int ResrcData, int Q, int R, unsigned char axis)
{
	static int ax_last; //static 函数退出后变量值保留
	static int ax_p_last;
	static int ay_last; //static 函数退出后变量值保留
	static int ay_p_last;
	static int az_last; //static 函数退出后变量值保留
	static int az_p_last;

	int a_mid;
	long a_now;

	long p_mid;
	long p_now;
	int kg;
	long temp;
	switch (axis)
	{
		case 0:
			a_mid = ax_last;
			p_mid = ax_p_last + Q;
			break;
		case 1:
			a_mid = ay_last;
			p_mid = ay_p_last + Q;
			break;
		case 2:
			a_mid = az_last;
			p_mid = az_p_last + Q;
			break;
		default:
			return (-1);
	}
	temp = p_mid << 15;
	kg = (temp / ((long)p_mid + R));
	a_now = a_mid + (((long)kg * (ResrcData - a_mid)) >> 15);
	p_now = ((long)p_mid * (32768 - kg)) >> 15;
	switch (axis)
	{
		case 0:
			ax_p_last = p_now;
			ax_last = a_now;
			break;
		case 1:
			ay_p_last = p_now;
			ay_last = a_now;
			break;
		case 2:
			az_p_last = p_now;
			az_last = a_now;
			break;
		default:
			return (-1);
	}
	*FilteredData = a_now;

	return (0);
}



#if 0
int KalmanFilter_gyrox(int ResrcData, int ProcessNiose_Q, int MeasureNoise_R)
{
	int R = MeasureNoise_R;
	int Q = ProcessNiose_Q;
	static int gyrox_last;
	int gyrox_mid = gyrox_last;
	long gyrox_now;
	static int gyrox_p_last;
	long gyrox_p_mid;
	long gyrox_p_now;
	int gyrox_kg;
	long gyrox_temp;
	gyrox_mid = gyrox_last;
	gyrox_p_mid = gyrox_p_last + Q;
	gyrox_temp = gyrox_p_mid << 15;
	gyrox_kg = (gyrox_temp / ((long)gyrox_p_mid + R));

	gyrox_now = gyrox_mid + (((long)gyrox_kg * (ResrcData - gyrox_mid)) >> 15);
	gyrox_p_now = ((long)gyrox_p_mid * (32768 - gyrox_kg)) >> 15;
	gyrox_p_last = gyrox_p_now;
	gyrox_last = gyrox_now;
	return (gyrox_now);
}
int KalmanFilter_gyroy(int ResrcData, int ProcessNiose_Q, int MeasureNoise_R)
{
	int R = MeasureNoise_R;
	int Q = ProcessNiose_Q;
	static int gyroy_last;
	int gyroy_mid = gyroy_last;
	long gyroy_now;
	static int gyroy_p_last;
	long p_mid;
	long p_now;
	int kg;
	long temp;
	gyroy_mid = gyroy_last;
	p_mid = gyroy_p_last + Q;
	temp = p_mid << 15;
	kg = (temp / ((long)p_mid + R));

	gyroy_now = gyroy_mid + (((long)kg * (ResrcData - gyroy_mid)) >> 15);
	p_now = ((long)p_mid * (32768 - kg)) >> 15;
	gyroy_p_last = p_now;
	gyroy_last = gyroy_now;
	return (gyroy_now);
}
#endif
int RCLowPassFilter_gz(int ResrcData, int ProcessNiose_Q, int MeasureNoise_R)
{
	int R = MeasureNoise_R;
	int Q = ProcessNiose_Q;
	static int gyroz_last;
	int gyroz_mid = gyroz_last;
	long gyroz_now;
	static int gyroz_p_last;
	long p_mid;
	long p_now;
	int kg;
	long temp;
	gyroz_mid = gyroz_last;
	p_mid = gyroz_p_last + Q;
	temp = p_mid << 15;
	kg = (temp / ((long)p_mid + R));

	gyroz_now = gyroz_mid + (((long)kg * (ResrcData - gyroz_mid)) >> 15);
	p_now = ((long)p_mid * (32768 - kg)) >> 15;
	gyroz_p_last = p_now;
	gyroz_last = gyroz_now;
	return (gyroz_now);
}

