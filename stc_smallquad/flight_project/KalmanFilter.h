#ifndef _KalmanFilter_H_
#define _KalmanFilter_H_
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
int RCLowPassFilter_Acc(int *FilteredData, int ResrcData, int Q, int R, unsigned char axis);

//int KalmanFilter_gyrox( int ResrcData,int ProcessNiose_Q,int MeasureNoise_R);
//int KalmanFilter_gyroy( int ResrcData,int ProcessNiose_Q,int MeasureNoise_R);
int RCLowPassFilter_gz(int ResrcData, int ProcessNiose_Q, int MeasureNoise_R);

#endif
