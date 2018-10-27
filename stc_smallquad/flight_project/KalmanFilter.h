#ifndef _KalmanFilter_H_
#define _KalmanFilter_H_
/**
 * һ�׿������˲����˻�Ϊ�����˲�
 *
 * @author KingBoy (2018/10/27)
 *
 * @param FilteredData ������˲����ֵ
 * @param ResrcData ���룺���˲�ֵ
 * @param Q ���룺��������Q
 * @param R ���룺��������R
 * @param axis ���룺0-x,1-y,2-z
 *
 * @return int 0-success
 */
int RCLowPassFilter_Acc(int *FilteredData, int ResrcData, int Q, int R, unsigned char axis);

//int KalmanFilter_gyrox( int ResrcData,int ProcessNiose_Q,int MeasureNoise_R);
//int KalmanFilter_gyroy( int ResrcData,int ProcessNiose_Q,int MeasureNoise_R);
int RCLowPassFilter_gz(int ResrcData, int ProcessNiose_Q, int MeasureNoise_R);

#endif
