#ifndef _KalmanFilter_H_
#define _KalmanFilter_H_
int RCLowPassFilter_ax(int ResrcData, int ProcessNiose_Q, int MeasureNoise_R);
int RCLowPassFilter_ay(int ResrcData, int ProcessNiose_Q, int MeasureNoise_R);
int RCLowPassFilter_az(int ResrcData, int ProcessNiose_Q, int MeasureNoise_R);
//int KalmanFilter_gyrox( int ResrcData,int ProcessNiose_Q,int MeasureNoise_R);
//int KalmanFilter_gyroy( int ResrcData,int ProcessNiose_Q,int MeasureNoise_R);
int RCLowPassFilter_gz(int ResrcData, int ProcessNiose_Q, int MeasureNoise_R);

#endif
