#ifndef _GLOBE_VAR_H_
#define _GLOBE_VAR_H_

/*ȫ�ֱ�������*/
extern int ret;
extern unsigned char data RxBuf[20]; //���ý��ճ��ȣ����Ϊ32�ֽ�

/*�ɿؿ��Ʋ���*/
extern unsigned int Ctl_throttle;//���ű仯�ٶȿ��ƣ����������Ļ����ٱ仯����ʱ�����ʧ�ٷ�ת��׹��
/*����ٶȲ���*/
extern int speed0;
extern int speed1;
extern int speed2;
extern int speed3;
/*������PWMģ��Ĳ���*/
extern int PWM0;
extern int PWM1;
extern int PWM2;
extern int PWM3;

//double g_x=0,g_y=0,g_z=0;       //�����ǽ�������
/*�ǶȽ�������*/
extern char rcAngle_X_offset;
extern char rcAngle_Y_offset;
extern char rcAngle_Z_offset;

extern float IMU_gz;//����δ֪

/*MPU-6050�Ĵ������ݲ���*/
#if 0
extern double Gyro_y; //Y�������������ݴ�
extern double Gyro_x; //X�������������ݴ�
extern double Gyro_z; //Z�������������ݴ�
#endif
extern int xdata Angle_a[3];//�ɼ��ٶȼ���ļ��ٶ�(������)
extern int idata Omega_g[3];				//�ɽ��ٶȼ���Ľ�����(�Ƕ���)


extern int data AngleXest, AngleYest;//��Ԫ���������ŷ����  ,AngleZ=0

extern unsigned char data IMUdata[16];//acc16*3+tempreture16+gyro16*3ֱ�Ӷ�ȡMPU6050�����Ǻͼ��ٶȵ����ݰ����õ�14�ֽڣ�
 

/*���п��Ʊ���*/
extern unsigned char LockState;//�Ͽ�/���� ��������
extern unsigned char RCnum;//ͨѶ״̬ ����
extern unsigned char RCnum_pre;//ʧ������
extern unsigned char LostComCount;//ʧ����������
extern unsigned int rc_throttle;//���ű���
extern unsigned int rc_Yaw;//�������
extern unsigned int rc_Roll;//�������
extern unsigned int rc_Pitch;//��������
//extern unsigned char PitchRoll;//�����������
extern unsigned char CKV_calc;//CRC ���Ч���
extern unsigned char CKV_match;//CRC ����Ч���

/*PID�㷨����*/
extern float data PID_Output;//PID���������

extern float xdata Last_Angle_gx;//�⻷PI�����  ��һ������������
extern float xdata Last_Omega_gy;//�⻷PI�����  ��һ������������

extern float xdata integOmegaErr_X;//�ڻ�P �ڻ�I �ڻ�D �ڻ�������
extern float xdata integAngleErr_X;//�⻷P �⻷I       �⻷������

extern float xdata integOmegaErr_Y;//�ڻ�P �ڻ�I �ڻ�D �ڻ�������
extern float xdata integAngleErr_Y;//�⻷P �⻷I       �⻷������

extern float xdata integAngleErr_Z;//�⻷P �⻷I       �⻷������
extern float xdata LastAngle_Z;//Z�����??

extern float xdata LastAngleErr_X;//�Ƕȸ���
extern float xdata LastAngleErr_Y;//�Ƕȸ���

extern float xdata LastOmegaErr_X;//���ٶȸ���
extern float xdata LastOmegaErr_Y;//���ٶȸ���

extern float xdata AngleErr[3];		//[x,y,z]����ң������������ĽǶ� Zδ�õ�

extern int xdata OmegaErr_X, OmegaErr_Y;//����ң������������Ľ��ٶ�
//extern long xdata g_x_aver;//û�õ���
//extern long xdata g_y_aver;//û�õ���
//extern long xdata g_z_aver;//û�õ���
extern int delta_rc[3];					//[x,y,z] �ݲ�ȷ����������

extern long idata PID_P;//�ݲ�ȷ����������
extern long idata PID_I;//�ݲ�ȷ����������
extern long idata PID_D;//�ݲ�ȷ����������


#endif
