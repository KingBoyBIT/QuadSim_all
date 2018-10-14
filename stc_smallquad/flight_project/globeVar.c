#include "globeVar.h"

/*ȫ�ֱ�������*/
unsigned char data RxBuf[20];		//���ý��ճ��ȣ����Ϊ32�ֽ�

/*�ɿؿ��Ʋ���*/
unsigned int Ctl_throttle = 0;		//���ű仯�ٶȿ��ƣ����������Ļ����ٱ仯����ʱ�����ʧ�ٷ�ת��׹��
/*����ٶȲ���*/
int speed0 = 0;
int speed1 = 0;
int speed2 = 0;
int speed3 = 0;
/*������PWMģ��Ĳ���*/
int PWM0 = 0;
int PWM1 = 0;
int PWM2 = 0;
int PWM3 = 0;

//double g_x=0,g_y=0,g_z=0;				//�����ǽ�������
/*�ǶȽ�������*/
char rcAngle_X_offset = 0;
char rcAngle_Y_offset = 0;
char rcAngle_Z_offset = 0;

float IMU_gz;//����δ֪

/*MPU-6050�Ĵ������ݲ���*/
double Gyro_y = 0;						//Y�������������ݴ�
double Gyro_x = 0;						//X�������������ݴ�
double Gyro_z = 0;						//Z�������������ݴ�
int xdata Angle_ax = 0;					//�ɼ��ٶȼ���ļ��ٶ�(������)
int xdata Angle_ay = 0;					//�ɼ��ٶȼ���ļ��ٶ�(������)
int xdata Angle_az = 0;					//�ɼ��ٶȼ���ļ��ٶ�(������)
int idata Omega_gy = 0;					//�ɽ��ٶȼ���Ľ�����(�Ƕ���)
int idata Omega_gx = 0;					//�ɽ��ٶȼ���Ľ�����(�Ƕ���)
int idata Omega_gz = 0;					//�ɽ��ٶȼ���Ľ�����(�Ƕ���)

int data AngleXest = 0, AngleYest = 0;		//��Ԫ���������ŷ����  ,AngleZ=0

/*acc16*3+tempreture16+gyro16*3ֱ�Ӷ�ȡMPU6050�����Ǻͼ��ٶȵ����ݰ����õ�14�ֽڣ�*/
unsigned char data IMUdata[16];				
 

/*���п��Ʊ���*/
unsigned char LockState;				//�Ͽ�/���� ��������
unsigned char RCnum;					//ͨѶ״̬ ����
unsigned char RCnum_pre;					//ң����ָ����ˮ��
unsigned char LostComCount;				//ʧ����������
unsigned int rc_throttle;					//���ű���
unsigned int rc_Yaw;						//�������
unsigned int rc_Roll;			   			//�������
unsigned int rc_Pitch;			   			//��������
//unsigned char PitchRoll;	   			//�����������
unsigned char CKV_calc;		   			//CRC ���Ч���
unsigned char CKV_match;	   			//CRC ����Ч���

/*PID�㷨����*/
float data PID_Output;					//PID���������

float xdata Last_Angle_gx = 0;			//�⻷PI�����  ��һ������������
float xdata Last_Omega_gy = 0;			//�⻷PI�����  ��һ������������

float xdata integOmegaErr_X = 0;				//�ڻ�P �ڻ�I �ڻ�D �ڻ�������
float xdata integAngleErr_X = 0;		//�⻷P �⻷I       �⻷������

float xdata integOmegaErr_Y = 0;				//�ڻ�P �ڻ�I �ڻ�D �ڻ�������
float xdata integAngleErr_Y = 0;		//�⻷P �⻷I       �⻷������

float xdata integAngleErr_Z = 0;		//�⻷P �⻷I       �⻷������
float xdata LastAngle_Z = 0;			//Z�����??

float xdata LastAngleErr_X = 0;				//�Ƕȸ���
float xdata LastAngleErr_Y = 0;				//�Ƕȸ���

float xdata LastOmegaErr_X = 0;				//���ٶȸ���
float xdata LastOmegaErr_Y = 0;				//���ٶȸ���

float xdata AngleErr_X = 0, AngleErr_Y = 0, AngleErr_Z = 0;		//����ң������������ĽǶ�
int xdata OmegaErr_X = 0, OmegaErr_Y = 0;				//����ң������������Ľ��ٶ�
//long xdata g_x_aver = 0;				//û�õ���
//long xdata g_y_aver = 0;				//û�õ���
//long xdata g_z_aver = 0;				//û�õ���
int delta_rc_x = 0;						//�ݲ�ȷ����������
int delta_rc_y = 0;						//�ݲ�ȷ����������
int delta_rc_z = 0;						//�ݲ�ȷ����������
long idata PID_P;						//PID������ʱ����ֵ
long idata PID_I;						//PID������ʱ����ֵ
long idata PID_D;						//PID������ʱ����ֵ

