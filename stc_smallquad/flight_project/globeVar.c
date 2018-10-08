#include "globeVar.h"

/*ȫ�ֱ�������*/
unsigned char data RxBuf[20];		//���ý��ճ��ȣ����Ϊ32�ֽ�

/*�ɿؿ��Ʋ���*/
unsigned int d_throttle = 0;		//���ű仯�ٶȿ��ƣ����������Ļ����ٱ仯����ʱ�����ʧ�ٷ�ת��׹��
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
char a_x = 0;
char a_y = 0;
char a_z = 0;

float IMU_gz;//����δ֪

/*MPU-6050�Ĵ������ݲ���*/
double Gyro_y = 0;						//Y�������������ݴ�
double Gyro_x = 0;						//X�������������ݴ�
double Gyro_z = 0;						//Z�������������ݴ�
int xdata Angle_ax = 0;					//�ɼ��ٶȼ���ļ��ٶ�(������)
int xdata Angle_ay = 0;					//�ɼ��ٶȼ���ļ��ٶ�(������)
int xdata Angle_az = 0;					//�ɼ��ٶȼ���ļ��ٶ�(������)
int idata Angle_gy = 0;					//�ɽ��ٶȼ���Ľ�����(�Ƕ���)
int idata Angle_gx = 0;					//�ɽ��ٶȼ���Ľ�����(�Ƕ���)
int idata Angle_gz = 0;					//�ɽ��ٶȼ���Ľ�����(�Ƕ���)

int data AngleX = 0, AngleY = 0;		//��Ԫ���������ŷ����  ,AngleZ=0

/*acc16*3+tempreture16+gyro16*3ֱ�Ӷ�ȡMPU6050�����Ǻͼ��ٶȵ����ݰ����õ�14�ֽڣ�*/
unsigned char data IMUdata[16];				
 

/*���п��Ʊ���*/
unsigned char LockState;				//�Ͽ�/���� ��������
unsigned char LostCom;					//ͨѶ״̬ ����
unsigned char ShiLian;					//ʧ������
unsigned char ShiLianCount;				//ʧ����������
unsigned int throttle;					//���ű���
unsigned int Yaw;						//�������
unsigned int Roll;			   			//�������
unsigned int Pitch;			   			//��������
unsigned char PitchRoll;	   			//�����������
unsigned char CKV_calc;		   			//CRC ���Ч���
unsigned char CKV_match;	   			//CRC ����Ч���

/*PID�㷨����*/
float data PID_Output;					//PID���������

float xdata Last_Angle_gx = 0;			//�⻷PI�����  ��һ������������
float xdata Last_Angle_gy = 0;			//�⻷PI�����  ��һ������������

float xdata ErrORX_In = 0;				//�ڻ�P �ڻ�I �ڻ�D �ڻ�������
float xdata integAngleErr_X = 0;		//�⻷P �⻷I       �⻷������

float xdata ErrORY_In = 0;				//�ڻ�P �ڻ�I �ڻ�D �ڻ�������
float xdata integAngleErr_Y = 0;		//�⻷P �⻷I       �⻷������

float xdata integAngleErr_Z = 0;		//�⻷P �⻷I       �⻷������
float xdata AngleZ_late = 0;			//Z�����??

float xdata Last_Ax = 0;				//�Ƕȸ���
float xdata Last_Ay = 0;				//�Ƕȸ���

float xdata Last_gx = 0;				//���ٶȸ���
float xdata Last_gy = 0;				//���ٶȸ���

float xdata Ax = 0, Ay = 0, Az = 0;		//����ң������������ĽǶ�
int xdata gx = 0, gy = 0;				//����ң������������Ľ��ٶ�
//long xdata g_x_aver = 0;				//û�õ���
//long xdata g_y_aver = 0;				//û�õ���
//long xdata g_z_aver = 0;				//û�õ���
int delta_rc_x = 0;						//�ݲ�ȷ����������
int delta_rc_y = 0;						//�ݲ�ȷ����������
int delta_rc_z = 0;						//�ݲ�ȷ����������
long idata PID_P;						//�ݲ�ȷ����������
long idata PID_I;						//�ݲ�ȷ����������
long idata PID_D;						//�ݲ�ȷ����������

