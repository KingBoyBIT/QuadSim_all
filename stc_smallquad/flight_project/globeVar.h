#ifndef _GLOBE_VAR_H_
#define _GLOBE_VAR_H_

/*ȫ�ֱ�������*/
extern unsigned char data RxBuf[20]; //���ý��ճ��ȣ����Ϊ32�ֽ�

/*�ɿؿ��Ʋ���*/
extern unsigned int d_throttle;//���ű仯�ٶȿ��ƣ����������Ļ����ٱ仯����ʱ�����ʧ�ٷ�ת��׹��
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
extern char a_x;
extern char a_y;
extern char a_z;

extern float IMU_gz;//����δ֪

/*MPU-6050�Ĵ������ݲ���*/
extern double Gyro_y;//Y�������������ݴ�
extern double Gyro_x;//X�������������ݴ�
extern double Gyro_z;//Z�������������ݴ�
extern int xdata Angle_ax;//�ɼ��ٶȼ���ļ��ٶ�(������)
extern int xdata Angle_ay;//�ɼ��ٶȼ���ļ��ٶ�(������)
extern int xdata Angle_az;//�ɼ��ٶȼ���ļ��ٶ�(������)
extern int idata Angle_gy;//�ɽ��ٶȼ���Ľ�����(�Ƕ���)
extern int idata Angle_gx;//�ɽ��ٶȼ���Ľ�����(�Ƕ���)
extern int idata Angle_gz;//�ɽ��ٶȼ���Ľ�����(�Ƕ���)

extern int data AngleX, AngleY;//��Ԫ���������ŷ����  ,AngleZ=0

extern unsigned char data tp[16];//acc16*3+tempreture16+gyro16*3ֱ�Ӷ�ȡMPU6050�����Ǻͼ��ٶȵ����ݰ����õ�14�ֽڣ�
 

/*���п��Ʊ���*/
extern unsigned char LockState;//�Ͽ�/���� ��������
extern unsigned char LostCom;//ͨѶ״̬ ����
extern unsigned char ShiLian;//ʧ������
extern unsigned char ShiLianCount;//ʧ����������
extern unsigned int throttle;//���ű���
extern unsigned int Yaw;//�������
extern unsigned int Roll;//�������
extern unsigned int Pitch;//��������
extern unsigned char PitchRoll;//�����������
extern unsigned char CKV_calc;//CRC ���Ч���
extern unsigned char CKV_match;//CRC ����Ч���

/*PID�㷨����*/
extern float data PID_Output;//PID���������

extern float xdata Last_Angle_gx;//�⻷PI�����  ��һ������������
extern float xdata Last_Angle_gy;//�⻷PI�����  ��һ������������

extern float xdata ErrORX_In;//�ڻ�P �ڻ�I �ڻ�D �ڻ�������
extern float xdata integAngleErr_X;//�⻷P �⻷I       �⻷������

extern float xdata ErrORY_In;//�ڻ�P �ڻ�I �ڻ�D �ڻ�������
extern float xdata integAngleErr_Y;//�⻷P �⻷I       �⻷������

extern float xdata integAngleErr_Z;//�⻷P �⻷I       �⻷������
extern float xdata AngleZ_late;//Z�����??

extern float xdata Last_Ax;//�Ƕȸ���
extern float xdata Last_Ay;//�Ƕȸ���

extern float xdata Last_gx;//���ٶȸ���
extern float xdata Last_gy;//���ٶȸ���

extern float xdata Ax, Ay, Az;//����ң������������ĽǶ�
extern int xdata gx, gy;//����ң������������Ľ��ٶ�
//extern long xdata g_x_aver;//û�õ���
//extern long xdata g_y_aver;//û�õ���
//extern long xdata g_z_aver;//û�õ���
extern int delta_rc_x;//�ݲ�ȷ����������
extern int delta_rc_y;//�ݲ�ȷ����������
extern int delta_rc_z;//�ݲ�ȷ����������
extern long idata PID_P;//�ݲ�ȷ����������
extern long idata PID_I;//�ݲ�ȷ����������
extern long idata PID_D;//�ݲ�ȷ����������


#endif
