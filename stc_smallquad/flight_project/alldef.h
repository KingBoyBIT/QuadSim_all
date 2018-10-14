#ifndef _ALLDEF_H_
#define _ALLDEF_H_

#include <STC15W4K60S4.h>	//STC15W4K48S4 ר��ͷ�ļ�
#include <intrins.h>		//STC���������������

typedef unsigned char U8,BYTE,UINT8,UCHAR,uchar;
typedef unsigned int uint;
typedef unsigned short u16;

#define BAUD 115200 //���ڲ�����
#define FOSC 24000000L //ϵͳƵ��


//==================================================//
//  LED�� ���Ŷ���
//==================================================//
sbit LedR = P0 ^ 1; //LED ��ɫ R,����ʽ���͵�ƽ��
sbit LedG = P0 ^ 2; //LED ��ɫ G,����ʽ���͵�ƽ��
sbit LedB = P0 ^ 3; //LED ��ɫ B,����ʽ���͵�ƽ��

sbit KARX = P3 ^ 0; //����ӿ� RXD
sbit KATX = P3 ^ 1; //����ӿ� TXD



/*����ֵС��1��С��תΪ����*/
#define Q15(X) ((X < 0.0) ? (int)(32768*(X) - 0.5) : (int)(32767*(X) + 0.5))
#define RC_KALMAN_Q	Q15(0.20)
#define RC_KALMAN_R	Q15(0.80)
//==================================================//
//   PID �ֶ�΢������ֵ
//   �޸�PID���²��� ���԰ɷɿص��ĸ���ƽ��
//   ������Է����Ųο�ѧϰ����
//==================================================//
#define	CTL_PARA_PID_ANGLEX_P	12.0f	    	//�⻷P
#define	CTL_PARA_PID_ANGLEX_I	Q15(0.12)   	//�⻷I
#define	CTL_PARA_PID_ANGLEX_D	5.2f	    	//�⻷D

#define	CTL_PARA_PID_OMEGA_X_P	Q15(0.31) 		//�ڻ�P
#define	CTL_PARA_PID_OMEGA_X_I	Q15(0.012)		//�ڻ�I
#define	CTL_PARA_PID_OMEGA_X_D	4.3f			//�ڻ�D


#define	CTL_PARA_PID_ANGLEY_P	12.0f	    	//�⻷P
#define	CTL_PARA_PID_ANGLEY_I	Q15(0.12)   	//�⻷I
#define	CTL_PARA_PID_ANGLEY_D	5.2f	    	//�⻷D

#define	CTL_PARA_PID_OMEGA_Y_P	Q15(0.31) 		//�ڻ�P
#define	CTL_PARA_PID_OMEGA_Y_I	Q15(0.012)		//�ڻ�I
#define	CTL_PARA_PID_OMEGA_Y_D	4.3f			//�ڻ�D

#define	CTL_PARA_PID_ANGLEZ_P	5.0f			//Z�� P
#define CTL_PARA_PID_ANGLEZ_I	Q15(0.24)		//Z�� I
#define	CTL_PARA_PID_ANGLEZ_D	4.0f			//Z�� D

#define	INTEG_ANGLE_ERR_MAX		800				//�����޷�

#define	MATH_PI_F				3.14159265f
#define MATH_180_PI_10			572.957795f		//10*180/pi
#define	NAV_PARA_EST_KP			2.0f			//AHRS����
#define	NAV_PARA_EST_KI			0.001f			//AHRS����
#define NAV_PARA_EST_KD			0.001f			//AHRS����
#define	NAV_PARA_EST_HALF_T		0.005f			//�˲�����ʱ������
/*ң��������*/
#define RC_UNLOCK				5
#define RC_LOCK					1

/*MPU6050*/
#define	SMPLRT_DIV				0x19			//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG					0x1A			//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG				0x1B			//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG			0x1C			//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define	ACCEL_XOUT_H			0x3B
#define	ACCEL_XOUT_L			0x3C
#define	ACCEL_YOUT_H			0x3D
#define	ACCEL_YOUT_L			0x3E
#define	ACCEL_ZOUT_H			0x3F
#define	ACCEL_ZOUT_L			0x40
#define	TEMP_OUT_H				0x41
#define	TEMP_OUT_L				0x42
#define	GYRO_XOUT_H				0x43
#define	GYRO_XOUT_L				0x44
#define	GYRO_YOUT_H				0x45
#define	GYRO_YOUT_L				0x46
#define	GYRO_ZOUT_H				0x47
#define	GYRO_ZOUT_L				0x48
#define	PWR_MGMT_1				0x6B			//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I				0x75			//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define	SLAVE_ADDR				0xD0			//IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ
#define IICSPEED        		0x24

#endif