#include <STC15W4K60S4.h>	//STC15W4K48S4 ר��ͷ�ļ�
#include <intrins.h>		//STC���������������
#include <MPU-6050.H>		//MPU6050����������
#include <STC15W4K-PWM.H>	//��Ƭ������IO�ڳ�ʼ��-PWM�ڳ�ʼ��
#include <LT8910.h>			//����2.4Gģ��
//#include <STC15W4K-ADC.h>	//STC15W4K-ADC	Ӳ��ADCģ��ת��
#include <IMU.H>			//IMU���к����㷨
#include <KalmanFilter.h>  //�������˲��㷨
#include "globeVar.h"
#include "alldef.h"
#include "FlightControl.h"
#include "Uart.h"



/**
 * ��ʱ��0 ��ʼ������
 * 12����@24.000MHz
 *
 * @author KingBoy (2018/5/19)
 *
 * @param void
 */
void TimerInit(void);
/**
 * ʱ����ʱ ����
 *
 * @author KingBoy (2018/5/20)
 *
 * @param x
 */
void Delay(unsigned int x);

/**
 * ��Ƭ������STC15W4K48S4-��ǿ�͵�Ƭ����IPA���޸�EEPROM���룩
 * ������ѹΪ��5V ����Ƶ��Ϊ��24M ����ͨѶģ��Ϊ��LT8910
 * �������в�����MCU6050����̬���ݲɼ���PID�㷨��Ӳ��4·PWM����MOS���������ı����
 * ע�⣺�˳���ֻ���ݣ����ı�716�����45MMר�����������Լ��ĵ�������޸�PID������
 * �ں��ڵĳ�������У�������Ӳ������ﵽԤ�ڷ��й��ܡ������ע��
 *
 * @author KingBoy (2018/5/19)
 *
 * @param void
 */
void main(void)
{
	unsigned char i = 0;

	PWM_Init(); //��ʼ��PWM
	Set_PWM(1000, 1000, 1000, 1000); //�رյ��
	LedR = 0;
	LedG = 1;
	LedB = 1; //3��״̬��
	Delay(500); //��ʱһ��
	LedR = 1;
	LedG = 0;
	LedB = 1; //3��״̬��
	Delay(500); //��ʱһ��
	LedR = 1;
	LedG = 1;
	LedB = 0; //3��״̬��
	Delay(500); //��ʱһ��
	LedR = 1;
	LedG = 1;
	LedB = 1; //3��״̬��
	Delay(10);    // ��ʱ 100

	//InitADC();/ADCģ��ת�� ��ʼ�������ڿ�����
	Delay(10); //��ʱ 100
	Init_MPU6050(); //��ʼ��MPU-6050
	Delay(10); //��ʱ 100
	LT8910_Init(); //����2.4Gģ���ʼ��
	Delay(100); //��ʱһ�� 1S

	TimerInit(); //��ʼ����ʱ��
	UartInit(BAUD, FOSC); //��ʼ������
	Delay(100);   //��ʱһ�� 1S
	/*Ĭ��ֵ��ʼ��*/
	rc_throttle = 0;   //��ʼ�����ű���
	rc_Yaw = 128;      //��ʼ���������
	rc_Roll = 128;     //��ʼ���������
	rc_Pitch = 128;    //��ʼ����������
	LedB = 0;       //�����̵�
	rcAngle_X_offset = 0;        //����ֶ�ֵ
	rcAngle_Y_offset = 0;        //�����ֶ�ֵ
	rcAngle_Z_offset = 0;        //�����ֶ�ֵ

	//Flight();//�����2��������˵ �ɿغ����ж��� ���������������Ҫ���᡿
	ES = 1; //ʹ�ܴ���1�ж�
	EA = 1;  //�����ж�
	UartSendStr("Uart Test !\r\n");
	while (1)
	{
#if 0
		//Ҫ�ŵ���ʱ���ж�������
		RX_model();             //����ģʽ
		js_shuju(RxBuf, 15);     //��ȡ���ݰ�
#endif

		/*����ָ�������ȷ�򻺴棬������*/
		if (MAC_calc(RxBuf, 10, RxBuf[10]) == 0)
		{
#if 1
			if (RCnum != RxBuf[0])
			{
				for (i = 0; i < 10; i++)
				{
					UartSendByte((RxBuf[i] & 0xf0) >> 4, 1);
					UartSendByte(RxBuf[i] & 0x0f, 1);
				}
				UartSendByte(0x0d, 0);
				UartSendByte(0x0a, 0);
				//UartSendStr("\r\n");
				//Delay(300);     //��ʱһ��
			}
#endif
			RCnum = RxBuf[0];                         //���� ʧ������
			LockState = RxBuf[1];                       //���� ����ֵ 1=����  5=����
			rc_throttle = RxBuf[2] * 0xff + RxBuf[3];   //���� ���ű���
			rc_Yaw = RxBuf[4];                          //���� ����ҡ�˲���
			rc_Roll = RxBuf[5];                         //���� ���ҡ�˲���
			rc_Pitch = RxBuf[6];                        //���� ����ҡ�˲���
			rcAngle_X_offset = RxBuf[7] - 128;          //���� ���΢������
			rcAngle_Y_offset = RxBuf[8] - 128;          //���� ����΢������
			rcAngle_Z_offset = RxBuf[9] - 128;          //���� ����΢������
			LedG = 1;                                   //LED �̵���
		}
		else
		{
			LedG = 0;                               //LED �̵���
		}
		if (LockState == RC_LOCK)                         //ң������ ����
		{
			LedB = 1;                               //����� ��ɫ��
		}
		if (LockState == RC_UNLOCK)                         //ң������ ����
		{
			LedB = 0;                               //����� ��ɫ��
		}
		Delay(3);     //��ʱһ��
#if 0
		//ADC��ѹ��ѹ�����ͣ�������� ���ڿ���
		ADC_CONTR
#endif

	}
}

/**
 * ��ʱ��0 ��ʼ������
 * 12����@24.000MHz
 *
 * @author KingBoy (2018/5/19)
 *
 * @param void
 */
void TimerInit(void)
{
	AUXR &= 0x7F;   //��ʱ��ʱ��12Tģʽ
	TMOD &= 0xF0;   //���ö�ʱ��ģʽ
	TL0 = 0x40;     //���ö�ʱ��ֵ
	TH0 = 0xA2;     //���ö�ʱ��ֵ
	TF0 = 0;        //���TF0��־
	TR0 = 1;        //��ʱ��0��ʼ��ʱ
	ET0 = 1;        //Timer0 Interrupt Enable
}
/**
 * ʱ����ʱ ����
 *
 * @author KingBoy (2018/5/20)
 *
 * @param x
 */
void Delay(unsigned int x)
{
	unsigned int i, j;
	for (i = 0; i < x; i++)
	{
		for (j = 0; j < 250; j++)
		{
			;
		}
	}
}

/**
 * PID�㷨�ɿ���ƽ�⺯������ʱ��0�ж�12����ִ��һ��
 *
 * @author KingBoy (2018/5/20)
 *
 * @param void
 */
void Flight(void) interrupt 1
{
	/*LT8910����Ϊ����ģʽ*/
	RX_model();
	/*����15�ֽ�����*/
	ReceiveRC_Data(RxBuf, 15);

	/*��ȡ�ߵ�����*/
	Read_MPU6050(IMUdata); //ֱ�Ӷ�ȡMPU6050�����Ǻͼ��ٶȵ����ݰ�

	Angle_ax = RCLowPassFilter_ax(((int *)&IMUdata)[0], RC_KALMAN_Q, RC_KALMAN_R);  //��ͨ�˲������ĵ�����
	Angle_ay = RCLowPassFilter_ay(((int *)&IMUdata)[1], RC_KALMAN_Q, RC_KALMAN_R);
	Angle_az = RCLowPassFilter_az(((int *)&IMUdata)[2], RC_KALMAN_Q, RC_KALMAN_R);

	Omega_gx = ((float)(((int *)&IMUdata)[4])) / DEGPSEC;   //�����Ǵ���	�����λ�� +-��
	Omega_gy = ((float)(((int *)&IMUdata)[5])) / DEGPSEC;   //���������� +-500��/S, 1��/�� ��Ӧ���� 65.536
	Omega_gz = RCLowPassFilter_gyroz(((int *)&IMUdata)[6], Q15(0.2), Q15(0.8));
	IMU_gz = Omega_gz / DEGPSEC;
	Last_Angle_gx = Omega_gx;       //������һ�ν��ٶ�����
	Last_Omega_gy = Omega_gy;

	/*��Ԫ������*/
	//��̬���㣬����0.1��
	IMUupdate(Omega_gx * DEG2RAD, Omega_gy * DEG2RAD, IMU_gz * DEG2RAD, Angle_ax, Angle_ay, Angle_az);

	/*�ɿ�ʧ���ж� �Զ������㷨*/
	//����ң���������Ĳ��ϸ������� �ж�����ͨѶ�Ƿ�����
	LostControlProtect();
	/*��̬��ʧ�ر���*/
	DangerMotionProtect();

	/*���п����㷨*/
	PIDcontrolX();
	PIDcontrolY();
	PIDcontrolZ();
	/*���ٶȲ���������PWMģ��*/
	//�ٶȲ������ƣ���ֹ����PWM������Χ0-1000��X����Ч��
	PWMoutput();

	//������������������2.4G=5�����Ŵ���30�����ܿ��Ƶ��
	if (LockState == RC_UNLOCK && Ctl_throttle >= 50)
	{
		Set_PWM(1000 - PWM1, 1000 - PWM2, 1000 - PWM3, 1000 - PWM0);
	} //����PWM
	else
	{
		Set_PWM(1000, 1000, 1000, 1000);
	} //�ر�PWM��1000Ϊ�أ�

#if 0
	//����ǿ�йرյ��
	Set_PWM(1000, 1000, 1000, 1000);
#endif
}

void Uart(void) interrupt 4 using 1
{
	while(RI)
	{
		RI = 0;
		P0 = SBUF;
		P22 = RB8;
	}
	while(TI)
	{
		TI = 0;
		busy = 0;
	}
}
