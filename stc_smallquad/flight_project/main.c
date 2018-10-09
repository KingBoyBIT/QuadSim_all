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



/**
 * ��ʱ��0 ��ʼ������
 * 12����@24.000MHz
 *
 * @author KingBoy (2018/5/19)
 *
 * @param void
 */
void Timer0Init(void);
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

	Timer0Init(); //��ʼ����ʱ��
	Delay(100);   //��ʱһ�� 1S
	/*Ĭ��ֵ��ʼ��*/
	throttle = 0;   //��ʼ�����ű���
	Yaw = 128;      //��ʼ���������
	Roll = 128;     //��ʼ���������
	Pitch = 128;    //��ʼ����������
	LedB = 0;       //�����̵�
	a_x = 0;        //����ֶ�ֵ
	a_y = 0;        //�����ֶ�ֵ
	a_z = 0;        //�����ֶ�ֵ

	//Flight();//�����2��������˵ �ɿغ����ж��� ���������������Ҫ���᡿
	EA = 1;  //�����ж�
	while (1)
	{
#if 0
		//Ҫ�ŵ���ʱ���ж�������
		RX_model();             //����ģʽ
		js_shuju(RxBuf, 15);     //��ȡ���ݰ�
#endif
		/*����ָ�������ȷ*/
		if (MAC_calc(RxBuf, 10, RxBuf[10]) == 0)
		{
			LostCom = RxBuf[0];                     //���� ʧ������
			LockState = RxBuf[1];                   //���� ����ֵ 1=����  5=����
			throttle = RxBuf[2] * 0xff + RxBuf[3];  //���� ���ű���
			Yaw = RxBuf[4];                         //���� �������
			Roll = RxBuf[5];                        //���� �������
			Pitch = RxBuf[6];                       //���� ��������
			a_x = RxBuf[7] - 128;                   //���� X�ᱣ��ֵ
			a_y = RxBuf[8] - 128;                   //���� Y�ᱣ��ֵ
			a_z = RxBuf[9] - 128;                   //���� Z�ᱣ��ֵ
			LedG = 1;                               //LED �̵���
		}
		else
		{
			LedG = 0;                               //LED �̵���
		}

		if (LockState == 1)                         //ң������ ����
		{
			LedB = 1;                               //����� ��ɫ��
		}
		if (LockState == 5)                         //ң������ ����
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
void Timer0Init(void)
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
	ReceiveData(RxBuf, 15);


	Read_MPU6050(IMUdata); //ֱ�Ӷ�ȡMPU6050�����Ǻͼ��ٶȵ����ݰ�

	Angle_ax = RCLowPassFilter_ax(((int *)&IMUdata)[0], RC_KALMAN_Q, RC_KALMAN_R);  //��ͨ�˲������ĵ�����
	Angle_ay = RCLowPassFilter_ay(((int *)&IMUdata)[1], RC_KALMAN_Q, RC_KALMAN_R);
	Angle_az = RCLowPassFilter_az(((int *)&IMUdata)[2], RC_KALMAN_Q, RC_KALMAN_R);

	Angle_gx = ((float)(((int *)&IMUdata)[4])) / 65.5;   //�����Ǵ���	�����λ�� +-��
	Angle_gy = ((float)(((int *)&IMUdata)[5])) / 65.5;   //���������� +-500��/S, 1��/�� ��Ӧ���� 65.536
	Angle_gz = RCLowPassFilter_gyroz(((int *)&IMUdata)[6], Q15(0.2), Q15(0.8));
	IMU_gz = Angle_gz / 65.5;
	Last_Angle_gx = Angle_gx;       //������һ�ν��ٶ�����
	Last_Angle_gy = Angle_gy;

	//*********************************** ��Ԫ������ ***********************************
	IMUupdate(Angle_gx * 0.0174533f,Angle_gy * 0.0174533f,IMU_gz * 0.0174533f,Angle_ax,Angle_ay,Angle_az);
	//��̬���㣬����0.1��
	//���͵�ң����
	//	TxBuf[0]=(AngleX+900)/0xff; // ��ֵ�� 48~1752 = 0-360��
	//	TxBuf[1]=(AngleX+900)%0xff;	// ��ֵ�� 48~1752 = 0-360��
	//	TxBuf[2]=(AngleY+900)/0xff;	// ��ֵ�� 48~1752 = 0-360��
	//	TxBuf[3]=(AngleY+900)%0xff;	// ��ֵ�� 48~1752 = 0-360��
	//****�ɿ�ʧ���ж� �Զ������㷨*********************
	//����ң���������Ĳ��ϸ������� �ж�����ͨѶ�Ƿ�����
	LostControlProtect();
	/*��̬��ʧ�ر���*/
	DangerMotionProtect();

	//****�����Ƿ��п����㷨************************************
	PIDcontrolX();
	PIDcontrolY();
	PIDcontrolZ();
	//**************���ٶȲ���������PWMģ��*************************************************
	//�ٶȲ������ƣ���ֹ����PWM������Χ0-1000��X����Ч��
	PWMoutput();

	//������������������2.4G=5�����Ŵ���30�����ܿ��Ƶ��
	if (LockState == 5 && d_throttle >= 50)
	{
		Set_PWM(1000 - PWM1, 1000 - PWM2, 1000 - PWM3, 1000 - PWM0);
	} //����PWM
	else
	{
		Set_PWM(1000, 1000, 1000, 1000);
	} //�ر�PWM
}

