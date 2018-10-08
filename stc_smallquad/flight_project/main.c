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

//==================================================//
//  LED�� ���Ŷ���
//==================================================//
sbit LedR = P0 ^ 1; //LED ��ɫ R,����ʽ���͵�ƽ��
sbit LedG = P0 ^ 2; //LED ��ɫ G,����ʽ���͵�ƽ��
sbit LedB = P0 ^ 3; //LED ��ɫ B,����ʽ���͵�ƽ��

sbit KARX = P3 ^ 0; //����ӿ� RXD
sbit KATX = P3 ^ 1; //����ӿ� TXD

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


	Read_MPU6050(tp); //ֱ�Ӷ�ȡMPU6050�����Ǻͼ��ٶȵ����ݰ�

	Angle_ax = KalmanFilter_ax(((int *)&tp)[0], KALMAN_Q, KALMAN_R);  //��ͨ�˲������ĵ�����
	Angle_ay = KalmanFilter_ay(((int *)&tp)[1], KALMAN_Q, KALMAN_R);
	Angle_az = KalmanFilter_az(((int *)&tp)[2], KALMAN_Q, KALMAN_R);

	Angle_gx = ((float)(((int *)&tp)[4])) / 65.5;   //�����Ǵ���	�����λ�� +-��
	Angle_gy = ((float)(((int *)&tp)[5])) / 65.5;   //���������� +-500��/S, 1��/�� ��Ӧ���� 65.536
	Angle_gz = KalmanFilter_gyroz(((int *)&tp)[6], Q15(0.2), Q15(0.8));
	IMU_gz = Angle_gz / 65.5;
	Last_Angle_gx = Angle_gx;       //������һ�ν��ٶ�����
	Last_Angle_gy = Angle_gy;

	//*********************************** ��Ԫ������ ***********************************
	IMUupdate(Angle_gx * 0.0174533f,
			  Angle_gy * 0.0174533f,
			  IMU_gz * 0.0174533f,
			  Angle_ax,
			  Angle_ay,
			  Angle_az); //��̬���㣬����0.1��
						 //���͵�ң����
						 //	TxBuf[0]=(AngleX+900)/0xff; // ��ֵ�� 48~1752 = 0-360��
						 //	TxBuf[1]=(AngleX+900)%0xff;	// ��ֵ�� 48~1752 = 0-360��
						 //	TxBuf[2]=(AngleY+900)/0xff;	// ��ֵ�� 48~1752 = 0-360��
						 //	TxBuf[3]=(AngleY+900)%0xff;	// ��ֵ�� 48~1752 = 0-360��
						 //****�ɿ�ʧ���ж� �Զ������㷨*********************
						 //����ң���������Ĳ��ϸ������� �ж�����ͨѶ�Ƿ�����
	if (LostCom == ShiLian)   //���SSLL������û�и��¼�ʧ��
	{
		if (++ShiLianCount >= 20)
		{
			ShiLianCount = 19;      //״̬��ʶ
			Yaw = 128;  //�������
			Roll = 128;  //�������
			Pitch = 128;  //��������
			if (d_throttle > 20)
			{
				d_throttle--; //������ԭֵ�𽥼�С
			}
		}
	}
	else
	{
		ShiLianCount = 0;
		if (throttle > 1001)
		{
			throttle = 1000; //������0-1000���ֵ
							 //�����Ż��㷨 ��������ҡ�˵Ŀ��Ʒ��ȴ�60%���ӵ�90%�����㷨������
		}
		else
		{
			if (throttle > 50)             //ҡ������50ִ��
			{
				d_throttle = (throttle + 300) / 1.3; //ҡ�������㷨
			}
			else
			{
				d_throttle = throttle;          //ҡ�˵���ֱ�Ӹ�ֵ
			}
		}
	}
	ShiLian = LostCom; //ʧ����������
	d_throttle = throttle;
	//****��б�Ƕȼ��޿���***********************************************************************
	//���޽Ƕ�ֵ   30��
	if ((AngleX + 900) > 1200)    //�ɿ�������б
	{
		LedR = 0;
		d_throttle = 0;
	}
	else if ((AngleX + 900) < 500)    //�ɿ�������б
	{
		LedR = 0;
		d_throttle = 0;
	}
	else if ((AngleY + 900) > 1200)    //�ɿ���ǰ��б
	{
		LedR = 0;
		d_throttle = 0;
	}
	else if ((AngleY + 900) < 500)    //�ɿ������б
	{
		LedR = 0;
		d_throttle = 0;
	}
	else
	{
		LedR = 1;  //��ɫ
	}

	//****�����Ƿ��п����㷨************************************
	//************** MPU6050 X��ָ�� **************************
	delta_rc_x = ((float)Roll - 128) * 2; //�õ� ������ݱ���
	Ax = -AngleX - delta_rc_x + a_x * 5; //
										 //	Ax =-AngleX+a_x*5;
	if (d_throttle > 20)
	{
		integAngleErr_X += Ax;   //�⻷����(����С��ĳ��ֵʱ������)
	}
	else
	{
		integAngleErr_X = 0;     //����С�ڶ�ֵʱ�������ֵ
	}

	if (integAngleErr_X > INTEG_ANGLE_ERR_MAX)
	{
		integAngleErr_X = INTEG_ANGLE_ERR_MAX;  //�����޷�
	}
	else if (integAngleErr_X < -INTEG_ANGLE_ERR_MAX)
	{
		integAngleErr_X = -INTEG_ANGLE_ERR_MAX;  //�����޷�
	}

	PID_P = (long)Ax * CTL_PARA_PID_ANGLEY_P;
	PID_I = ((long)integAngleErr_X * CTL_PARA_PID_ANGLEX_I) >> 15;
	PID_D = ((Angle_gy + Last_Angle_gy) / 2) * CTL_PARA_PID_ANGLEX_D;
	PID_Output = (PID_P + PID_I + PID_D + 5) / 10;  //�⻷PID

	Last_Ax = Ax;
	gx = PID_Output - Angle_gy;      //�⻷ -   ������Y��

	if (d_throttle > 20)
	{
		ErrORX_In += gx;    //�ڻ�����(����С��ĳ��ֵʱ������)
	}
	else
	{
		ErrORX_In = 0; //����С�ڶ�ֵʱ�������ֵ
	}

	if (ErrORX_In > INTEG_ANGLE_ERR_MAX)
	{
		ErrORX_In = INTEG_ANGLE_ERR_MAX;
	}
	else if (ErrORX_In < -INTEG_ANGLE_ERR_MAX)
	{
		ErrORX_In = -INTEG_ANGLE_ERR_MAX;   //�����޷�
	}

	PID_P = ((long)gx * CTL_PARA_PID_OMEGA_X_P) >> 15;
	PID_I = ((long)ErrORX_In * CTL_PARA_PID_OMEGA_X_I) >> 15;
	PID_D = ((long)gx - Last_gx) * CTL_PARA_PID_OMEGA_X_D;
	PID_Output = PID_P + PID_I + PID_D;   //�ڻ�PID

	Last_gx = gx;

	if (PID_Output > 1000)
	{
		PID_Output = 1000;  //������޷�
	}
	if (PID_Output < -1000)
	{
		PID_Output = -1000;
	}
	speed0 = 0 + PID_Output;
	speed1 = 0 - PID_Output;
	speed3 = 0 + PID_Output;
	speed2 = 0 - PID_Output;
	//**************MPU6050 Y��ָ��**************************************************
	delta_rc_y = ((float)Pitch - 128) * 2; //�õ� �������ݱ���
	Ay = -AngleY - delta_rc_y - a_y * 5;
	//	Ay  =-AngleY-a_y*5;
	if (d_throttle > 20)
	{
		integAngleErr_Y += Ay;               //�⻷����(����С��ĳ��ֵʱ������)
	}
	else
	{
		integAngleErr_Y = 0;                 //����С�ڶ�ֵʱ�������ֵ
	}

	if (integAngleErr_Y > INTEG_ANGLE_ERR_MAX)
	{
		integAngleErr_Y = INTEG_ANGLE_ERR_MAX;
	}
	else if (integAngleErr_Y < -INTEG_ANGLE_ERR_MAX)
	{
		integAngleErr_Y = -INTEG_ANGLE_ERR_MAX;  //�����޷�
	}

	PID_P = (long)Ay * CTL_PARA_PID_ANGLEY_P;
	PID_I = ((long)integAngleErr_Y * CTL_PARA_PID_ANGLEY_I) >> 15;
	PID_D = ((Angle_gx + Last_Angle_gx) / 2) * CTL_PARA_PID_ANGLEY_D;
	PID_Output = (PID_P + PID_I + PID_D + 5) / 10; //�⻷PID����+5��Ϊ����������?

	Last_Ay = Ay;
	gy = PID_Output - Angle_gx;

	if (d_throttle > 20)
	{
		ErrORY_In += gy; //�ڻ�����(����С��ĳ��ֵʱ������)
	}
	else
	{
		ErrORY_In = 0;                          //����С�ڶ�ֵʱ�������ֵ
	}

	if (ErrORY_In > INTEG_ANGLE_ERR_MAX)
	{
		ErrORY_In = INTEG_ANGLE_ERR_MAX;
	}
	else if (ErrORY_In < -INTEG_ANGLE_ERR_MAX)
	{
		ErrORY_In = -INTEG_ANGLE_ERR_MAX;   //�����޷�
	}

	PID_P = ((long)gy * CTL_PARA_PID_OMEGA_Y_P) >> 15;
	PID_I = ((long)ErrORY_In * CTL_PARA_PID_OMEGA_Y_I) >> 15;
	PID_D = ((long)gy - Last_gy) * CTL_PARA_PID_OMEGA_Y_D;
	PID_Output = PID_P + PID_I + PID_D;

	Last_gy = gy;

	if (PID_Output > 1000)
	{
		PID_Output = 1000;  //������޷�
	}
	if (PID_Output < -1000)
	{
		PID_Output = -1000;
	}
	speed0 = speed0 + PID_Output;
	speed1 = speed1 + PID_Output; //���ص��ٶȲ���
	speed3 = speed3 - PID_Output;
	speed2 = speed2 - PID_Output;

	//************** MPU6050 Z��ָ�� *****************************
	delta_rc_z = -Angle_gz + ((float)Yaw - 128) * 65 + a_z * 20; //�õ� �������ݱ��� ������
	if (d_throttle > 20)
	{
		integAngleErr_Z += delta_rc_z;
	}
	else
	{
		integAngleErr_Z = 0;
	}
	if (integAngleErr_Z > 50000)
	{
		integAngleErr_Z = 50000;
	}
	else if (integAngleErr_Z < -50000)
	{
		integAngleErr_Z = -50000;    //�����޷�
	}
	PID_P = ((long)delta_rc_z) * CTL_PARA_PID_ANGLEZ_P;
	PID_I = ((long)integAngleErr_Z * CTL_PARA_PID_ANGLEZ_I) >> 15;
	PID_D = ((long)delta_rc_z - AngleZ_late) * CTL_PARA_PID_ANGLEZ_D;
	PID_Output = (PID_P + PID_I + PID_D) >> 6;

	AngleZ_late = delta_rc_z;
	speed0 = speed0 + PID_Output;
	speed1 = speed1 - PID_Output;
	speed3 = speed3 - PID_Output;
	speed2 = speed2 + PID_Output;

	//**************���ٶȲ���������PWMģ��*************************************************
	//�ٶȲ������ƣ���ֹ����PWM������Χ0-1000��X����Ч��
	PWM0 = (d_throttle + speed0);
	if (PWM0 > 1000)
	{
		PWM0 = 1000;
	}
	else if (PWM0 < 0)
	{
		PWM0 = 0;
	}
	PWM1 = (d_throttle + speed1);
	if (PWM1 > 1000)
	{
		PWM1 = 1000;
	}
	else if (PWM1 < 0)
	{
		PWM1 = 0;
	}
	PWM2 = (d_throttle + speed2);
	if (PWM2 > 1000)
	{
		PWM2 = 1000;
	}
	else if (PWM2 < 0)
	{
		PWM2 = 0;
	}
	PWM3 = (d_throttle + speed3);
	if (PWM3 > 1000)
	{
		PWM3 = 1000;
	}
	else if (PWM3 < 0)
	{
		PWM3 = 0;
	}

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

