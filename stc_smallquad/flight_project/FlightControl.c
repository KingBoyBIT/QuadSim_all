#include "FlightControl.h"
#include "globeVar.h"
#include "alldef.h"
//#include <STC15W4K60S4.h>	//STC15W4K48S4 ר��ͷ�ļ�
//#include <intrins.h>		//STC���������������

void PIDcontrolX(void)
{
	//************** MPU6050 X��ָ�� **************************
	delta_rc_x = ((float)Roll - 128) * 2; //�õ� ������ݱ���
	AngleErr_X = -AngleXest - delta_rc_x + inputAngle_x * 5; //
															 //	Ax =-AngleX+a_x*5;
	if (d_throttle > 20)
	{
		integAngleErr_X += AngleErr_X;   //�⻷����(����С��ĳ��ֵʱ������)
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

	PID_P = (long)AngleErr_X * CTL_PARA_PID_ANGLEY_P;
	PID_I = ((long)integAngleErr_X * CTL_PARA_PID_ANGLEX_I) >> 15;
	PID_D = ((Omega_gy + Last_Omega_gy) / 2) * CTL_PARA_PID_ANGLEX_D;
	PID_Output = (PID_P + PID_I + PID_D + 5) / 10;  //�⻷PID

	LastAngleErr_X = AngleErr_X;
	OmegaErr_X = PID_Output - Omega_gy;      //�⻷ -   ������Y��

	if (d_throttle > 20)
	{
		integOmegaErr_X += OmegaErr_X;    //�ڻ�����(����С��ĳ��ֵʱ������)
	}
	else
	{
		integOmegaErr_X = 0; //����С�ڶ�ֵʱ�������ֵ
	}

	if (integOmegaErr_X > INTEG_ANGLE_ERR_MAX)
	{
		integOmegaErr_X = INTEG_ANGLE_ERR_MAX;
	}
	else if (integOmegaErr_X < -INTEG_ANGLE_ERR_MAX)
	{
		integOmegaErr_X = -INTEG_ANGLE_ERR_MAX;   //�����޷�
	}

	PID_P = ((long)OmegaErr_X * CTL_PARA_PID_OMEGA_X_P) >> 15;
	PID_I = ((long)integOmegaErr_X * CTL_PARA_PID_OMEGA_X_I) >> 15;
	PID_D = ((long)OmegaErr_X - LastOmegaErr_X) * CTL_PARA_PID_OMEGA_X_D;
	PID_Output = PID_P + PID_I + PID_D;   //�ڻ�PID

	LastOmegaErr_X = OmegaErr_X;

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
}

void PIDcontrolY(void)
{
	//**************MPU6050 Y��ָ��**************************************************
	delta_rc_y = ((float)Pitch - 128) * 2; //�õ� �������ݱ���
	AngleErr_Y = -AngleY - delta_rc_y - a_y * 5;
	//	Ay  =-AngleY-a_y*5;
	if (d_throttle > 20)
	{
		integAngleErr_Y += AngleErr_Y;               //�⻷����(����С��ĳ��ֵʱ������)
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

	PID_P = (long)AngleErr_Y * CTL_PARA_PID_ANGLEY_P;
	PID_I = ((long)integAngleErr_Y * CTL_PARA_PID_ANGLEY_I) >> 15;
	PID_D = ((Omega_gx + Last_Angle_gx) / 2) * CTL_PARA_PID_ANGLEY_D;
	PID_Output = (PID_P + PID_I + PID_D + 5) / 10; //�⻷PID����+5��Ϊ����������?

	LastAngleErr_Y = AngleErr_Y;
	OmegaErr_Y = PID_Output - Omega_gx;

	if (d_throttle > 20)
	{
		integOmegaErr_Y += OmegaErr_Y; //�ڻ�����(����С��ĳ��ֵʱ������)
	}
	else
	{
		integOmegaErr_Y = 0;                          //����С�ڶ�ֵʱ�������ֵ
	}

	if (integOmegaErr_Y > INTEG_ANGLE_ERR_MAX)
	{
		integOmegaErr_Y = INTEG_ANGLE_ERR_MAX;
	}
	else if (integOmegaErr_Y < -INTEG_ANGLE_ERR_MAX)
	{
		integOmegaErr_Y = -INTEG_ANGLE_ERR_MAX;   //�����޷�
	}

	PID_P = ((long)OmegaErr_Y * CTL_PARA_PID_OMEGA_Y_P) >> 15;
	PID_I = ((long)integOmegaErr_Y * CTL_PARA_PID_OMEGA_Y_I) >> 15;
	PID_D = ((long)OmegaErr_Y - LastOmegaErr_Y) * CTL_PARA_PID_OMEGA_Y_D;
	PID_Output = PID_P + PID_I + PID_D;

	LastOmegaErr_Y = OmegaErr_Y;

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
}

void PIDcontrolZ(void)
{
	//************** MPU6050 Z��ָ�� *****************************
	delta_rc_z = -Omega_gz + ((float)Yaw - 128) * 65 + a_z * 20; //�õ� �������ݱ��� ������
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
	PID_D = ((long)delta_rc_z - LastAngle_Z) * CTL_PARA_PID_ANGLEZ_D;
	PID_Output = (PID_P + PID_I + PID_D) >> 6;

	LastAngle_Z = delta_rc_z;
	speed0 = speed0 + PID_Output;
	speed1 = speed1 - PID_Output;
	speed3 = speed3 - PID_Output;
	speed2 = speed2 + PID_Output;
}

void DangerMotionProtect(void)
{
	//****��б�Ƕȼ��޿���***********************************************************************
	//���޽Ƕ�ֵ   30��
	if ((AngleXest + 900) > 1200)    //�ɿ�������б
	{
		LedR = 0;
		d_throttle = 0;
	}
	else if ((AngleXest + 900) < 500)    //�ɿ�������б
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
}
void PWMoutput(void)
{
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
}
void LostControlProtect(void)
{
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
}
