#include "FlightControl.h"
#include "globeVar.h"
#include "alldef.h"
//#include <STC15W4K60S4.h>	//STC15W4K48S4 专用头文件
//#include <intrins.h>		//STC特殊命令符号声明

void PIDcontrolX(void)
{
	//************** MPU6050 X轴指向 **************************
	delta_rc_x = ((float)Roll - 128) * 2; //得到 横滚数据变量
	AngleErr_X = -AngleXest - delta_rc_x + inputAngle_x * 5; //
															 //	Ax =-AngleX+a_x*5;
	if (d_throttle > 20)
	{
		integAngleErr_X += AngleErr_X;   //外环积分(油门小于某个值时不积分)
	}
	else
	{
		integAngleErr_X = 0;     //油门小于定值时清除积分值
	}

	if (integAngleErr_X > INTEG_ANGLE_ERR_MAX)
	{
		integAngleErr_X = INTEG_ANGLE_ERR_MAX;  //积分限幅
	}
	else if (integAngleErr_X < -INTEG_ANGLE_ERR_MAX)
	{
		integAngleErr_X = -INTEG_ANGLE_ERR_MAX;  //积分限幅
	}

	PID_P = (long)AngleErr_X * CTL_PARA_PID_ANGLEY_P;
	PID_I = ((long)integAngleErr_X * CTL_PARA_PID_ANGLEX_I) >> 15;
	PID_D = ((Omega_gy + Last_Omega_gy) / 2) * CTL_PARA_PID_ANGLEX_D;
	PID_Output = (PID_P + PID_I + PID_D + 5) / 10;  //外环PID

	LastAngleErr_X = AngleErr_X;
	OmegaErr_X = PID_Output - Omega_gy;      //外环 -   陀螺仪Y轴

	if (d_throttle > 20)
	{
		integOmegaErr_X += OmegaErr_X;    //内环积分(油门小于某个值时不积分)
	}
	else
	{
		integOmegaErr_X = 0; //油门小于定值时清除积分值
	}

	if (integOmegaErr_X > INTEG_ANGLE_ERR_MAX)
	{
		integOmegaErr_X = INTEG_ANGLE_ERR_MAX;
	}
	else if (integOmegaErr_X < -INTEG_ANGLE_ERR_MAX)
	{
		integOmegaErr_X = -INTEG_ANGLE_ERR_MAX;   //积分限幅
	}

	PID_P = ((long)OmegaErr_X * CTL_PARA_PID_OMEGA_X_P) >> 15;
	PID_I = ((long)integOmegaErr_X * CTL_PARA_PID_OMEGA_X_I) >> 15;
	PID_D = ((long)OmegaErr_X - LastOmegaErr_X) * CTL_PARA_PID_OMEGA_X_D;
	PID_Output = PID_P + PID_I + PID_D;   //内环PID

	LastOmegaErr_X = OmegaErr_X;

	if (PID_Output > 1000)
	{
		PID_Output = 1000;  //输出量限幅
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
	//**************MPU6050 Y轴指向**************************************************
	delta_rc_y = ((float)Pitch - 128) * 2; //得到 俯仰数据变量
	AngleErr_Y = -AngleY - delta_rc_y - a_y * 5;
	//	Ay  =-AngleY-a_y*5;
	if (d_throttle > 20)
	{
		integAngleErr_Y += AngleErr_Y;               //外环积分(油门小于某个值时不积分)
	}
	else
	{
		integAngleErr_Y = 0;                 //油门小于定值时清除积分值
	}

	if (integAngleErr_Y > INTEG_ANGLE_ERR_MAX)
	{
		integAngleErr_Y = INTEG_ANGLE_ERR_MAX;
	}
	else if (integAngleErr_Y < -INTEG_ANGLE_ERR_MAX)
	{
		integAngleErr_Y = -INTEG_ANGLE_ERR_MAX;  //积分限幅
	}

	PID_P = (long)AngleErr_Y * CTL_PARA_PID_ANGLEY_P;
	PID_I = ((long)integAngleErr_Y * CTL_PARA_PID_ANGLEY_I) >> 15;
	PID_D = ((Omega_gx + Last_Angle_gx) / 2) * CTL_PARA_PID_ANGLEY_D;
	PID_Output = (PID_P + PID_I + PID_D + 5) / 10; //外环PID，“+5”为了四舍五入?

	LastAngleErr_Y = AngleErr_Y;
	OmegaErr_Y = PID_Output - Omega_gx;

	if (d_throttle > 20)
	{
		integOmegaErr_Y += OmegaErr_Y; //内环积分(油门小于某个值时不积分)
	}
	else
	{
		integOmegaErr_Y = 0;                          //油门小于定值时清除积分值
	}

	if (integOmegaErr_Y > INTEG_ANGLE_ERR_MAX)
	{
		integOmegaErr_Y = INTEG_ANGLE_ERR_MAX;
	}
	else if (integOmegaErr_Y < -INTEG_ANGLE_ERR_MAX)
	{
		integOmegaErr_Y = -INTEG_ANGLE_ERR_MAX;   //积分限幅
	}

	PID_P = ((long)OmegaErr_Y * CTL_PARA_PID_OMEGA_Y_P) >> 15;
	PID_I = ((long)integOmegaErr_Y * CTL_PARA_PID_OMEGA_Y_I) >> 15;
	PID_D = ((long)OmegaErr_Y - LastOmegaErr_Y) * CTL_PARA_PID_OMEGA_Y_D;
	PID_Output = PID_P + PID_I + PID_D;

	LastOmegaErr_Y = OmegaErr_Y;

	if (PID_Output > 1000)
	{
		PID_Output = 1000;  //输出量限幅
	}
	if (PID_Output < -1000)
	{
		PID_Output = -1000;
	}
	speed0 = speed0 + PID_Output;
	speed1 = speed1 + PID_Output; //加载到速度参数
	speed3 = speed3 - PID_Output;
	speed2 = speed2 - PID_Output;
}

void PIDcontrolZ(void)
{
	//************** MPU6050 Z轴指向 *****************************
	delta_rc_z = -Omega_gz + ((float)Yaw - 128) * 65 + a_z * 20; //得到 航向数据变量 操作量
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
		integAngleErr_Z = -50000;    //积分限幅
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
	//****倾斜角度极限控制***********************************************************************
	//极限角度值   30度
	if ((AngleXest + 900) > 1200)    //飞控向右倾斜
	{
		LedR = 0;
		d_throttle = 0;
	}
	else if ((AngleXest + 900) < 500)    //飞控向左倾斜
	{
		LedR = 0;
		d_throttle = 0;
	}
	else if ((AngleY + 900) > 1200)    //飞控向前倾斜
	{
		LedR = 0;
		d_throttle = 0;
	}
	else if ((AngleY + 900) < 500)    //飞控向后倾斜
	{
		LedR = 0;
		d_throttle = 0;
	}
	else
	{
		LedR = 1;  //红色
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
	if (LostCom == ShiLian)   //如果SSLL的数据没有更新即失联
	{
		if (++ShiLianCount >= 20)
		{
			ShiLianCount = 19;      //状态标识
			Yaw = 128;  //航向变量
			Roll = 128;  //横滚变量
			Pitch = 128;  //俯仰变量
			if (d_throttle > 20)
			{
				d_throttle--; //油门在原值逐渐减小
			}
		}
	}
	else
	{
		ShiLianCount = 0;
		if (throttle > 1001)
		{
			throttle = 1000; //油门量0-1000最大值
							 //油门优化算法 【将油门摇杆的控制幅度从60%增加到90%控制算法】如下
		}
		else
		{
			if (throttle > 50)             //摇杆量上50执行
			{
				d_throttle = (throttle + 300) / 1.3; //摇杆增幅算法
			}
			else
			{
				d_throttle = throttle;          //摇杆低于直接赋值
			}
		}
	}
	ShiLian = LostCom; //失联变量更新
	d_throttle = throttle;
}
