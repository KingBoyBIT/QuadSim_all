#include "FlightControl.h"
#include "globeVar.h"
#include "alldef.h"
//#include <STC15W4K60S4.h>	//STC15W4K48S4 专用头文件
//#include <intrins.h>		//STC特殊命令符号声明

void PIDcontrolX(void)
{
	//************** MPU6050 X轴指向 **************************
	delta_rc[0] = ((float)rc_Roll - 128) * 2; //得到 横滚数据变量
	AngleErr[0] = -AngleXest - delta_rc[0] + rcAngle_X_offset * 5; //
															 //	Ax =-AngleX+a_x*5;
	if (Ctl_throttle > 20)
	{
		integAngleErr_X += AngleErr[0];   //外环积分(油门小于某个值时不积分)
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

	PID_P = (long)AngleErr[0] * CTL_PARA_PID_ANGLEY_P;
	PID_I = ((long)integAngleErr_X * CTL_PARA_PID_ANGLEX_I) >> 15;
	PID_D = ((Omega_gy + Last_Omega_gy) / 2) * CTL_PARA_PID_ANGLEX_D;
	PID_Output = (PID_P + PID_I + PID_D + 5) / 10;  //外环PID

	LastAngleErr_X = AngleErr[0];
	OmegaErr_X = PID_Output - Omega_gy;      //外环 -   陀螺仪Y轴

	if (Ctl_throttle > 20)
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
	delta_rc[1] = ((float)rc_Pitch - 128) * 2; //得到 俯仰数据变量
	AngleErr[1] = -AngleYest - delta_rc[1] - rcAngle_Y_offset * 5;
	//	Ay  =-AngleY-a_y*5;
	if (Ctl_throttle > 20)
	{
		integAngleErr_Y += AngleErr[1];               //外环积分(油门小于某个值时不积分)
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

	PID_P = (long)AngleErr[1] * CTL_PARA_PID_ANGLEY_P;
	PID_I = ((long)integAngleErr_Y * CTL_PARA_PID_ANGLEY_I) >> 15;
	PID_D = ((Omega_gx + Last_Angle_gx) / 2) * CTL_PARA_PID_ANGLEY_D;
	PID_Output = (PID_P + PID_I + PID_D + 5) / 10; //外环PID，“+5”为了四舍五入?

	LastAngleErr_Y = AngleErr[1];
	OmegaErr_Y = PID_Output - Omega_gx;

	if (Ctl_throttle > 20)
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
	delta_rc[2] = -Omega_gz + ((float)rc_Yaw - 128) * 65 + rcAngle_Z_offset * 20; //得到 航向数据变量 操作量
	if (Ctl_throttle > 20)
	{
		integAngleErr_Z += delta_rc[2];
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
	PID_P = ((long)delta_rc[2]) * CTL_PARA_PID_ANGLEZ_P;
	PID_I = ((long)integAngleErr_Z * CTL_PARA_PID_ANGLEZ_I) >> 15;
	PID_D = ((long)delta_rc[2] - LastAngle_Z) * CTL_PARA_PID_ANGLEZ_D;
	PID_Output = (PID_P + PID_I + PID_D) >> 6;

	LastAngle_Z = delta_rc[2];
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
		Ctl_throttle = 0;
	}
	else if ((AngleXest + 900) < 500)    //飞控向左倾斜
	{
		LedR = 0;
		Ctl_throttle = 0;
	}
	else if ((AngleYest + 900) > 1200)    //飞控向前倾斜
	{
		LedR = 0;
		Ctl_throttle = 0;
	}
	else if ((AngleYest + 900) < 500)    //飞控向后倾斜
	{
		LedR = 0;
		Ctl_throttle = 0;
	}
	else
	{
		LedR = 1;  //红色
	}
}
void PWMoutput(void)
{
	PWM0 = (Ctl_throttle + speed0);
	if (PWM0 > 1000)
	{
		PWM0 = 1000;
	}
	else if (PWM0 < 0)
	{
		PWM0 = 0;
	}
	PWM1 = (Ctl_throttle + speed1);
	if (PWM1 > 1000)
	{
		PWM1 = 1000;
	}
	else if (PWM1 < 0)
	{
		PWM1 = 0;
	}
	PWM2 = (Ctl_throttle + speed2);
	if (PWM2 > 1000)
	{
		PWM2 = 1000;
	}
	else if (PWM2 < 0)
	{
		PWM2 = 0;
	}
	PWM3 = (Ctl_throttle + speed3);
	if (PWM3 > 1000)
	{
		PWM3 = 1000;
	}
	else if (PWM3 < 0)
	{
		PWM3 = 0;
	}
}
/**
 * 失联控制保护
 *
 * @author KingBoy (2018/10/14)
 *
 * @param void
 */
void LostControlProtect(void)
{
	if (RCnum == RCnum_pre)   //如果指令流水号没有更新即失联
	{
		//超过20条指令没有响应，降低油门降落
		if (++LostComCount >= 20)
		{
			LostComCount = 19;//失联计数
			rc_Yaw = 128;  //航向变量
			rc_Roll = 128;  //横滚变量
			rc_Pitch = 128;  //俯仰变量
			if (Ctl_throttle > 20)
			{
				Ctl_throttle--; //油门在原值逐渐减小
			}
		}
	}
	else
	{
		LostComCount = 0;//失联计数清空
		if (rc_throttle >= 1001)
		{
			rc_throttle = 1000; //油门量0-1000最大值
		}
		else
		{
			//油门优化算法 【将油门摇杆的控制幅度从60%增加到90%控制算法】如下
			if (rc_throttle > 50)             //摇杆量上50执行
			{
				Ctl_throttle = (rc_throttle + 300) / 1.3; //摇杆增幅算法
			}
			else
			{
				Ctl_throttle = rc_throttle;          //摇杆低于直接赋值
			}
		}
	}
	RCnum_pre = RCnum; //指令流水号更新
	Ctl_throttle = rc_throttle;
}
