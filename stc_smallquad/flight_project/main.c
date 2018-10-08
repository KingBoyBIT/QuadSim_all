#include <STC15W4K60S4.h>	//STC15W4K48S4 专用头文件
#include <intrins.h>		//STC特殊命令符号声明
#include <MPU-6050.H>		//MPU6050数字陀螺仪
#include <STC15W4K-PWM.H>	//单片机所有IO口初始化-PWM口初始化
#include <LT8910.h>			//无线2.4G模块
//#include <STC15W4K-ADC.h>	//STC15W4K-ADC	硬件ADC模数转换
#include <IMU.H>			//IMU飞行核心算法
#include <KalmanFilter.h>  //卡尔曼滤波算法
#include "globeVar.h"
#include "alldef.h"

//==================================================//
//  LED灯 引脚定义
//==================================================//
sbit LedR = P0 ^ 1; //LED 红色 R,灌入式，低电平亮
sbit LedG = P0 ^ 2; //LED 绿色 G,灌入式，低电平亮
sbit LedB = P0 ^ 3; //LED 蓝色 B,灌入式，低电平亮

sbit KARX = P3 ^ 0; //排针接口 RXD
sbit KATX = P3 ^ 1; //排针接口 TXD

/**
 * 定时器0 初始化函数
 * 12毫秒@24.000MHz
 *
 * @author KingBoy (2018/5/19)
 *
 * @param void
 */
void Timer0Init(void);
/**
 * 时间延时 函数
 *
 * @author KingBoy (2018/5/20)
 *
 * @param x
 */
void Delay(unsigned int x);

/**
 * 单片机采用STC15W4K48S4-增强型单片机（IPA需修改EEPROM代码）
 * 工作电压为：5V 晶振频率为：24M 无线通讯模块为：LT8910
 * 本程序中采用了MCU6050做姿态数据采集加PID算法给硬件4路PWM控制MOS管驱动空心杯电机
 * 注意：此程序只兼容：空心杯716电机与45MM专用正反桨（自己的电机或需修改PID参数）
 * 在后期的程序更新中，会增加硬件代码达到预期飞行功能。敬请关注。
 *
 * @author KingBoy (2018/5/19)
 *
 * @param void
 */
void main(void)
{
	PWM_Init(); //初始化PWM
	Set_PWM(1000, 1000, 1000, 1000); //关闭电机
	LedR = 0;
	LedG = 1;
	LedB = 1; //3颗状态灯
	Delay(500); //延时一会
	LedR = 1;
	LedG = 0;
	LedB = 1; //3颗状态灯
	Delay(500); //延时一会
	LedR = 1;
	LedG = 1;
	LedB = 0; //3颗状态灯
	Delay(500); //延时一会
	LedR = 1;
	LedG = 1;
	LedB = 1; //3颗状态灯
	Delay(10);    // 延时 100

	//InitADC();/ADC模数转换 初始化（后期开发）
	Delay(10); //延时 100
	Init_MPU6050(); //初始化MPU-6050
	Delay(10); //延时 100
	LT8910_Init(); //无线2.4G模块初始化
	Delay(100); //延时一会 1S

	Timer0Init(); //初始化定时器
	Delay(100);   //延时一会 1S
	/*默认值初始化*/
	throttle = 0;   //初始化油门变量
	Yaw = 128;      //初始化航向变量
	Roll = 128;     //初始化横滚变量
	Pitch = 128;    //初始化俯仰变量
	LedB = 0;       //开启绿灯
	a_x = 0;        //横滚手动值
	a_y = 0;        //俯仰手动值
	a_z = 0;        //航向手动值

	//Flight();//编译后2个警告是说 飞控函数中断量 不在主函数里【不需要纠结】
	EA = 1;  //开总中断
	while (1)
	{
#if 0
		//要放到定时器中断里运行
		RX_model();             //接收模式
		js_shuju(RxBuf, 15);     //读取数据包
#endif
		/*控制指令接收正确*/
		if (MAC_calc(RxBuf, 10, RxBuf[10]) == 0)
		{
			LostCom = RxBuf[0];                     //接收 失联变量
			LockState = RxBuf[1];                   //接收 命令值 1=上锁  5=解锁
			throttle = RxBuf[2] * 0xff + RxBuf[3];  //接收 油门变量
			Yaw = RxBuf[4];                         //接收 航向变量
			Roll = RxBuf[5];                        //接收 横滚变量
			Pitch = RxBuf[6];                       //接收 俯仰变量
			a_x = RxBuf[7] - 128;                   //读出 X轴保存值
			a_y = RxBuf[8] - 128;                   //读出 Y轴保存值
			a_z = RxBuf[9] - 128;                   //读出 Z轴保存值
			LedG = 1;                               //LED 绿灯灭
		}
		else
		{
			LedG = 0;                               //LED 绿灯亮
		}

		if (LockState == 1)                         //遥控命令 上锁
		{
			LedB = 1;                               //航向灯 蓝色灭
		}
		if (LockState == 5)                         //遥控命令 解锁
		{
			LedB = 0;                               //航向灯 蓝色亮
		}
		Delay(3);     //延时一会
#if 0
		//ADC电压低压检测自停保护功能 后期开发
		ADC_CONTR
#endif
	}
}

/**
 * 定时器0 初始化函数
 * 12毫秒@24.000MHz
 *
 * @author KingBoy (2018/5/19)
 *
 * @param void
 */
void Timer0Init(void)
{
	AUXR &= 0x7F;   //定时器时钟12T模式
	TMOD &= 0xF0;   //设置定时器模式
	TL0 = 0x40;     //设置定时初值
	TH0 = 0xA2;     //设置定时初值
	TF0 = 0;        //清除TF0标志
	TR0 = 1;        //定时器0开始计时
	ET0 = 1;        //Timer0 Interrupt Enable
}
/**
 * 时间延时 函数
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
 * PID算法飞控自平衡函数，定时器0中断12毫秒执行一次
 *
 * @author KingBoy (2018/5/20)
 *
 * @param void
 */
void Flight(void) interrupt 1
{
	/*LT8910设置为接收模式*/
	RX_model();
	/*接收15字节数据*/
	ReceiveData(RxBuf, 15);


	Read_MPU6050(tp); //直接读取MPU6050陀螺仪和加速度的数据包

	Angle_ax = KalmanFilter_ax(((int *)&tp)[0], KALMAN_Q, KALMAN_R);  //低通滤波，见文档解释
	Angle_ay = KalmanFilter_ay(((int *)&tp)[1], KALMAN_Q, KALMAN_R);
	Angle_az = KalmanFilter_az(((int *)&tp)[2], KALMAN_Q, KALMAN_R);

	Angle_gx = ((float)(((int *)&tp)[4])) / 65.5;   //陀螺仪处理	结果单位是 +-度
	Angle_gy = ((float)(((int *)&tp)[5])) / 65.5;   //陀螺仪量程 +-500度/S, 1度/秒 对应读数 65.536
	Angle_gz = KalmanFilter_gyroz(((int *)&tp)[6], Q15(0.2), Q15(0.8));
	IMU_gz = Angle_gz / 65.5;
	Last_Angle_gx = Angle_gx;       //储存上一次角速度数据
	Last_Angle_gy = Angle_gy;

	//*********************************** 四元数解算 ***********************************
	IMUupdate(Angle_gx * 0.0174533f,
			  Angle_gy * 0.0174533f,
			  IMU_gz * 0.0174533f,
			  Angle_ax,
			  Angle_ay,
			  Angle_az); //姿态解算，精度0.1度
						 //发送到遥控器
						 //	TxBuf[0]=(AngleX+900)/0xff; // 数值是 48~1752 = 0-360度
						 //	TxBuf[1]=(AngleX+900)%0xff;	// 数值是 48~1752 = 0-360度
						 //	TxBuf[2]=(AngleY+900)/0xff;	// 数值是 48~1752 = 0-360度
						 //	TxBuf[3]=(AngleY+900)%0xff;	// 数值是 48~1752 = 0-360度
						 //****飞控失联判断 自动降落算法*********************
						 //接收遥控器发来的不断更新数据 判断联机通讯是否正常
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
	//****倾斜角度极限控制***********************************************************************
	//极限角度值   30度
	if ((AngleX + 900) > 1200)    //飞控向右倾斜
	{
		LedR = 0;
		d_throttle = 0;
	}
	else if ((AngleX + 900) < 500)    //飞控向左倾斜
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

	//****以下是飞行控制算法************************************
	//************** MPU6050 X轴指向 **************************
	delta_rc_x = ((float)Roll - 128) * 2; //得到 横滚数据变量
	Ax = -AngleX - delta_rc_x + a_x * 5; //
										 //	Ax =-AngleX+a_x*5;
	if (d_throttle > 20)
	{
		integAngleErr_X += Ax;   //外环积分(油门小于某个值时不积分)
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

	PID_P = (long)Ax * CTL_PARA_PID_ANGLEY_P;
	PID_I = ((long)integAngleErr_X * CTL_PARA_PID_ANGLEX_I) >> 15;
	PID_D = ((Angle_gy + Last_Angle_gy) / 2) * CTL_PARA_PID_ANGLEX_D;
	PID_Output = (PID_P + PID_I + PID_D + 5) / 10;  //外环PID

	Last_Ax = Ax;
	gx = PID_Output - Angle_gy;      //外环 -   陀螺仪Y轴

	if (d_throttle > 20)
	{
		ErrORX_In += gx;    //内环积分(油门小于某个值时不积分)
	}
	else
	{
		ErrORX_In = 0; //油门小于定值时清除积分值
	}

	if (ErrORX_In > INTEG_ANGLE_ERR_MAX)
	{
		ErrORX_In = INTEG_ANGLE_ERR_MAX;
	}
	else if (ErrORX_In < -INTEG_ANGLE_ERR_MAX)
	{
		ErrORX_In = -INTEG_ANGLE_ERR_MAX;   //积分限幅
	}

	PID_P = ((long)gx * CTL_PARA_PID_OMEGA_X_P) >> 15;
	PID_I = ((long)ErrORX_In * CTL_PARA_PID_OMEGA_X_I) >> 15;
	PID_D = ((long)gx - Last_gx) * CTL_PARA_PID_OMEGA_X_D;
	PID_Output = PID_P + PID_I + PID_D;   //内环PID

	Last_gx = gx;

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
	//**************MPU6050 Y轴指向**************************************************
	delta_rc_y = ((float)Pitch - 128) * 2; //得到 俯仰数据变量
	Ay = -AngleY - delta_rc_y - a_y * 5;
	//	Ay  =-AngleY-a_y*5;
	if (d_throttle > 20)
	{
		integAngleErr_Y += Ay;               //外环积分(油门小于某个值时不积分)
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

	PID_P = (long)Ay * CTL_PARA_PID_ANGLEY_P;
	PID_I = ((long)integAngleErr_Y * CTL_PARA_PID_ANGLEY_I) >> 15;
	PID_D = ((Angle_gx + Last_Angle_gx) / 2) * CTL_PARA_PID_ANGLEY_D;
	PID_Output = (PID_P + PID_I + PID_D + 5) / 10; //外环PID，“+5”为了四舍五入?

	Last_Ay = Ay;
	gy = PID_Output - Angle_gx;

	if (d_throttle > 20)
	{
		ErrORY_In += gy; //内环积分(油门小于某个值时不积分)
	}
	else
	{
		ErrORY_In = 0;                          //油门小于定值时清除积分值
	}

	if (ErrORY_In > INTEG_ANGLE_ERR_MAX)
	{
		ErrORY_In = INTEG_ANGLE_ERR_MAX;
	}
	else if (ErrORY_In < -INTEG_ANGLE_ERR_MAX)
	{
		ErrORY_In = -INTEG_ANGLE_ERR_MAX;   //积分限幅
	}

	PID_P = ((long)gy * CTL_PARA_PID_OMEGA_Y_P) >> 15;
	PID_I = ((long)ErrORY_In * CTL_PARA_PID_OMEGA_Y_I) >> 15;
	PID_D = ((long)gy - Last_gy) * CTL_PARA_PID_OMEGA_Y_D;
	PID_Output = PID_P + PID_I + PID_D;

	Last_gy = gy;

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

	//************** MPU6050 Z轴指向 *****************************
	delta_rc_z = -Angle_gz + ((float)Yaw - 128) * 65 + a_z * 20; //得到 航向数据变量 操作量
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
	PID_D = ((long)delta_rc_z - AngleZ_late) * CTL_PARA_PID_ANGLEZ_D;
	PID_Output = (PID_P + PID_I + PID_D) >> 6;

	AngleZ_late = delta_rc_z;
	speed0 = speed0 + PID_Output;
	speed1 = speed1 - PID_Output;
	speed3 = speed3 - PID_Output;
	speed2 = speed2 + PID_Output;

	//**************将速度参数加载至PWM模块*************************************************
	//速度参数控制，防止超过PWM参数范围0-1000（X型有效）
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

	//满足条件：（解锁：2.4G=5；油门大于30）才能控制电机
	if (LockState == 5 && d_throttle >= 50)
	{
		Set_PWM(1000 - PWM1, 1000 - PWM2, 1000 - PWM3, 1000 - PWM0);
	} //启动PWM
	else
	{
		Set_PWM(1000, 1000, 1000, 1000);
	} //关闭PWM
}

