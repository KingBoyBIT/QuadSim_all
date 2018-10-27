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
#include "FlightControl.h"
#include "Uart.h"



/**
 * 定时器0 初始化函数
 * 12毫秒@24.000MHz
 *
 * @author KingBoy (2018/5/19)
 *
 * @param void
 */
void TimerInit(void);
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
	unsigned char i = 0;

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

	TimerInit(); //初始化定时器
	UartInit(BAUD, FOSC); //初始化串口
	Delay(100);   //延时一会 1S
	/*默认值初始化*/
	rc_throttle = 0;   //初始化油门变量
	rc_Yaw = 128;      //初始化航向变量
	rc_Roll = 128;     //初始化横滚变量
	rc_Pitch = 128;    //初始化俯仰变量
	LedB = 0;       //开启绿灯
	rcAngle_X_offset = 0;        //横滚手动值
	rcAngle_Y_offset = 0;        //俯仰手动值
	rcAngle_Z_offset = 0;        //航向手动值

	//Flight();//编译后2个警告是说 飞控函数中断量 不在主函数里【不需要纠结】
	ES = 1; //使能串口1中断
	EA = 1;  //开总中断
	UartSendStr("Uart Test !\r\n");
	while (1)
	{
#if 0
		//要放到定时器中断里运行
		RX_model();             //接收模式
		js_shuju(RxBuf, 15);     //读取数据包
#endif

		/*控制指令接收正确则缓存，否则丢弃*/
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
				//Delay(300);     //延时一会
			}
#endif
			RCnum = RxBuf[0];                         //接收 失联变量
			LockState = RxBuf[1];                       //接收 命令值 1=上锁  5=解锁
			rc_throttle = RxBuf[2] * 0xff + RxBuf[3];   //接收 油门变量
			rc_Yaw = RxBuf[4];                          //接收 航向摇杆参数
			rc_Roll = RxBuf[5];                         //接收 横滚摇杆参数
			rc_Pitch = RxBuf[6];                        //接收 俯仰摇杆参数
			rcAngle_X_offset = RxBuf[7] - 128;          //读出 横滚微调变量
			rcAngle_Y_offset = RxBuf[8] - 128;          //读出 俯仰微调变量
			rcAngle_Z_offset = RxBuf[9] - 128;          //读出 航向微调变量
			LedG = 1;                                   //LED 绿灯灭
		}
		else
		{
			LedG = 0;                               //LED 绿灯亮
		}
		if (LockState == RC_LOCK)                         //遥控命令 上锁
		{
			LedB = 1;                               //航向灯 蓝色灭
		}
		if (LockState == RC_UNLOCK)                         //遥控命令 解锁
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
void TimerInit(void)
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
	ReceiveRC_Data(RxBuf, 15);

	/*获取惯导数据*/
	Read_MPU6050(IMUdata); //直接读取MPU6050陀螺仪和加速度的数据包

	Angle_ax = RCLowPassFilter_ax(((int *)&IMUdata)[0], RC_KALMAN_Q, RC_KALMAN_R);  //低通滤波，见文档解释
	Angle_ay = RCLowPassFilter_ay(((int *)&IMUdata)[1], RC_KALMAN_Q, RC_KALMAN_R);
	Angle_az = RCLowPassFilter_az(((int *)&IMUdata)[2], RC_KALMAN_Q, RC_KALMAN_R);

	Omega_gx = ((float)(((int *)&IMUdata)[4])) / DEGPSEC;   //陀螺仪处理	结果单位是 +-度
	Omega_gy = ((float)(((int *)&IMUdata)[5])) / DEGPSEC;   //陀螺仪量程 +-500度/S, 1度/秒 对应读数 65.536
	Omega_gz = RCLowPassFilter_gyroz(((int *)&IMUdata)[6], Q15(0.2), Q15(0.8));
	IMU_gz = Omega_gz / DEGPSEC;
	Last_Angle_gx = Omega_gx;       //储存上一次角速度数据
	Last_Omega_gy = Omega_gy;

	/*四元数解算*/
	//姿态解算，精度0.1度
	IMUupdate(Omega_gx * DEG2RAD, Omega_gy * DEG2RAD, IMU_gz * DEG2RAD, Angle_ax, Angle_ay, Angle_az);

	/*飞控失联判断 自动降落算法*/
	//接收遥控器发来的不断更新数据 判断联机通讯是否正常
	LostControlProtect();
	/*姿态角失控保护*/
	DangerMotionProtect();

	/*飞行控制算法*/
	PIDcontrolX();
	PIDcontrolY();
	PIDcontrolZ();
	/*将速度参数加载至PWM模块*/
	//速度参数控制，防止超过PWM参数范围0-1000（X型有效）
	PWMoutput();

	//满足条件：（解锁：2.4G=5；油门大于30）才能控制电机
	if (LockState == RC_UNLOCK && Ctl_throttle >= 50)
	{
		Set_PWM(1000 - PWM1, 1000 - PWM2, 1000 - PWM3, 1000 - PWM0);
	} //启动PWM
	else
	{
		Set_PWM(1000, 1000, 1000, 1000);
	} //关闭PWM（1000为关）

#if 0
	//调试强行关闭电机
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
