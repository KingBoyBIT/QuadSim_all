#ifndef _ALLDEF_H_
#define _ALLDEF_H_

#include <STC15W4K60S4.h>	//STC15W4K48S4 专用头文件
#include <intrins.h>		//STC特殊命令符号声明

typedef unsigned char U8,BYTE,UINT8,UCHAR,uchar;
typedef unsigned int uint;
typedef unsigned short u16;

#define BAUD 115200 //串口波特率
#define FOSC 24000000L //系统频率


//==================================================//
//  LED灯 引脚定义
//==================================================//
sbit LedR = P0 ^ 1; //LED 红色 R,灌入式，低电平亮
sbit LedG = P0 ^ 2; //LED 绿色 G,灌入式，低电平亮
sbit LedB = P0 ^ 3; //LED 蓝色 B,灌入式，低电平亮

sbit KARX = P3 ^ 0; //排针接口 RXD
sbit KATX = P3 ^ 1; //排针接口 TXD



/*绝对值小于1的小数转为整数*/
#define Q15(X) ((X < 0.0) ? (int)(32768*(X) - 0.5) : (int)(32767*(X) + 0.5))
#define RC_KALMAN_Q	Q15(0.20)
#define RC_KALMAN_R	Q15(0.80)
//==================================================//
//   PID 手动微调参数值
//   修改PID以下参数 可以吧飞控调的更加平稳
//   具体调试方法颗参考学习资料
//==================================================//
#define	CTL_PARA_PID_ANGLEX_P	12.0f	    	//外环P
#define	CTL_PARA_PID_ANGLEX_I	Q15(0.12)   	//外环I
#define	CTL_PARA_PID_ANGLEX_D	5.2f	    	//外环D

#define	CTL_PARA_PID_OMEGA_X_P	Q15(0.31) 		//内环P
#define	CTL_PARA_PID_OMEGA_X_I	Q15(0.012)		//内环I
#define	CTL_PARA_PID_OMEGA_X_D	4.3f			//内环D


#define	CTL_PARA_PID_ANGLEY_P	12.0f	    	//外环P
#define	CTL_PARA_PID_ANGLEY_I	Q15(0.12)   	//外环I
#define	CTL_PARA_PID_ANGLEY_D	5.2f	    	//外环D

#define	CTL_PARA_PID_OMEGA_Y_P	Q15(0.31) 		//内环P
#define	CTL_PARA_PID_OMEGA_Y_I	Q15(0.012)		//内环I
#define	CTL_PARA_PID_OMEGA_Y_D	4.3f			//内环D

#define	CTL_PARA_PID_ANGLEZ_P	5.0f			//Z轴 P
#define CTL_PARA_PID_ANGLEZ_I	Q15(0.24)		//Z轴 I
#define	CTL_PARA_PID_ANGLEZ_D	4.0f			//Z轴 D

#define	INTEG_ANGLE_ERR_MAX		800				//积分限幅

#define	MATH_PI_F				3.14159265f
#define MATH_180_PI_10			572.957795f		//10*180/pi
#define	NAV_PARA_EST_KP			2.0f			//AHRS递推
#define	NAV_PARA_EST_KI			0.001f			//AHRS递推
#define NAV_PARA_EST_KD			0.001f			//AHRS递推
#define	NAV_PARA_EST_HALF_T		0.005f			//滤波更新时间周期
/*遥控器设置*/
#define RC_UNLOCK				5
#define RC_LOCK					1

/*MPU6050*/
#define	SMPLRT_DIV				0x19			//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG					0x1A			//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG				0x1B			//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG			0x1C			//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
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
#define	PWR_MGMT_1				0x6B			//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I				0x75			//IIC地址寄存器(默认数值0x68，只读)
#define	SLAVE_ADDR				0xD0			//IIC写入时的地址字节数据，+1为读取
#define IICSPEED        		0x24

#endif