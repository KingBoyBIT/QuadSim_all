#ifndef _GLOBE_VAR_H_
#define _GLOBE_VAR_H_

/*全局变量定义*/
extern int ret;
extern unsigned char data RxBuf[20]; //设置接收长度，最高为32字节

/*飞控控制参数*/
extern unsigned int Ctl_throttle;//油门变化速度控制，不这样做的话快速变化油门时四轴会失速翻转并坠毁
/*电机速度参数*/
extern int speed0;
extern int speed1;
extern int speed2;
extern int speed3;
/*加载至PWM模块的参数*/
extern int PWM0;
extern int PWM1;
extern int PWM2;
extern int PWM3;

//double g_x=0,g_y=0,g_z=0;       //陀螺仪矫正参数
/*角度矫正参数*/
extern char rcAngle_X_offset;
extern char rcAngle_Y_offset;
extern char rcAngle_Z_offset;

extern float IMU_gz;//定义未知

/*MPU-6050寄存器数据参数*/
#if 0
extern double Gyro_y; //Y轴陀螺仪数据暂存
extern double Gyro_x; //X轴陀螺仪数据暂存
extern double Gyro_z; //Z轴陀螺仪数据暂存
#endif
extern int xdata Angle_a[3];//由加速度计算的加速度(弧度制)
extern int idata Omega_g[3];				//由角速度计算的角速率(角度制)


extern int data AngleXest, AngleYest;//四元数解算出的欧拉角  ,AngleZ=0

extern unsigned char data IMUdata[16];//acc16*3+tempreture16+gyro16*3直接读取MPU6050陀螺仪和加速度的数据包（用掉14字节）
 

/*飞行控制变量*/
extern unsigned char LockState;//断开/连接 解锁变量
extern unsigned char RCnum;//通讯状态 变量
extern unsigned char RCnum_pre;//失联变量
extern unsigned char LostComCount;//失联计数变量
extern unsigned int rc_throttle;//油门变量
extern unsigned int rc_Yaw;//航向变量
extern unsigned int rc_Roll;//横滚变量
extern unsigned int rc_Pitch;//俯仰变量
//extern unsigned char PitchRoll;//俯仰横滚变量
extern unsigned char CKV_calc;//CRC 算出效验包
extern unsigned char CKV_match;//CRC 接收效验包

/*PID算法变量*/
extern float data PID_Output;//PID最终输出量

extern float xdata Last_Angle_gx;//外环PI输出量  上一次陀螺仪数据
extern float xdata Last_Omega_gy;//外环PI输出量  上一次陀螺仪数据

extern float xdata integOmegaErr_X;//内环P 内环I 内环D 内环误差积分
extern float xdata integAngleErr_X;//外环P 外环I       外环误差积分

extern float xdata integOmegaErr_Y;//内环P 内环I 内环D 内环误差积分
extern float xdata integAngleErr_Y;//外环P 外环I       外环误差积分

extern float xdata integAngleErr_Z;//外环P 外环I       外环误差积分
extern float xdata LastAngle_Z;//Z轴相关??

extern float xdata LastAngleErr_X;//角度更新
extern float xdata LastAngleErr_Y;//角度更新

extern float xdata LastOmegaErr_X;//角速度更新
extern float xdata LastOmegaErr_Y;//角速度更新

extern float xdata AngleErr[3];		//[x,y,z]加入遥控器控制量后的角度 Z未用到

extern int xdata OmegaErr_X, OmegaErr_Y;//加入遥控器控制量后的角速度
//extern long xdata g_x_aver;//没用到？
//extern long xdata g_y_aver;//没用到？
//extern long xdata g_z_aver;//没用到？
extern int delta_rc[3];					//[x,y,z] 暂不确定，待更新

extern long idata PID_P;//暂不确定，待更新
extern long idata PID_I;//暂不确定，待更新
extern long idata PID_D;//暂不确定，待更新


#endif
