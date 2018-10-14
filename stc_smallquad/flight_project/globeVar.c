#include "globeVar.h"

/*全局变量定义*/
unsigned char data RxBuf[20];		//设置接收长度，最高为32字节

/*飞控控制参数*/
unsigned int Ctl_throttle = 0;		//油门变化速度控制，不这样做的话快速变化油门时四轴会失速翻转并坠毁
/*电机速度参数*/
int speed0 = 0;
int speed1 = 0;
int speed2 = 0;
int speed3 = 0;
/*加载至PWM模块的参数*/
int PWM0 = 0;
int PWM1 = 0;
int PWM2 = 0;
int PWM3 = 0;

//double g_x=0,g_y=0,g_z=0;				//陀螺仪矫正参数
/*角度矫正参数*/
char rcAngle_X_offset = 0;
char rcAngle_Y_offset = 0;
char rcAngle_Z_offset = 0;

float IMU_gz;//定义未知

/*MPU-6050寄存器数据参数*/
double Gyro_y = 0;						//Y轴陀螺仪数据暂存
double Gyro_x = 0;						//X轴陀螺仪数据暂存
double Gyro_z = 0;						//Z轴陀螺仪数据暂存
int xdata Angle_ax = 0;					//由加速度计算的加速度(弧度制)
int xdata Angle_ay = 0;					//由加速度计算的加速度(弧度制)
int xdata Angle_az = 0;					//由加速度计算的加速度(弧度制)
int idata Omega_gy = 0;					//由角速度计算的角速率(角度制)
int idata Omega_gx = 0;					//由角速度计算的角速率(角度制)
int idata Omega_gz = 0;					//由角速度计算的角速率(角度制)

int data AngleXest = 0, AngleYest = 0;		//四元数解算出的欧拉角  ,AngleZ=0

/*acc16*3+tempreture16+gyro16*3直接读取MPU6050陀螺仪和加速度的数据包（用掉14字节）*/
unsigned char data IMUdata[16];				
 

/*飞行控制变量*/
unsigned char LockState;				//断开/连接 解锁变量
unsigned char RCnum;					//通讯状态 变量
unsigned char RCnum_pre;					//遥控器指令流水号
unsigned char LostComCount;				//失联计数变量
unsigned int rc_throttle;					//油门变量
unsigned int rc_Yaw;						//航向变量
unsigned int rc_Roll;			   			//横滚变量
unsigned int rc_Pitch;			   			//俯仰变量
//unsigned char PitchRoll;	   			//俯仰横滚变量
unsigned char CKV_calc;		   			//CRC 算出效验包
unsigned char CKV_match;	   			//CRC 接收效验包

/*PID算法变量*/
float data PID_Output;					//PID最终输出量

float xdata Last_Angle_gx = 0;			//外环PI输出量  上一次陀螺仪数据
float xdata Last_Omega_gy = 0;			//外环PI输出量  上一次陀螺仪数据

float xdata integOmegaErr_X = 0;				//内环P 内环I 内环D 内环误差积分
float xdata integAngleErr_X = 0;		//外环P 外环I       外环误差积分

float xdata integOmegaErr_Y = 0;				//内环P 内环I 内环D 内环误差积分
float xdata integAngleErr_Y = 0;		//外环P 外环I       外环误差积分

float xdata integAngleErr_Z = 0;		//外环P 外环I       外环误差积分
float xdata LastAngle_Z = 0;			//Z轴相关??

float xdata LastAngleErr_X = 0;				//角度更新
float xdata LastAngleErr_Y = 0;				//角度更新

float xdata LastOmegaErr_X = 0;				//角速度更新
float xdata LastOmegaErr_Y = 0;				//角速度更新

float xdata AngleErr_X = 0, AngleErr_Y = 0, AngleErr_Z = 0;		//加入遥控器控制量后的角度
int xdata OmegaErr_X = 0, OmegaErr_Y = 0;				//加入遥控器控制量后的角速度
//long xdata g_x_aver = 0;				//没用到？
//long xdata g_y_aver = 0;				//没用到？
//long xdata g_z_aver = 0;				//没用到？
int delta_rc_x = 0;						//暂不确定，待更新
int delta_rc_y = 0;						//暂不确定，待更新
int delta_rc_z = 0;						//暂不确定，待更新
long idata PID_P;						//PID计算临时缓存值
long idata PID_I;						//PID计算临时缓存值
long idata PID_D;						//PID计算临时缓存值

