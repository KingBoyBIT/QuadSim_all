#ifndef _STC15W4KPWM_H_
#define _STC15W4KPWM_H_
/**
 * 单片机所有IO口初始化-PWM口初始化 函数
 *
 * @author KingBoy (2018/5/20)
 *
 * @param void
 */
void PWM_Init(void);
/**
 * 硬件PWM空占比控制
 * 函数本函数输入的4个值取值范围为0-1000,1000电机停，0转速最高，
 *
 * @author KingBoy (2018/5/20)
 *
 * @param PWMa PWM2T
 * @param PWMb PWM3T
 * @param PWMc PWM4T
 * @param PWMd PWM5T
 */
void Set_PWM(unsigned int PWMa, unsigned int PWMb, unsigned int PWMc, unsigned int PWMd);
#endif  