#ifndef FLIGHTCONTROL_H
#define FLIGHTCONTROL_H


void PIDcontrolX();
void PIDcontrolY();
void PIDcontrolZ();

void DangerMotionProtect();
void PWMoutput();
/**
 * ʧ�����Ʊ���
 *
 * @author KingBoy (2018/10/14)
 *
 * @param void
 */
void LostControlProtect();
#endif