#ifndef FLIGHTCONTROL_H
#define FLIGHTCONTROL_H


void PIDcontrolX();
void PIDcontrolY();
void PIDcontrolZ();

void DangerMotionProtect();
void PWMoutput();
void LostControlProtect();
#endif