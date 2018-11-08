#ifndef _CONTROL_H
#define _CONTROL_H
#include "sys.h"

void Pitch_PID(void);
void Roll_PID(void);
void Yaw_PID(void);
void Motor_Out(void);

void Pitch_PID_Clear(void);
void Roll_PID_Clear(void);
void Yaw_PID_Clear(void);

#endif
