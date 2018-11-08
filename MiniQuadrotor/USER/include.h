#ifndef _INCLUDE_H
#define _INCLUDE_H

#include "stm32f4xx.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"

#include "math.h" 
#include "arm_math.h"

#include "gpio.h"
#include "led.h"
#include "pwm.h"
#include "timer.h"
#include "spi.h"
#include "myiic.h"
#include "adc.h"
#include "cc1101.h"
#include "ms5611.h"
#include "mpu9250.h"
#include "inv_mpu.h"
#include "Control.h"

extern unsigned long Int_Cnt;

extern volatile unsigned char Control_Flag,Unlock_Cnt,Unlock_Flag;

extern volatile float Pitch[2],Roll[2],Yaw[2];
extern float DMP_Pitch,DMP_Roll,DMP_Yaw;
extern float MPL_Pitch,MPL_Roll,MPL_Yaw;

extern float Err_DMP_Pitch,Err_DMP_Roll,Err_DMP_Yaw;
extern float Err_MPL_Pitch,Err_MPL_Roll,Err_MPL_Yaw;

extern double P_Pitch,Kg_Pitch;
extern double P_Roll,Kg_Roll;
extern double P_Yaw,Kg_Yaw;

extern double Weight_DMP_Pitch,Weight_MPL_Pitch,Weight_Pitch_Sum;
extern double Weight_DMP_Roll,Weight_MPL_Roll,Weight_Roll_Sum;
extern double Weight_DMP_Yaw,Weight_MPL_Yaw,Weight_Yaw_Sum;

extern unsigned int d1,d2;
extern volatile double P[2],T;
extern double dT,OFF,SENS;
extern double P_Presure,Kg_Presure;

extern volatile short Gyo_Raw[3][2],Acc_Raw[3][2],Mag_Raw[3][2];

extern volatile double Throttle,Pitch_Angle_Want,Roll_Angle_Want,Yaw_Angle_Want,Yaw_Angle_Change_Want;
extern volatile int Motor_Out_1,Motor_Out_2,Motor_Out_3,Motor_Out_4;
extern float Pitch_Bias,Roll_Bias,Yaw_Bias;

extern u8   TIM1CH1_CAPTURE_STA;
extern u32	TIM1CH1_CAPTURE_VAL;

extern u8   TIM1CH2_CAPTURE_STA;
extern u32	TIM1CH2_CAPTURE_VAL;

extern u8   TIM1CH3_CAPTURE_STA;
extern u32	TIM1CH3_CAPTURE_VAL;

extern u8   TIM1CH4_CAPTURE_STA;
extern u32	TIM1CH4_CAPTURE_VAL;

extern u8   TIM2CH1_CAPTURE_STA;
extern u32	TIM2CH1_CAPTURE_VAL;

extern volatile long TIM1CH1_Val;
extern volatile long TIM1CH2_Val;
extern volatile long TIM1CH3_Val;
extern volatile long TIM1CH4_Val;

extern volatile long TIM2CH1_Val;

#endif
