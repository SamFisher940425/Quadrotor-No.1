#ifndef __MS5611_H
#define __MS5611_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//ADC ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 
 
//MS5611 define

#define Reset  0x1e
#define	CONV   0x40
#define READ   0x00
#define OSR	   0x08
#define D1	   0x00
#define	D2	   0x10
#define	PROM_RD	0xA0

extern u16 MS5611_PROM[8];

void MS5611_Init(void);
void MS5611_Reset(void);
void MS5611_P_ADC_Start(void);
void MS5611_T_ADC_Start(void);
u32 MS5611_ADC_Read(void);
 
#endif
