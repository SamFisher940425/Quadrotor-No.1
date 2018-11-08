#ifndef __MS5611_H
#define __MS5611_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//ADC 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
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
