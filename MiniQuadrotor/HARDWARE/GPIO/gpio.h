#ifndef __GPIO_H
#define __GPIO_H
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

#define MPU9250_nCS PAout(3)
#define MS5611_CSB PAout(4)
#define CC1101_CSN PEout(15)

#define MPU9250_INT PCin(4)

void MPU9250_MS5611_CC1101_CS_GPIO_Init(void);

#endif
