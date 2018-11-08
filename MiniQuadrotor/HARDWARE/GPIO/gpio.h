#ifndef __GPIO_H
#define __GPIO_H
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

#define MPU9250_nCS PAout(3)
#define MS5611_CSB PAout(4)
#define CC1101_CSN PEout(15)

#define MPU9250_INT PCin(4)

void MPU9250_MS5611_CC1101_CS_GPIO_Init(void);

#endif
