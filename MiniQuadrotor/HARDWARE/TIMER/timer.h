#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//��ʱ�� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/6/16
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//�޸�˵��
//V1.2 20140504
//����TIM5_CH1_Cap_Init����,�������벶��
//////////////////////////////////////////////////////////////////////////////////

void TIM4_Int_Init(u16 arr,u16 psc);

void TIM1_CH1_Cap_Init(u32 arr,u16 psc);
void TIM1_CH2_Cap_Init(u32 arr,u16 psc);
void TIM1_CH3_Cap_Init(u32 arr,u16 psc);
void TIM1_CH4_Cap_Init(u32 arr,u16 psc);

void TIM2_CH1_Cap_Init(u32 arr,u16 psc);	

#endif
