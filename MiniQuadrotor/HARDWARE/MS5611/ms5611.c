#include "ms5611.h"
#include "include.h"
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

u16 MS5611_PROM[8]={0};

void MS5611_Init(void)
{
	char i=0;
	u8 temp_H=0;
	u8 temp_L=0;
	
	MS5611_Reset();
	
	for(i=0;i<8;i++)
	{
		MS5611_CSB=0;
		SPI1_ReadWriteByte(PROM_RD+i*2);
		temp_H=SPI1_ReadWriteByte(0x00);
		temp_L=SPI1_ReadWriteByte(0x00);
		MS5611_CSB=1;
		
		MS5611_PROM[i]=((temp_H<<8)|temp_L);
	}
	
	MS5611_T_ADC_Start();
	delay_ms(10);
}

void MS5611_P_ADC_Start(void)
{
	MS5611_CSB=0;
	SPI1_ReadWriteByte(CONV+OSR+D1);
	MS5611_CSB=1;
}

void MS5611_T_ADC_Start(void)
{
	MS5611_CSB=0;
	SPI1_ReadWriteByte(CONV+OSR+D2);
	MS5611_CSB=1;
}

u32 MS5611_ADC_Read(void)
{
	u8 temp_HH=0;
	u8 temp_HL=0;
	u8 temp_LH=0;
	u8 temp_LL=0;
	u32 Temp=0;
	
	MS5611_CSB=0;
	temp_HH=SPI1_ReadWriteByte(READ);
	temp_HL=SPI1_ReadWriteByte(0x00);
	temp_LH=SPI1_ReadWriteByte(0x00);
	temp_LL=SPI1_ReadWriteByte(0x00);
	MS5611_CSB=1;
	
	Temp=((temp_HH<<24)|(temp_HL<<16)|(temp_LH<<8)|temp_LL);
	
	return (Temp&0x00FFFFFF);
}

void MS5611_Reset(void)
{
	MS5611_CSB=0;
	SPI1_ReadWriteByte(Reset);
	delay_ms(3);
	MS5611_CSB=1;
}
