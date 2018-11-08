#include "myiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//IIC ��������
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//��ʼ��IIC
void IIC_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��

  //GPIOB8,B9��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	IIC_SCL=1;
	IIC_SDA=1;
}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	  SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		    delay_us(2);   //��TEA5767��������ʱ���Ǳ����
	    	IIC_SCL=1;
		    delay_us(2); 
		    IIC_SCL=0;	
		    delay_us(2);
    }	 
}
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	  {
        IIC_SCL=0; 
        delay_us(2);
		    IIC_SCL=1;
			  delay_us(2);
        receive<<=1;
        if(READ_SDA)receive++;   
		    delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}

/**
 * @brief  I2C write mutiple data
 * @param  instance: instance of i2c moudle
 *         @arg chipAddr   : i2c slave addr
 *         @arg addr       : i2c slave register offset
 *         @arg addrLen    : len of slave register addr(in byte)
 *         @arg buf        : data buf
 *         @arg buf        : read len
 * @note 
 */
uint8_t I2C_BurstWrite(uint8_t chipAddr, uint32_t addr, uint32_t addrLen, uint8_t *buf, uint32_t len)//(40+18len)D
{
    uint8_t *p;
    uint8_t err;
    
    p = (uint8_t*)&addr;
    err = 0;
    chipAddr <<= 1;
    
    IIC_Start();//2D
    IIC_Send_Byte(chipAddr);//16D
    err += IIC_Wait_Ack();//2D

    while(addrLen--)
    {
        IIC_Send_Byte(*p++);//16D
        err += IIC_Wait_Ack();//2D
    }
    
    while(len--)
    {
        IIC_Send_Byte(*buf++);//16D
        err += IIC_Wait_Ack();//2D
    }

    IIC_Stop();//2D
    return err;
}

/**
 * @brief  I2C read mutiple data
 * @param  instance: instance of i2c moudle
 *         @arg chipAddr   : i2c slave addr
 *         @arg addr       : i2c slave register offset
 *         @arg addrLen    : len of slave register addr(in byte)
 *         @arg buf        : data buf
 *         @arg buf        : read len
 * @note 
 */
int32_t I2C_BurstRead(uint8_t chipAddr, uint32_t addr, uint32_t addrLen, uint8_t *buf, uint32_t len)//(80+19(len-1))D
{
    uint8_t *p;
    uint8_t err;
    
    p = (uint8_t*)&addr;
    err = 0;
    chipAddr <<= 1;
    
    IIC_Start();//2D
    IIC_Send_Byte(chipAddr);//16D
    err += IIC_Wait_Ack();//2D
    
    while(addrLen--)
    {
        IIC_Send_Byte(*p++);//16D
        err += IIC_Wait_Ack();//2D
    }
    
    IIC_Start();//2D
    IIC_Send_Byte(chipAddr+1);//16D
    err += IIC_Wait_Ack();//2D
    
    while(len--)
    {
        *buf++ = IIC_Read_Byte(0);//16D
        if(len)
        {
            IIC_Ack();//3D
        }
    }
    
    IIC_NAck();//4D
    IIC_Stop();//2D
    
    return err;
}
