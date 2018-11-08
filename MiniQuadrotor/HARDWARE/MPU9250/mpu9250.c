#include "mpu9250.h"
#include "include.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F746开发板
//MPU9250驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/12/30
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//////////////////////////////////////////////////////////////////////////
//Register Map for Gyroscope and Accelerometer
#define MPU9250_SELF_TEST_X_GYRO        0x00
#define MPU9250_SELF_TEST_Y_GYRO        0x01
#define MPU9250_SELF_TEST_Z_GYRO        0x02

#define MPU9250_SELF_TEST_X_ACCEL       0x0D
#define MPU9250_SELF_TEST_Y_ACCEL       0x0E
#define MPU9250_SELF_TEST_Z_ACCEL       0x0F

#define MPU9250_XG_OFFSET_H             0x13
#define MPU9250_XG_OFFSET_L             0x14
#define MPU9250_YG_OFFSET_H             0x15
#define MPU9250_YG_OFFSET_L             0x16
#define MPU9250_ZG_OFFSET_H             0x17
#define MPU9250_ZG_OFFSET_L             0x18
#define MPU9250_SMPLRT_DIV              0x19
#define MPU9250_CONFIG                  0x1A
#define MPU9250_GYRO_CONFIG             0x1B
#define MPU9250_ACCEL_CONFIG            0x1C
#define MPU9250_ACCEL_CONFIG2           0x1D
#define MPU9250_LP_ACCEL_ODR            0x1E
#define MPU9250_WOM_THR                 0x1F

#define MPU9250_FIFO_EN                 0x23
#define MPU9250_I2C_MST_CTRL            0x24
#define MPU9250_I2C_SLV0_ADDR           0x25
#define MPU9250_I2C_SLV0_REG            0x26
#define MPU9250_I2C_SLV0_CTRL           0x27
#define MPU9250_I2C_SLV1_ADDR           0x28
#define MPU9250_I2C_SLV1_REG            0x29
#define MPU9250_I2C_SLV1_CTRL           0x2A
#define MPU9250_I2C_SLV2_ADDR           0x2B
#define MPU9250_I2C_SLV2_REG            0x2C
#define MPU9250_I2C_SLV2_CTRL           0x2D
#define MPU9250_I2C_SLV3_ADDR           0x2E
#define MPU9250_I2C_SLV3_REG            0x2F
#define MPU9250_I2C_SLV3_CTRL           0x30
#define MPU9250_I2C_SLV4_ADDR           0x31
#define MPU9250_I2C_SLV4_REG            0x32
#define MPU9250_I2C_SLV4_DO             0x33
#define MPU9250_I2C_SLV4_CTRL           0x34
#define MPU9250_I2C_SLV4_DI             0x35
#define MPU9250_I2C_MST_STATUS          0x36
#define MPU9250_INT_PIN_CFG             0x37
#define MPU9250_INT_ENABLE              0x38

#define MPU9250_INT_STATUS              0x3A
#define MPU9250_ACCEL_XOUT_H            0x3B
#define MPU9250_ACCEL_XOUT_L            0x3C
#define MPU9250_ACCEL_YOUT_H            0x3D
#define MPU9250_ACCEL_YOUT_L            0x3E
#define MPU9250_ACCEL_ZOUT_H            0x3F
#define MPU9250_ACCEL_ZOUT_L            0x40
#define MPU9250_TEMP_OUT_H              0x41
#define MPU9250_TEMP_OUT_L              0x42
#define MPU9250_GYRO_XOUT_H             0x43
#define MPU9250_GYRO_XOUT_L             0x44
#define MPU9250_GYRO_YOUT_H             0x45
#define MPU9250_GYRO_YOUT_L             0x46
#define MPU9250_GYRO_ZOUT_H             0x47
#define MPU9250_GYRO_ZOUT_L             0x48
#define MPU9250_EXT_SENS_DATA_00        0x49
#define MPU9250_EXT_SENS_DATA_01        0x4A
#define MPU9250_EXT_SENS_DATA_02        0x4B
#define MPU9250_EXT_SENS_DATA_03        0x4C
#define MPU9250_EXT_SENS_DATA_04        0x4D
#define MPU9250_EXT_SENS_DATA_05        0x4E
#define MPU9250_EXT_SENS_DATA_06        0x4F
#define MPU9250_EXT_SENS_DATA_07        0x50
#define MPU9250_EXT_SENS_DATA_08        0x51
#define MPU9250_EXT_SENS_DATA_09        0x52
#define MPU9250_EXT_SENS_DATA_10        0x53
#define MPU9250_EXT_SENS_DATA_11        0x54
#define MPU9250_EXT_SENS_DATA_12        0x55
#define MPU9250_EXT_SENS_DATA_13        0x56
#define MPU9250_EXT_SENS_DATA_14        0x57
#define MPU9250_EXT_SENS_DATA_15        0x58
#define MPU9250_EXT_SENS_DATA_16        0x59
#define MPU9250_EXT_SENS_DATA_17        0x5A
#define MPU9250_EXT_SENS_DATA_18        0x5B
#define MPU9250_EXT_SENS_DATA_19        0x5C
#define MPU9250_EXT_SENS_DATA_20        0x5D
#define MPU9250_EXT_SENS_DATA_21        0x5E
#define MPU9250_EXT_SENS_DATA_22        0x5F
#define MPU9250_EXT_SENS_DATA_23        0x60

#define MPU9250_I2C_SLV0_DO             0x63
#define MPU9250_I2C_SLV1_DO             0x64
#define MPU9250_I2C_SLV2_DO             0x65
#define MPU9250_I2C_SLV3_DO             0x66
#define MPU9250_I2C_MST_DELAY_CTRL      0x67
#define MPU9250_SIGNAL_PATH_RESET       0x68
#define MPU9250_MOT_DETECT_CTRL         0x69
#define MPU9250_USER_CTRL               0x6A
#define MPU9250_PWR_MGMT_1              0x6B
#define MPU9250_PWR_MGMT_2              0x6C

#define MPU9250_FIFO_COUNTH             0x72
#define MPU9250_FIFO_COUNTL             0x73
#define MPU9250_FIFO_R_W                0x74
#define MPU9250_WHO_AM_I                0x75
#define MPU9250_XA_OFFSET_H             0x77
#define MPU9250_XA_OFFSET_L             0x78

#define MPU9250_YA_OFFSET_H             0x7A
#define MPU9250_YA_OFFSET_L             0x7B

#define MPU9250_ZA_OFFSET_H             0x7D
#define MPU9250_ZA_OFFSET_L             0x7E
//
#define MPU9250_I2C_READ 0x80

//Magnetometer register maps
#define MPU9250_AK8963_WIA                 0x00
#define MPU9250_AK8963_INFO                0x01
#define MPU9250_AK8963_ST1                 0x02
#define MPU9250_AK8963_XOUT_L              0x03
#define MPU9250_AK8963_XOUT_H              0x04
#define MPU9250_AK8963_YOUT_L              0x05
#define MPU9250_AK8963_YOUT_H              0x06
#define MPU9250_AK8963_ZOUT_L              0x07
#define MPU9250_AK8963_ZOUT_H              0x08
#define MPU9250_AK8963_ST2                 0x09
#define MPU9250_AK8963_CNTL                0x0A
#define MPU9250_AK8963_CNTL2               0x0B
#define MPU9250_AK8963_RSV                 0x0B //DO NOT ACCESS <MPU9250_AK8963_CNTL2>
#define MPU9250_AK8963_ASTC                0x0C
#define MPU9250_AK8963_TS1                 0x0D //DO NOT ACCESS
#define MPU9250_AK8963_TS2                 0x0E //DO NOT ACCESS
#define MPU9250_AK8963_I2CDIS              0x0F
#define MPU9250_AK8963_ASAX                0x10
#define MPU9250_AK8963_ASAY                0x11
#define MPU9250_AK8963_ASAZ                0x12

#define MPU9250_AK8963_I2C_ADDR 0x0C
#define MPU9250_AK8963_POWER_DOWN 0x10
#define MPU9250_AK8963_FUSE_ROM_ACCESS 0x1F
#define MPU9250_AK8963_SINGLE_MEASUREMENT 0x11
#define MPU9250_AK8963_CONTINUOUS_MEASUREMENT 0x16 //MODE 2
#define MPU9250_AK8963_DATA_READY      (0x01)
#define MPU9250_AK8963_DATA_OVERRUN    (0x02)
#define MPU9250_AK8963_OVERFLOW        (0x80)
#define MPU9250_AK8963_DATA_ERROR      (0x40)
#define MPU9250_AK8963_CNTL2_SRST 0x01

//
#define MPU9250_I2C_SLV4_EN 0x80
#define MPU9250_I2C_SLV4_DONE 0x40
#define MPU9250_I2C_SLV4_NACK 0x10
//
#define MPU9250_I2C_IF_DIS (0x10)
#define MPU9250_I2C_MST_EN (0x20)
#define MPU9250_FIFO_RST (0x04)
#define MPU9250_FIFO_ENABLE (0x40)
//
#define MPU9250_RESET 0x80
#define MPU9250_CLOCK_MASK 0xF8
#define MPU9250_CLOCK_INTERNAL 0x00
#define MPU9250_CLOCK_PLL 0x01
#define MPU9250_CLOCK_PLLGYROZ 0x03
#define MPU9250_FS_SEL_MASK 0xE7
#define MPU9250_SLEEP_MASK 0x40
//
#define MPU9250_XYZ_GYRO 0xC7
#define MPU9250_XYZ_ACCEL 0xF8
//
#define MPU9250_RAW_RDY_EN (0x01)
#define MPU9250_RAW_DATA_RDY_INT (0x01)
#define MPU9250_FIFO_OVERFLOW (0x10)
//
#define MPU9250_INT_ANYRD_2CLEAR (0x10)
#define MPU9250_LATCH_INT_EN (0x20)
//
#define MPU9250_MAX_FIFO (1024)
#define MPU9250_FIFO_SIZE_1024  (0x40)
#define MPU9250_FIFO_SIZE_2048  (0x80)
#define MPU9250_FIFO_SIZE_4096  (0xC0)

#define MPU9250_TEMP_OUT (0x80)
#define MPU9250_GYRO_XOUT (0x40)
#define MPU9250_GYRO_YOUT (0x20)
#define MPU9250_GYRO_ZOUT (0x10)
#define MPU9250_ACCEL (0x08)

//
#define SMPLRT_DIV 0
#define MPU9250_SPIx_ADDR 0x00
//////////////////////////////////////////////////////////////////////////
//
enum MPU9250_GYRO_DLPF {
    MPU9250_GYRO_DLPF_250HZ = 0,
    MPU9250_GYRO_DLPF_184HZ,
    MPU9250_GYRO_DLPF_92HZ,
    MPU9250_GYRO_DLPF_41HZ,
    MPU9250_GYRO_DLPF_20HZ,
    MPU9250_GYRO_DLPF_10HZ,
    MPU9250_GYRO_DLPF_5HZ,
    MPU9250_GYRO_DLPF_3600HZ,
    NUM_GYRO_DLPF
};

enum MPU9250_GYRO_FSR {
    MPU9250_FSR_250DPS = 0,
    MPU9250_FSR_500DPS,
    MPU9250_FSR_1000DPS,
    MPU9250_FSR_2000DPS,
    MPU9250_NUM_GYRO_FSR
};

enum MPU9250_ACCEL_DLPF {
    MPU9250_ACCEL_DLPF_460HZ = 0,
    MPU9250_ACCEL_DLPF_184HZ,
    MPU9250_ACCEL_DLPF_92HZ,
    MPU9250_ACCEL_DLPF_41HZ,
    MPU9250_ACCEL_DLPF_20HZ,
    MPU9250_ACCEL_DLPF_10HZ,
    MPU9250_ACCEL_DLPF_5HZ,
    MPU9250_ACCEL_DLPF_460HZ2,
    MPU9250_NUM_ACCEL_DLPF
};

enum MPU9250_ACCEL_FSR {
    MPU9250_FSR_2G = 0,
    MPU9250_FSR_4G,
    MPU9250_FSR_8G,
    MPU9250_FSR_16G,
    MPU9250_NUM_ACCEL_FSR
};

enum MPU9250_CLK {
    MPU9250_CLK_INTERNAL = 0,
    MPU9250_CLK_PLL,
    MPU9250_NUM_CLK
};

//static s16 MPU9250_AK8963_ASA[3] = {0, 0, 0};

//初始化MPU9250
//返回值:0,成功
//    其他,错误代码
/* 此函数仅用于软件融合使用，若采用DMP解算，请勿使用此函数初始化，请调用inv mpu中的初始化函数 */
u8 MPU9250_Init(void)
{
	/*
    u8 res=0;
    //IIC_Init();     //初始化IIC总线   此处不采用IIC通信
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);//复位MPU9250
    delay_ms(100);  //延时100ms
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);//唤醒MPU9250
    MPU_Set_Gyro_Fsr(3);					        	//陀螺仪传感器,±2000dps
	  MPU_Set_Accel_Fsr(0);					       	 	//加速度传感器,±2g
    MPU_Set_Rate(50);						       	 	//设置采样率50Hz
    MPU_Write_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0X00);   //关闭所有中断
	  MPU_Write_Byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0X00);//I2C主模式关闭
	  MPU_Write_Byte(MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	  MPU_Write_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0X82);//INT引脚低电平有效，开启bypass模式，可以直接读取磁力计
    res=MPU_Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG);  //读取MPU6500的ID
    if(res==MPU6500_ID) //器件ID正确
    {
        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//设置CLKSEL,PLL X轴为参考
        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//加速度与陀螺仪都工作
		    MPU_Set_Rate(50);						       	//设置采样率为50Hz   
    }else return 1;
 
    res=MPU_Read_Byte(AK8963_ADDR,MAG_WIA);    			//读取AK8963 ID   
    if(res==AK8963_ID)
    {
        MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11);		//设置AK8963为单次测量模式
    }else return 1;
*/
	u8 data = 0, state = 0;
	uint8_t response[3] = {0, 0, 0};
	//Lower level hardware Init
	//////////////////////////////////////////////////////////////////////////
	//MPU9250 Reset
	MPU_Write_Byte(MPU9250_SPIx_ADDR, MPU9250_PWR_MGMT_1, MPU9250_RESET);
	delay_ms(100);
	//MPU9250 Set Clock Source
	MPU_Write_Byte(MPU9250_SPIx_ADDR, MPU9250_PWR_MGMT_1,  MPU9250_CLOCK_PLLGYROZ);
	delay_ms(1);
	//MPU9250 Set Interrupt
	MPU_Write_Byte(MPU9250_SPIx_ADDR, MPU9250_INT_PIN_CFG,  MPU9250_INT_ANYRD_2CLEAR);
	delay_ms(1);
	MPU_Write_Byte(MPU9250_SPIx_ADDR, MPU9250_INT_ENABLE, ENABLE);
	delay_ms(1);
	//MPU9250 Set Sensors
	MPU_Write_Byte(MPU9250_SPIx_ADDR, MPU9250_PWR_MGMT_2, MPU9250_XYZ_GYRO & MPU9250_XYZ_ACCEL);
	delay_ms(1);
	//MPU9250 Set SampleRate
	//SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV)
	MPU_Write_Byte(MPU9250_SPIx_ADDR, MPU9250_SMPLRT_DIV, SMPLRT_DIV);
	delay_ms(1);
	//MPU9250 Set Full Scale Gyro Range
	//Fchoice_b[1:0] = [00] enable DLPF
	MPU_Write_Byte(MPU9250_SPIx_ADDR, MPU9250_GYRO_CONFIG, (MPU9250_FSR_2000DPS << 3));
	delay_ms(1);
	//MPU9250 Set Full Scale Accel Range PS:2G
	MPU_Write_Byte(MPU9250_SPIx_ADDR, MPU9250_ACCEL_CONFIG, (MPU9250_FSR_2G << 3));
	delay_ms(1);
	//MPU9250 Set Accel DLPF
	data = MPU_Read_Byte(MPU9250_SPIx_ADDR, MPU9250_ACCEL_CONFIG2);
	data |= MPU9250_ACCEL_DLPF_41HZ;
	delay_ms(1);
	MPU_Write_Byte(MPU9250_SPIx_ADDR, MPU9250_ACCEL_CONFIG2, data);
	delay_ms(1);
	//MPU9250 Set Gyro DLPF
	MPU_Write_Byte(MPU9250_SPIx_ADDR, MPU9250_CONFIG, MPU9250_GYRO_DLPF_41HZ);
	delay_ms(1);
	//MPU9250 Set SPI Mode
	state = MPU_Read_Byte(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL);
	delay_ms(1);
	MPU_Write_Byte(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL, state | MPU9250_I2C_IF_DIS);
	delay_ms(1);
	state = MPU_Read_Byte(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL);
	delay_ms(1);
	MPU_Write_Byte(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL, state | MPU9250_I2C_MST_EN);
	delay_ms(1);
	//////////////////////////////////////////////////////////////////////////
	//AK8963 Setup
	//reset AK8963
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL2, MPU9250_AK8963_CNTL2_SRST);
	delay_ms(2);

	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	delay_ms(1);
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_FUSE_ROM_ACCESS);
	delay_ms(1);
	//
	//AK8963 get calibration data
	MPU9250_AK8963_SPIx_Reads(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_ASAX, 3, response);
	//AK8963_SENSITIVITY_SCALE_FACTOR
	//AK8963_ASA[i++] = (s16)((data - 128.0f) / 256.0f + 1.0f) ;
	//MPU9250_AK8963_ASA[0] = (s16)(response[0]) + 128;
	//MPU9250_AK8963_ASA[1] = (s16)(response[1]) + 128;
	//MPU9250_AK8963_ASA[2] = (s16)(response[2]) + 128;
	delay_ms(1);
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	delay_ms(1);
	//
	MPU_Write_Byte(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_CTRL, 0x5D);
	delay_ms(1);
	MPU_Write_Byte(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR | MPU9250_I2C_READ);
	delay_ms(1);
	MPU_Write_Byte(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_REG, MPU9250_AK8963_ST1);
	delay_ms(1);
	MPU_Write_Byte(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_CTRL, 0x88);
	delay_ms(1);
	//
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_CONTINUOUS_MEASUREMENT);
	delay_ms(1);

	//
	MPU_Write_Byte(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 0x09);
	delay_ms(1);
	//
	MPU_Write_Byte(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_DELAY_CTRL, 0x81);
	delay_ms(100);

    return 0;
}

//设置MPU9250陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU9250加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}

//设置MPU9250的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU9250_ADDR,MPU_CFG_REG,data);//设置数字低通滤波器  
}

//设置MPU9250的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU9250_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=21+((double)raw)/333.87;  
    return temp*100;;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res; 
	res=MPU_Read_Len(MPU9250_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}

//得到磁力计值(原始值)
//mx,my,mz:磁力计x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Magnetometer(short *mx,short *my,short *mz)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(AK8963_ADDR,MAG_XOUT_L,6,buf);
	if(res==0)
	{
		*mx=((u16)buf[1]<<8)|buf[0];  
		*my=((u16)buf[3]<<8)|buf[2];  
		*mz=((u16)buf[5]<<8)|buf[4];
	} 	
    MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11); //AK8963每次读完以后都需要重新设置为单次测量模式
    return res;;
}

//-------------------------------------------------------------------------以下为基本读写功能函数
//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	/*
    u8 i;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_Wait_Ack())          //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
    for(i=0;i<len;i++)
    {
        IIC_Send_Byte(buf[i]);  //发送数据
        if(IIC_Wait_Ack())      //等待ACK
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
	*/
    u32 i=0;
		char cnt=0;
		
		MPU9250_nCS=0;
	  SPI1_ReadWriteByte(reg);
		while(i<len)
		{
			SPI1_ReadWriteByte(buf[i++]);
		}
		while(cnt<15){cnt++;}//500ns
	  MPU9250_nCS=1;
    while(cnt>10){cnt--;}//200ns
		
    return 0;
} 

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	/*
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_Wait_Ack())          //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
	IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //发送器件地址+读命令
    IIC_Wait_Ack();             //等待应答
    while(len)
    {
        if(len==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++;  
    }
    IIC_Stop();                 //产生一个停止条件
		*/
		u32 i=0;
		char cnt=0;
		
		MPU9250_nCS=0;
	  SPI1_ReadWriteByte(0x80|reg);
		while(i<len)
		{
			buf[i++]=SPI1_ReadWriteByte(0xFF);
		}
		while(cnt<15){cnt++;}
	  MPU9250_nCS=1;
    while(cnt>10){cnt--;}
		
    return 0;
}

//IIC写一个字节 
//devaddr:器件IIC地址
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Byte(u8 addr,u8 reg,u8 data)
{
	/*
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_Wait_Ack())          //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
    IIC_Send_Byte(data);        //发送数据
    if(IIC_Wait_Ack())          //等待ACK
    {
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
		*/
	  char cnt=0;
	
		MPU9250_nCS=0;
	  SPI1_ReadWriteByte(reg);
	  SPI1_ReadWriteByte(data);
		while(cnt<15){cnt++;}
	  MPU9250_nCS=1;
    while(cnt>10){cnt--;}
		
    return 0;
}

//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 MPU_Read_Byte(u8 addr,u8 reg)
{
	u8 res;
	/*
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    IIC_Wait_Ack();             //等待应答
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
	  IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //发送器件地址+读命令
    IIC_Wait_Ack();             //等待应答
    res=IIC_Read_Byte(0);		//读数据,发送nACK  
    IIC_Stop();                 //产生一个停止条件
	*/
	  char cnt=0;
	
		MPU9250_nCS=0;
	  SPI1_ReadWriteByte(0x80|reg);
	  res=SPI1_ReadWriteByte(0xFF);
		while(cnt<15){cnt++;}
	  MPU9250_nCS=1;
    while(cnt>10){cnt--;}
		
    return res;  
}

int MPU9250_AK8963_SPIx_Read(u8 akm_addr, u8 reg_addr, u8* data) {
	u8 status = 0;
	u32 timeout = 0;

	MPU_Write_Len(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &reg_addr);
	reg_addr = akm_addr | MPU9250_I2C_READ;
	MPU_Write_Len(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &reg_addr);
	reg_addr = MPU9250_I2C_SLV4_EN;
	MPU_Write_Len(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &reg_addr);
	do {
		if (timeout++ > 1024)
			return -2;
		delay_ms(1);
		MPU_Read_Len(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
	} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
	MPU_Read_Len(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DI, 1, data);
	return 0;
}

int MPU9250_AK8963_SPIx_Reads(u8 akm_addr, u8 reg_addr, u8 len, u8* data){
	u8 index = 0;
	u8 status = 0;
	u32 timeout = 0;
	u8 tmp = 0;

	tmp = akm_addr | MPU9250_I2C_READ;
	MPU_Write_Len(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	while(index < len){
		tmp = reg_addr + index;
		MPU_Write_Len(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
		tmp = 0xFF;
		MPU_Write_Len(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DO, 1, &tmp);
		tmp = MPU9250_I2C_SLV4_EN;
		MPU_Write_Len(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
		do {
			if (timeout++ > 1024)
				return -2;
			delay_ms(1);
			MPU_Read_Len(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
		} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
		MPU_Read_Len(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DI, 1, data + index);
		index++;
	}
	return 0;
}

int MPU9250_AK8963_SPIx_Write(u8 akm_addr, u8 reg_addr, u8 data)
{
	u32 timeout = 0;
	uint8_t status = 0;
	u8 tmp = 0;

	tmp = akm_addr;
	MPU_Write_Len(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	tmp = reg_addr;
	MPU_Write_Len(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
	tmp = data;
	MPU_Write_Len(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DO, 1, &tmp);
	tmp = MPU9250_I2C_SLV4_EN;
	MPU_Write_Len(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
	do {
		if (timeout++ > 1024)
			return -2;
		delay_ms(1);
		MPU_Read_Len(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
	} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
	if (status & MPU9250_I2C_SLV4_NACK)
		return -3;
	return 0;
}

int MPU9250_AK8963_SPIx_Writes(u8 akm_addr, u8 reg_addr, u8 len, u8* data)
{
	u32 timeout = 0;
	uint8_t status = 0;
	u8 tmp = 0;
	u8 index = 0;

	tmp = akm_addr;
	MPU_Write_Len(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	while(index < len){
		tmp = reg_addr + index;
		MPU_Write_Len(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
		MPU_Write_Len(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DO, 1, data + index);
		tmp = MPU9250_I2C_SLV4_EN;
		MPU_Write_Len(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
		do {
			if (timeout++ > 1024)
				return -2;
			delay_ms(1);
			MPU_Read_Len(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
		} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
		if (status & MPU9250_I2C_SLV4_NACK)
			return -3;
		index++;
	}
	return 0;
}
