#include "include.h"

//ALIENTEK 探索者STM32F407开发板 实验0
//STM32F4工程模板-库函数版本
//技术支持：www.openedv.com
//淘宝店铺：http://eboard.taobao.com
//广州市星翼电子科技有限公司
//作者：正点原子 @ALIENTEK

unsigned long Int_Cnt=0;//若飞行时间长 需要清零

volatile long TIM1CH1_Val=1485;//偏航 左正 1155 1825 1485
volatile long TIM1CH2_Val=1485;//俯仰 上正 1150 1825 1485
volatile long TIM1CH3_Val=1485;//横滚 左正 1150 1820 1485
volatile long TIM1CH4_Val=1150;//油门 上正 1150 1820 1485
volatile long TIM2CH1_Val=1496;//模拟 右正 1155 1830 1496

volatile double Throttle=0.0,Pitch_Angle_Want=0.0,Roll_Angle_Want=0.0,Yaw_Angle_Want=0.0,Yaw_Angle_Change_Want=0.0;//0~400

volatile int Motor_Out_1=0,Motor_Out_2=0,Motor_Out_3=0,Motor_Out_4=0;

float Pitch_Bias=0.00000,Roll_Bias=0.00000,Yaw_Bias=0.00000;

volatile float Pitch[2]={365.0},Roll[2]={365.0},Yaw[2]={365.0};
float DMP_Pitch=0.0,DMP_Roll=0.0,DMP_Yaw=0.0;
float MPL_Pitch=0.0,MPL_Roll=0.0,MPL_Yaw=0.0;

float Err_DMP_Pitch=0.0,Err_DMP_Roll=0.0,Err_DMP_Yaw=0.0;
float Err_MPL_Pitch=0.0,Err_MPL_Roll=0.0,Err_MPL_Yaw=0.0;

double P_Pitch=0.025,Kg_Pitch=0.0;
double P_Roll=0.025,Kg_Roll=0.0;
double P_Yaw=0.025,Kg_Yaw=0.0;

double Weight_DMP_Pitch=0.0,Weight_MPL_Pitch=0.0,Weight_Pitch_Sum=0.0;
double Weight_DMP_Roll=0.0,Weight_MPL_Roll=0.0,Weight_Roll_Sum=0.0;
double Weight_DMP_Yaw=0.0,Weight_MPL_Yaw=0.0,Weight_Yaw_Sum=0.0;

unsigned int d1=0,d2=0;
volatile double P[2]={990.0},T=0.0;
double dT=0.0,OFF=0.0,SENS=0.0;
double P_Presure=0.0025,Kg_Presure=0.000;

volatile short Gyo_Raw[3][2]={0},Acc_Raw[3][2]={0},Mag_Raw[3][2]={0};

volatile unsigned char Control_Flag=0,Unlock_Cnt=0,Unlock_Flag=0;

unsigned char CC1101_RxBuffer[ 32 ] = { 0 }; 
unsigned int CC1101_Cnt=0;
const char *g_Ashining = "abcdefgh\n";

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);    //初始化延时函数
	uart_init(115200);	//初始化串口波特率为115200
	
	delay_ms(1000);
	LED_Init();
	MPU9250_MS5611_CC1101_CS_GPIO_Init();
//	SPI1_Init();
	LED0=1;
//	while(mpu_dmp_init());
//	MS5611_Init();
	SPI2_Init();
	
//	TIM3_PWM_Init(400-1,42-1);//pwm 10khz
//	TIM1_CH1_Cap_Init(0XFFFF,168-1);
//	TIM1_CH2_Cap_Init(0XFFFF,168-1);
//	TIM1_CH3_Cap_Init(0XFFFF,168-1);
//	TIM1_CH4_Cap_Init(0XFFFF,168-1);//Throttle 1138~1822
//	TIM2_CH1_Cap_Init(0XFFFFFFFF,84-1);
//	TIM4_Int_Init(100-1,8400-1);//pit 10ms
//	Adc_Init();
	
	CC1101_Init( );
	
  while(1)
		{
//			if(Unlock_Flag<2)
//			{
//				if(TIM1CH4_Val<1200&&TIM1CH1_Val<1200&&TIM1CH2_Val<1200&&TIM1CH3_Val>1800)//
//				{
//					Unlock_Flag=1;
//					if(Unlock_Cnt>200)
//					{
//						Unlock_Flag=2;
//						Unlock_Cnt=0;
//					}
//				}
//				else
//				{
//					Unlock_Flag=0;
//					Unlock_Cnt=0;
//				}
//				
//				Control_Flag=0;
//			}
//			else
//			{
//				if(Throttle>4)
//				{
//					Control_Flag=1;
//				}
//				else if(Throttle<2)
//				{
//					Control_Flag=0;
//				}
//				if(Roll[0]>45||Roll[0]<-45||Pitch[0]>45||Pitch[0]<-45)
//				{
//					Control_Flag=0;
//					Unlock_Flag=0;
//				}
//			}
			
//			CC1101_Cnt=0;
//			CC1101_Set_Mode( RX_MODE );
//			CC1101_Cnt = CC1101_Rx_Packet( CC1101_RxBuffer );		//接收字节

//			if( 0 != CC1101_Cnt )
//				LED0=!LED0;
//			delay_ms(1);
			
			CC1101_Tx_Packet( (uint8_t *)g_Ashining, 9 , ADDRESS_CHECK );		//模式1发送固定字符,1S一包
			delay_ms(1000);
			LED0=!LED0;

		}
}

