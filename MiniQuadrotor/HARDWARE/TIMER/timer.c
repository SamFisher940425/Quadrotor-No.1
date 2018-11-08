#include "timer.h"
#include "include.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//定时器 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/4
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

//-----------------------------------------------------------------------以下是定时中断的配置及中断函数
//通用定时器4中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!
void TIM4_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};
	NVIC_InitTypeDef NVIC_InitStructure={0};
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  ///使能TIM4时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//初始化TIM4
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //允许定时器4更新中断
	TIM_Cmd(TIM4,ENABLE); //使能定时器4
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; //定时器4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

//定时器4中断服务函数
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) //溢出中断
	{
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);  //清除中断标志位
		
		//GPIO_SetBits(GPIOE,GPIO_Pin_2);
		
		if(Unlock_Flag==1)
		{
			Unlock_Cnt++;
			if(Int_Cnt%10==0)
		  {
		  	LED0=!LED0;//DS1翻转
	  	}
		}
		else
		{
			if(Int_Cnt%50==0)
		  {
		  	LED0=!LED0;//DS1翻转
	  	}
		}
/*--------------------------------------------------------------------------------------------*/
		if(TIM1CH1_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
		{
			if(TIM1CH1_CAPTURE_VAL>1000&&TIM1CH1_CAPTURE_VAL<2000)
				TIM1CH1_Val=TIM1CH1_CAPTURE_VAL;		   //得到总的高电平时间
			else
				TIM1CH1_Val=TIM1CH1_Val;
			
			TIM1CH1_CAPTURE_STA=0;			     //开启下一次捕获
			TIM1CH1_CAPTURE_VAL=0;
			
			Yaw_Angle_Change_Want=(TIM1CH1_Val-1485)*2.0/67.0;
	  	if(Yaw_Angle_Change_Want>10.0)Yaw_Angle_Change_Want=10.0;
	  	if(Yaw_Angle_Change_Want>-0.1&&Yaw_Angle_Change_Want<0.1)Yaw_Angle_Change_Want=0.0;
	  	if(Yaw_Angle_Change_Want<-10.0)Yaw_Angle_Change_Want=-10.0;
		}
		
		if(TIM1CH2_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
		{
			if(TIM1CH2_CAPTURE_VAL>1000&&TIM1CH2_CAPTURE_VAL<2000)
				TIM1CH2_Val=TIM1CH2_CAPTURE_VAL;		   //得到总的高电平时间
			else
				TIM1CH2_Val=TIM1CH2_Val;
			
			TIM1CH2_CAPTURE_STA=0;			     //开启下一次捕获
			TIM1CH2_CAPTURE_VAL=0;
			
			Pitch_Angle_Want=-1*(TIM1CH2_Val-1485)*2.0/67.5;
		  if(Pitch_Angle_Want>10.0)Pitch_Angle_Want=10.0;
	  	if(Pitch_Angle_Want>-0.2&&Pitch_Angle_Want<0.2)Pitch_Angle_Want=0.0;
	  	if(Pitch_Angle_Want<-10.0)Pitch_Angle_Want=-10.0;
		}
		
		if(TIM1CH3_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
		{
			if(TIM1CH3_CAPTURE_VAL>1000&&TIM1CH3_CAPTURE_VAL<2000)
				TIM1CH3_Val=TIM1CH3_CAPTURE_VAL;		   //得到总的高电平时间
			else
				TIM1CH3_Val=TIM1CH3_Val;
			
			TIM1CH3_CAPTURE_STA=0;			     //开启下一次捕获
			TIM1CH3_CAPTURE_VAL=0;
			
			Roll_Angle_Want=-1*(TIM1CH3_Val-1485)*2.0/67.0;
  		if(Roll_Angle_Want>10.0)Roll_Angle_Want=10.0;
	  	if(Roll_Angle_Want>-0.1&&Roll_Angle_Want<0.1)Roll_Angle_Want=0.0;
	  	if(Roll_Angle_Want<-10.0)Roll_Angle_Want=-10.0;
		}
		
		if(TIM1CH4_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
		{
			if(TIM1CH4_CAPTURE_VAL>1000&&TIM1CH4_CAPTURE_VAL<2000)
				TIM1CH4_Val=TIM1CH4_CAPTURE_VAL;		   //得到总的高电平时间
			else
				TIM1CH4_Val=TIM1CH4_Val;
			
			TIM1CH4_CAPTURE_STA=0;			     //开启下一次捕获
			TIM1CH4_CAPTURE_VAL=0;
			
			Throttle=(TIM1CH4_Val-1150)*400.0/670.0;
	  	if(Throttle>400.0)Throttle=400.0;
	  	if(Throttle<0.0)Throttle=0.0;
		}
		
		if(TIM2CH1_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
		{
			if(TIM2CH1_CAPTURE_VAL>1000&&TIM2CH1_CAPTURE_VAL<2000)
				TIM2CH1_Val=TIM2CH1_CAPTURE_VAL;		   //得到总的高电平时间
			else
				TIM2CH1_Val=TIM2CH1_Val;
			
			TIM2CH1_CAPTURE_STA=0;			     //开启下一次捕获
			TIM2CH1_CAPTURE_VAL=0;
		}
		
/*--------------------------------------------------------------------------------------------*/
		if(Int_Cnt%2==0)
		{
			d2=MS5611_ADC_Read();
		  dT = d2 - MS5611_PROM[5]*256.0;
		  T = (2000+(dT*MS5611_PROM[6])/8388608.0)/100.0;//warning is allowed
			MS5611_P_ADC_Start();
		}
		else if(Int_Cnt%2==1)
		{
			d1=MS5611_ADC_Read();
			OFF = MS5611_PROM[2]*65536.0 + (dT*MS5611_PROM[4])/128.0;//warning is allowed
	  	SENS = MS5611_PROM[1]*32768.0 + (dT*MS5611_PROM[3])/256.0;//warning is allowed
			P[1]=P[0];
			P[0] = (((d1*SENS)/2097152.0 - OFF)/32768.0)/100.0;//warning is allowed
			MS5611_T_ADC_Start();
			
			P_Presure = P_Presure+0.0000010;
			Kg_Presure = P_Presure/(P_Presure+0.0025);
			P[0]=P[1]+Kg_Presure*(P[0]-P[1]);
			P_Presure=(1-Kg_Presure)*P_Presure;
		}
/*--------------------------------------------------------------------------------------------*/
		//mpu_mpl_get_data(&MPL_Pitch,&MPL_Roll,&MPL_Yaw);
		mpu_dmp_get_data(&DMP_Pitch,&DMP_Roll,&DMP_Yaw);
		
		if(Pitch[0]>360)
		{
			Pitch[0] = MPL_Pitch;
			Pitch[1]=Pitch[0];
		}
		else//
		{
			Pitch[1]=Pitch[0];
			/*
			Err_DMP_Pitch=fabsf(Pitch[0]-DMP_Pitch);
			Err_MPL_Pitch=fabsf(Pitch[0]-MPL_Pitch);

			Weight_DMP_Pitch=exp(-1*Err_DMP_Pitch);
			Weight_MPL_Pitch=exp(-1*Err_MPL_Pitch);
			if(Weight_MPL_Pitch>0.99)Weight_MPL_Pitch=0.99;
			if(Weight_DMP_Pitch<0.01)Weight_DMP_Pitch=0.01;
			Weight_Pitch_Sum=Weight_DMP_Pitch+Weight_MPL_Pitch;
			*/
			Pitch[0]=DMP_Pitch;//(Weight_DMP_Pitch*DMP_Pitch+Weight_MPL_Pitch*MPL_Pitch)/Weight_Pitch_Sum
			/*
			P_Pitch=P_Pitch+fabs(Pitch[1]-Pitch[0]);//卡尔曼滤波尝试
			Kg_Pitch=P_Pitch/(P_Pitch+fminf(Err_DMP_Pitch,Err_MPL_Pitch));
			Pitch[0]=Pitch[1]+Kg_Pitch*(Pitch[0]-Pitch[1]);
			P_Pitch=(1-Kg_Pitch)*P_Pitch;
			*/
		}
		if(Roll[0]>360)
		{
			Roll[0] = MPL_Roll;
			Roll[1]=Roll[0];
		}
		else
		{
			Roll[1]=Roll[0];
			/*
			Err_DMP_Roll=fabsf(Roll[0]-DMP_Roll);
			Err_MPL_Roll=fabsf(Roll[0]-MPL_Roll);

			Weight_DMP_Roll=exp(-1*Err_DMP_Roll);
			Weight_MPL_Roll=exp(-1*Err_MPL_Roll);
			if(Weight_MPL_Roll>0.99)Weight_MPL_Roll=0.99;
			if(Weight_DMP_Roll<0.01)Weight_DMP_Roll=0.01;
			Weight_Roll_Sum=Weight_DMP_Roll+Weight_MPL_Roll;
			*/
			Roll[0]=DMP_Roll;//(Weight_DMP_Roll*DMP_Roll+Weight_MPL_Roll*MPL_Roll)/Weight_Roll_Sum
			/*
			P_Roll=P_Roll+fabs(Roll[1]-Roll[0]);//卡尔曼滤波尝试
			Kg_Roll=P_Roll/(P_Roll+fminf(Err_DMP_Roll,Err_MPL_Roll));
			Roll[0]=Roll[1]+Kg_Roll*(Roll[0]-Roll[1]);
			P_Roll=(1-Kg_Roll)*P_Roll;
			*/
		}
		if(Yaw[0]>360)
		{
			Yaw[0] = MPL_Yaw;
			Yaw[1]=Yaw[0];
		}
		else
		{
			Yaw[1]=Yaw[0];
			/*
			Err_DMP_Yaw=fabsf(Yaw[0]-DMP_Yaw);
			Err_MPL_Yaw=fabsf(Yaw[0]-MPL_Yaw);

			Weight_DMP_Yaw=exp(-1*Err_DMP_Yaw);
			Weight_MPL_Yaw=exp(-1*Err_MPL_Yaw);
			if(Weight_MPL_Yaw>0.99)Weight_MPL_Yaw=0.99;
			if(Weight_DMP_Yaw<0.01)Weight_DMP_Yaw=0.01;
			Weight_Yaw_Sum=Weight_DMP_Yaw+Weight_MPL_Yaw;
			*/
			Yaw[0]=DMP_Yaw;//(Weight_DMP_Yaw*DMP_Yaw+Weight_MPL_Yaw*MPL_Yaw)/Weight_Yaw_Sum
			/*
			P_Yaw=P_Yaw+fabs(Yaw[1]-Yaw[0]);//卡尔曼滤波尝试
			Kg_Yaw=P_Yaw/(P_Yaw+fminf(Err_DMP_Yaw,Err_MPL_Yaw));
			Yaw[0]=Yaw[1]+Kg_Yaw*(Yaw[0]-Yaw[1]);
			P_Yaw=(1-Kg_Yaw)*P_Yaw;
			*/
		}
/*--------------------------------------------------------------------------------------------*/
		if(Control_Flag==1)
		{
			if(Int_Cnt%20==5)
	  	{
	  		Yaw_Angle_Want+=Yaw_Angle_Change_Want;
	  		if(Yaw_Angle_Want>180.0)Yaw_Angle_Want=180.0;
	  		if(Yaw_Angle_Want<-180.0)Yaw_Angle_Want=-180.0;
  		}
			Pitch_PID();
	  	Roll_PID();
	  	Yaw_PID();
		  Motor_Out();
		}
		else
		{
			Yaw_Angle_Want=0.00000;
			Yaw_Bias=Yaw[0];
			Pitch_PID_Clear();
			Roll_PID_Clear();
			Yaw_PID_Clear();
			Motor_Out_1=Motor_Out_2=Motor_Out_3=Motor_Out_4=0;
		}
		
		TIM_SetCompare1(TIM3,Motor_Out_3);//Throttle*0.2  M3R
    TIM_SetCompare2(TIM3,Motor_Out_4);//Throttle*0.2  M4L
    TIM_SetCompare3(TIM3,Motor_Out_1);//Throttle*0.2  M1L
    TIM_SetCompare4(TIM3,Motor_Out_2);//Throttle*0.2  M2R
		
		Int_Cnt++;
		if(Int_Cnt>=500)Int_Cnt=0;
		
		//GPIO_ResetBits(GPIOE,GPIO_Pin_2);
		
	}
	
}

//------------------------------------------------------------------以下是输入捕捉的配置以及中断函数
TIM_ICInitTypeDef  TIM2_ICInitStructure={0};
//定时器2通道1输入捕获配置
//arr：自动重装值(TIM5,TIM2是32位的!!)
//psc：时钟预分频数
void TIM2_CH1_Cap_Init(u32 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure={0};
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure={0};
	NVIC_InitTypeDef NVIC_InitStructure={0};

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	//TIM2时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能PORTA时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource15,GPIO_AF_TIM2); //复用位定时器2
  
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	

	//初始化TIM2输入捕获参数
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
		
	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC1,ENABLE);//允许更新中断 ,允许CC1IE捕获中断
	
  TIM_Cmd(TIM2,ENABLE ); 	//使能定时器2

 
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x00;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	
}

//捕获状态
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到上升沿;1,已经捕获到上升沿了.
//[5:0]:捕获低电平后溢出的次数(对于32位定时器来说,1us计数器加1,溢出时间:4294秒)
u8  TIM2CH1_CAPTURE_STA=0;	//输入捕获状态		    				
u32	TIM2CH1_CAPTURE_VAL=0;	//输入捕获值(TIM5/TIM2是32位)
//定时器2中断服务程序	 
void TIM2_IRQHandler(void)
{
	if((TIM2CH1_CAPTURE_STA&0X80)==1)//如果已经完成了捕捉
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1); //清除捕捉中断标志位，溢出标志待所有通道运算完成一起清除
	else//如果没有完成捕捉
	{
		if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)//是溢出导致中断
		{
			if(TIM2CH1_CAPTURE_STA&0X40)//曾经捕获到高电平了
			{
				if((TIM2CH1_CAPTURE_STA&0X3F)==0X3F)//高电平超过存储上限
				{
					TIM2CH1_CAPTURE_STA|=0X80;		//强制标记成功捕获了一次
					TIM2CH1_CAPTURE_VAL=0XFFFFFFFF;
				}
				else
					TIM2CH1_CAPTURE_STA++;//溢出计数增加
			}
			//清标志位放到最后，各通道运算完毕一起清除
		}
		
		if(TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)//捕获导致中断
		{
			if(TIM2CH1_CAPTURE_STA&0X40)		//曾经捕捉过上跳沿 此次为下跳沿 		
			{
				TIM2CH1_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
			  TIM2CH1_CAPTURE_VAL=TIM_GetCapture1(TIM2)+((TIM2CH1_CAPTURE_STA&0x3F)*0XFFFFFFFF)-TIM2CH1_CAPTURE_VAL;//计算脉宽.TIM2/5是32位
	 			TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}
			else  								//还未开始,第一次捕获上升沿
			{
				TIM2CH1_CAPTURE_STA=0;			//清空
				TIM2CH1_CAPTURE_VAL=0;
				TIM2CH1_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				TIM2CH1_CAPTURE_VAL=TIM_GetCapture1(TIM2);
	 			TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			}
			TIM_ClearITPendingBit(TIM2, TIM_IT_CC1); //清除捕获中断标志位
		}
	}
	
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); //清除溢出中断标志位
}

TIM_ICInitTypeDef  TIM1_ICInitStructure={0};

void TIM1_CH1_Cap_Init(u32 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure={0};
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure={0};
	NVIC_InitTypeDef NVIC_InitStructure={0};

	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//使能PORTE时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_Init(GPIOE,&GPIO_InitStructure); //初始化

	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); //复用位定时器1
  
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	

	//初始化TIM1输入捕获参数
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM1_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM1, &TIM1_ICInitStructure);
		
	TIM_ITConfig(TIM1,TIM_IT_Update|TIM_IT_CC1,ENABLE);//允许更新中断 ,允许CC1IE捕获中断
	
  TIM_Cmd(TIM1,ENABLE ); 	//使能定时器1

 
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x00;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	
}

void TIM1_CH2_Cap_Init(u32 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure={0};
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure={0};
	NVIC_InitTypeDef NVIC_InitStructure={0};

	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//使能PORTE时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_Init(GPIOE,&GPIO_InitStructure); //初始化

	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1); //复用位定时器1
  
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	

	//初始化TIM1输入捕获参数
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC2S=02 	选择输入端 IC2映射到TI2上
  TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI2上
  TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM1_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM1, &TIM1_ICInitStructure);
		
	TIM_ITConfig(TIM1,TIM_IT_Update|TIM_IT_CC2,ENABLE);//允许更新中断 ,允许CC2IE捕获中断
	
  TIM_Cmd(TIM1,ENABLE ); 	//使能定时器1

  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x00;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	
}

void TIM1_CH3_Cap_Init(u32 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure={0};
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure={0};
	NVIC_InitTypeDef NVIC_InitStructure={0};

	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//使能PORTE时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_Init(GPIOE,&GPIO_InitStructure); //初始化

	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1); //复用位定时器1
  
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	

	//初始化TIM1输入捕获参数
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC3S=03 	选择输入端 IC3映射到TI3上
  TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI3上
  TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM1_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM1, &TIM1_ICInitStructure);
		
	TIM_ITConfig(TIM1,TIM_IT_Update|TIM_IT_CC3,ENABLE);//允许更新中断 ,允许CC3IE捕获中断
	
  TIM_Cmd(TIM1,ENABLE ); 	//使能定时器1

  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x00;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	
}

void TIM1_CH4_Cap_Init(u32 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure={0};
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure={0};
	NVIC_InitTypeDef NVIC_InitStructure={0};

	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//使能PORTE时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_Init(GPIOE,&GPIO_InitStructure); //初始化

	GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1); //复用位定时器1
  
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	

	//初始化TIM1输入捕获参数
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC4S=04 	选择输入端 IC4映射到TI4上
  TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI4上
  TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM1_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM1, &TIM1_ICInitStructure);
		
	TIM_ITConfig(TIM1,TIM_IT_Update|TIM_IT_CC4,ENABLE);//允许更新中断 ,允许CC4IE捕获中断
	
  TIM_Cmd(TIM1,ENABLE ); 	//使能定时器1

  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x00;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	
}

//捕获状态
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到上升沿;1,已经捕获到上升沿了.
//[5:0]:捕获低电平后溢出的次数(对于32位定时器来说,1us计数器加1,溢出时间:4294秒)
u8  TIM1CH1_CAPTURE_STA=0;	//输入捕获状态		    				
u32	TIM1CH1_CAPTURE_VAL=0;	//输入捕获值(TIM5/TIM2是32位)

u8  TIM1CH2_CAPTURE_STA=0;	//输入捕获状态		    				
u32	TIM1CH2_CAPTURE_VAL=0;	//输入捕获值(TIM5/TIM2是32位)

u8  TIM1CH3_CAPTURE_STA=0;	//输入捕获状态		    				
u32	TIM1CH3_CAPTURE_VAL=0;	//输入捕获值(TIM5/TIM2是32位)

u8  TIM1CH4_CAPTURE_STA=0;	//输入捕获状态		    				
u32	TIM1CH4_CAPTURE_VAL=0;	//输入捕获值(TIM5/TIM2是32位)
//定时器2中断服务程序	 
void TIM1_CC_IRQHandler(void)
{
	if((TIM1CH1_CAPTURE_STA&0X80)==1)//如果已经完成了捕捉
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC1); //清除捕捉中断标志位，溢出标志待所有通道运算完成一起清除
	else//如果没有完成捕捉
	{
		if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)//是溢出导致中断
		{
			if(TIM1CH1_CAPTURE_STA&0X40)//曾经捕获到高电平了
			{
				if((TIM1CH1_CAPTURE_STA&0X3F)==0X3F)//高电平超过存储上限
				{
					TIM1CH1_CAPTURE_STA|=0X80;		//强制标记成功捕获了一次
					TIM1CH1_CAPTURE_VAL=0XFFFF;
				}
				else
					TIM1CH1_CAPTURE_STA++;//溢出计数增加
			}
			//清标志位放到最后，各通道运算完毕一起清除
		}
		
		if(TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)//捕获导致中断
		{
			if(TIM1CH1_CAPTURE_STA&0X40)		//曾经捕捉过上跳沿 此次为下跳沿 		
			{
				TIM1CH1_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
			  TIM1CH1_CAPTURE_VAL=TIM_GetCapture1(TIM1)+((TIM1CH1_CAPTURE_STA&0x3F)*0XFFFF)-TIM1CH1_CAPTURE_VAL;//计算脉宽.TIM2/5是32位
	 			TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}
			else  								//还未开始,第一次捕获上升沿
			{
				TIM1CH1_CAPTURE_STA=0;			//清空
				TIM1CH1_CAPTURE_VAL=0;
				TIM1CH1_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				TIM1CH1_CAPTURE_VAL=TIM_GetCapture1(TIM1);
	 			TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			}
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC1); //清除捕获中断标志位
		}
	}
	
	if((TIM1CH2_CAPTURE_STA&0X80)==1)//如果已经完成了捕捉
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC2); //清除捕捉中断标志位，溢出标志待所有通道运算完成一起清除
	else//如果没有完成捕捉
	{
		if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)//是溢出导致中断
		{
			if(TIM1CH2_CAPTURE_STA&0X40)//曾经捕获到高电平了
			{
				if((TIM1CH2_CAPTURE_STA&0X3F)==0X3F)//高电平超过存储上限
				{
					TIM1CH2_CAPTURE_STA|=0X80;		//强制标记成功捕获了一次
					TIM1CH2_CAPTURE_VAL=0XFFFF;
				}
				else
					TIM1CH2_CAPTURE_STA++;//溢出计数增加
			}
			//清标志位放到最后，各通道运算完毕一起清除
		}
		
		if(TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET)//捕获导致中断
		{
			if(TIM1CH2_CAPTURE_STA&0X40)		//曾经捕捉过上跳沿 此次为下跳沿 		
			{
				TIM1CH2_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
			  TIM1CH2_CAPTURE_VAL=TIM_GetCapture2(TIM1)+(TIM1CH2_CAPTURE_STA&0x3F)*0XFFFF-TIM1CH2_CAPTURE_VAL;//计算脉宽.TIM2/5是32位
	 			TIM_OC2PolarityConfig(TIM1,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}
			else  								//还未开始,第一次捕获上升沿
			{
				TIM1CH2_CAPTURE_STA=0;			//清空
				TIM1CH2_CAPTURE_VAL=0;
				TIM1CH2_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				TIM1CH2_CAPTURE_VAL=TIM_GetCapture2(TIM1);
	 			TIM_OC2PolarityConfig(TIM1,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			}
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC2); //清除捕获中断标志位
		}
	}
	
	if((TIM1CH3_CAPTURE_STA&0X80)==1)//如果已经完成了捕捉
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC3); //清除捕捉中断标志位，溢出标志待所有通道运算完成一起清除
	else//如果没有完成捕捉
	{
		if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)//是溢出导致中断
		{
			if(TIM1CH3_CAPTURE_STA&0X40)//曾经捕获到高电平了
			{
				if((TIM1CH3_CAPTURE_STA&0X3F)==0X3F)//高电平超过存储上限
				{
					TIM1CH3_CAPTURE_STA|=0X80;		//强制标记成功捕获了一次
					TIM1CH3_CAPTURE_VAL=0XFFFF;
				}
				else
					TIM1CH3_CAPTURE_STA++;//溢出计数增加
			}
			//清标志位放到最后，各通道运算完毕一起清除
		}
		
		if(TIM_GetITStatus(TIM1, TIM_IT_CC3) != RESET)//捕获导致中断
		{
			if(TIM1CH3_CAPTURE_STA&0X40)		//曾经捕捉过上跳沿 此次为下跳沿 		
			{
				TIM1CH3_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
			  TIM1CH3_CAPTURE_VAL=TIM_GetCapture3(TIM1)+(TIM1CH3_CAPTURE_STA&0x3F)*0XFFFF-TIM1CH3_CAPTURE_VAL;//计算脉宽.TIM2/5是32位
	 			TIM_OC3PolarityConfig(TIM1,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}
			else  								//还未开始,第一次捕获上升沿
			{
				TIM1CH3_CAPTURE_STA=0;			//清空
				TIM1CH3_CAPTURE_VAL=0;
				TIM1CH3_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				TIM1CH3_CAPTURE_VAL=TIM_GetCapture3(TIM1);
	 			TIM_OC3PolarityConfig(TIM1,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			}
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC3); //清除捕获中断标志位
		}
	}
	
	if((TIM1CH4_CAPTURE_STA&0X80)==1)//如果已经完成了捕捉
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC4); //清除捕捉中断标志位，溢出标志待所有通道运算完成一起清除
	else//如果没有完成捕捉
	{
		if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)//是溢出导致中断
		{
			if(TIM1CH4_CAPTURE_STA&0X40)//曾经捕获到高电平了
			{
				if((TIM1CH4_CAPTURE_STA&0X3F)==0X3F)//高电平超过存储上限
				{
					TIM1CH4_CAPTURE_STA|=0X80;		//强制标记成功捕获了一次
					TIM1CH4_CAPTURE_VAL=0XFFFF;
				}
				else
					TIM1CH4_CAPTURE_STA++;//溢出计数增加
			}
			//清标志位放到最后，各通道运算完毕一起清除
		}
		
		if(TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET)//捕获导致中断
		{
			if(TIM1CH4_CAPTURE_STA&0X40)		//曾经捕捉过上跳沿 此次为下跳沿 		
			{
				TIM1CH4_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
			  TIM1CH4_CAPTURE_VAL=TIM_GetCapture4(TIM1)+(TIM1CH4_CAPTURE_STA&0x3F)*0XFFFF-TIM1CH4_CAPTURE_VAL;//计算脉宽.TIM2/5是32位
	 			TIM_OC4PolarityConfig(TIM1,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}
			else  								//还未开始,第一次捕获上升沿
			{
				TIM1CH4_CAPTURE_STA=0;			//清空
				TIM1CH4_CAPTURE_VAL=0;
				TIM1CH4_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				TIM1CH4_CAPTURE_VAL=TIM_GetCapture4(TIM1);
	 			TIM_OC4PolarityConfig(TIM1,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			}
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC4); //清除捕获中断标志位
		}
	}
	
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update); //清除溢出中断标志位
}
