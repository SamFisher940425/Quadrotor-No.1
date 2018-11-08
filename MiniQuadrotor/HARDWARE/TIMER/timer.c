#include "timer.h"
#include "include.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//��ʱ�� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/4
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

//-----------------------------------------------------------------------�����Ƕ�ʱ�жϵ����ü��жϺ���
//ͨ�ö�ʱ��4�жϳ�ʼ��
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��3!
void TIM4_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};
	NVIC_InitTypeDef NVIC_InitStructure={0};
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  ///ʹ��TIM4ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//��ʼ��TIM4
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //����ʱ��4�����ж�
	TIM_Cmd(TIM4,ENABLE); //ʹ�ܶ�ʱ��4
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; //��ʱ��4�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

//��ʱ��4�жϷ�����
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) //����ж�
	{
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);  //����жϱ�־λ
		
		//GPIO_SetBits(GPIOE,GPIO_Pin_2);
		
		if(Unlock_Flag==1)
		{
			Unlock_Cnt++;
			if(Int_Cnt%10==0)
		  {
		  	LED0=!LED0;//DS1��ת
	  	}
		}
		else
		{
			if(Int_Cnt%50==0)
		  {
		  	LED0=!LED0;//DS1��ת
	  	}
		}
/*--------------------------------------------------------------------------------------------*/
		if(TIM1CH1_CAPTURE_STA&0X80)        //�ɹ�������һ�θߵ�ƽ
		{
			if(TIM1CH1_CAPTURE_VAL>1000&&TIM1CH1_CAPTURE_VAL<2000)
				TIM1CH1_Val=TIM1CH1_CAPTURE_VAL;		   //�õ��ܵĸߵ�ƽʱ��
			else
				TIM1CH1_Val=TIM1CH1_Val;
			
			TIM1CH1_CAPTURE_STA=0;			     //������һ�β���
			TIM1CH1_CAPTURE_VAL=0;
			
			Yaw_Angle_Change_Want=(TIM1CH1_Val-1485)*2.0/67.0;
	  	if(Yaw_Angle_Change_Want>10.0)Yaw_Angle_Change_Want=10.0;
	  	if(Yaw_Angle_Change_Want>-0.1&&Yaw_Angle_Change_Want<0.1)Yaw_Angle_Change_Want=0.0;
	  	if(Yaw_Angle_Change_Want<-10.0)Yaw_Angle_Change_Want=-10.0;
		}
		
		if(TIM1CH2_CAPTURE_STA&0X80)        //�ɹ�������һ�θߵ�ƽ
		{
			if(TIM1CH2_CAPTURE_VAL>1000&&TIM1CH2_CAPTURE_VAL<2000)
				TIM1CH2_Val=TIM1CH2_CAPTURE_VAL;		   //�õ��ܵĸߵ�ƽʱ��
			else
				TIM1CH2_Val=TIM1CH2_Val;
			
			TIM1CH2_CAPTURE_STA=0;			     //������һ�β���
			TIM1CH2_CAPTURE_VAL=0;
			
			Pitch_Angle_Want=-1*(TIM1CH2_Val-1485)*2.0/67.5;
		  if(Pitch_Angle_Want>10.0)Pitch_Angle_Want=10.0;
	  	if(Pitch_Angle_Want>-0.2&&Pitch_Angle_Want<0.2)Pitch_Angle_Want=0.0;
	  	if(Pitch_Angle_Want<-10.0)Pitch_Angle_Want=-10.0;
		}
		
		if(TIM1CH3_CAPTURE_STA&0X80)        //�ɹ�������һ�θߵ�ƽ
		{
			if(TIM1CH3_CAPTURE_VAL>1000&&TIM1CH3_CAPTURE_VAL<2000)
				TIM1CH3_Val=TIM1CH3_CAPTURE_VAL;		   //�õ��ܵĸߵ�ƽʱ��
			else
				TIM1CH3_Val=TIM1CH3_Val;
			
			TIM1CH3_CAPTURE_STA=0;			     //������һ�β���
			TIM1CH3_CAPTURE_VAL=0;
			
			Roll_Angle_Want=-1*(TIM1CH3_Val-1485)*2.0/67.0;
  		if(Roll_Angle_Want>10.0)Roll_Angle_Want=10.0;
	  	if(Roll_Angle_Want>-0.1&&Roll_Angle_Want<0.1)Roll_Angle_Want=0.0;
	  	if(Roll_Angle_Want<-10.0)Roll_Angle_Want=-10.0;
		}
		
		if(TIM1CH4_CAPTURE_STA&0X80)        //�ɹ�������һ�θߵ�ƽ
		{
			if(TIM1CH4_CAPTURE_VAL>1000&&TIM1CH4_CAPTURE_VAL<2000)
				TIM1CH4_Val=TIM1CH4_CAPTURE_VAL;		   //�õ��ܵĸߵ�ƽʱ��
			else
				TIM1CH4_Val=TIM1CH4_Val;
			
			TIM1CH4_CAPTURE_STA=0;			     //������һ�β���
			TIM1CH4_CAPTURE_VAL=0;
			
			Throttle=(TIM1CH4_Val-1150)*400.0/670.0;
	  	if(Throttle>400.0)Throttle=400.0;
	  	if(Throttle<0.0)Throttle=0.0;
		}
		
		if(TIM2CH1_CAPTURE_STA&0X80)        //�ɹ�������һ�θߵ�ƽ
		{
			if(TIM2CH1_CAPTURE_VAL>1000&&TIM2CH1_CAPTURE_VAL<2000)
				TIM2CH1_Val=TIM2CH1_CAPTURE_VAL;		   //�õ��ܵĸߵ�ƽʱ��
			else
				TIM2CH1_Val=TIM2CH1_Val;
			
			TIM2CH1_CAPTURE_STA=0;			     //������һ�β���
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
			P_Pitch=P_Pitch+fabs(Pitch[1]-Pitch[0]);//�������˲�����
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
			P_Roll=P_Roll+fabs(Roll[1]-Roll[0]);//�������˲�����
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
			P_Yaw=P_Yaw+fabs(Yaw[1]-Yaw[0]);//�������˲�����
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

//------------------------------------------------------------------���������벶׽�������Լ��жϺ���
TIM_ICInitTypeDef  TIM2_ICInitStructure={0};
//��ʱ��2ͨ��1���벶������
//arr���Զ���װֵ(TIM5,TIM2��32λ��!!)
//psc��ʱ��Ԥ��Ƶ��
void TIM2_CH1_Cap_Init(u32 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure={0};
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure={0};
	NVIC_InitTypeDef NVIC_InitStructure={0};

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	//TIM2ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//ʹ��PORTAʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource15,GPIO_AF_TIM2); //����λ��ʱ��2
  
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	

	//��ʼ��TIM2���벶�����
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
		
	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC1,ENABLE);//��������ж� ,����CC1IE�����ж�
	
  TIM_Cmd(TIM2,ENABLE ); 	//ʹ�ܶ�ʱ��2

 
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x00;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
	
}

//����״̬
//[7]:0,û�гɹ��Ĳ���;1,�ɹ�����һ��.
//[6]:0,��û����������;1,�Ѿ�������������.
//[5:0]:����͵�ƽ������Ĵ���(����32λ��ʱ����˵,1us��������1,���ʱ��:4294��)
u8  TIM2CH1_CAPTURE_STA=0;	//���벶��״̬		    				
u32	TIM2CH1_CAPTURE_VAL=0;	//���벶��ֵ(TIM5/TIM2��32λ)
//��ʱ��2�жϷ������	 
void TIM2_IRQHandler(void)
{
	if((TIM2CH1_CAPTURE_STA&0X80)==1)//����Ѿ�����˲�׽
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1); //�����׽�жϱ�־λ�������־������ͨ���������һ�����
	else//���û����ɲ�׽
	{
		if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)//����������ж�
		{
			if(TIM2CH1_CAPTURE_STA&0X40)//�������񵽸ߵ�ƽ��
			{
				if((TIM2CH1_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ�����洢����
				{
					TIM2CH1_CAPTURE_STA|=0X80;		//ǿ�Ʊ�ǳɹ�������һ��
					TIM2CH1_CAPTURE_VAL=0XFFFFFFFF;
				}
				else
					TIM2CH1_CAPTURE_STA++;//�����������
			}
			//���־λ�ŵ���󣬸�ͨ���������һ�����
		}
		
		if(TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)//�������ж�
		{
			if(TIM2CH1_CAPTURE_STA&0X40)		//������׽�������� �˴�Ϊ������ 		
			{
				TIM2CH1_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
			  TIM2CH1_CAPTURE_VAL=TIM_GetCapture1(TIM2)+((TIM2CH1_CAPTURE_STA&0x3F)*0XFFFFFFFF)-TIM2CH1_CAPTURE_VAL;//��������.TIM2/5��32λ
	 			TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
			}
			else  								//��δ��ʼ,��һ�β���������
			{
				TIM2CH1_CAPTURE_STA=0;			//���
				TIM2CH1_CAPTURE_VAL=0;
				TIM2CH1_CAPTURE_STA|=0X40;		//��ǲ�����������
				TIM2CH1_CAPTURE_VAL=TIM_GetCapture1(TIM2);
	 			TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
			}
			TIM_ClearITPendingBit(TIM2, TIM_IT_CC1); //��������жϱ�־λ
		}
	}
	
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); //�������жϱ�־λ
}

TIM_ICInitTypeDef  TIM1_ICInitStructure={0};

void TIM1_CH1_Cap_Init(u32 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure={0};
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure={0};
	NVIC_InitTypeDef NVIC_InitStructure={0};

	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//ʹ��PORTEʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
	GPIO_Init(GPIOE,&GPIO_InitStructure); //��ʼ��

	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); //����λ��ʱ��1
  
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	

	//��ʼ��TIM1���벶�����
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM1_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM1, &TIM1_ICInitStructure);
		
	TIM_ITConfig(TIM1,TIM_IT_Update|TIM_IT_CC1,ENABLE);//��������ж� ,����CC1IE�����ж�
	
  TIM_Cmd(TIM1,ENABLE ); 	//ʹ�ܶ�ʱ��1

 
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x00;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
	
}

void TIM1_CH2_Cap_Init(u32 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure={0};
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure={0};
	NVIC_InitTypeDef NVIC_InitStructure={0};

	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//ʹ��PORTEʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
	GPIO_Init(GPIOE,&GPIO_InitStructure); //��ʼ��

	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1); //����λ��ʱ��1
  
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	

	//��ʼ��TIM1���벶�����
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC2S=02 	ѡ������� IC2ӳ�䵽TI2��
  TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI2��
  TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM1_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM1, &TIM1_ICInitStructure);
		
	TIM_ITConfig(TIM1,TIM_IT_Update|TIM_IT_CC2,ENABLE);//��������ж� ,����CC2IE�����ж�
	
  TIM_Cmd(TIM1,ENABLE ); 	//ʹ�ܶ�ʱ��1

  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x00;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
	
}

void TIM1_CH3_Cap_Init(u32 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure={0};
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure={0};
	NVIC_InitTypeDef NVIC_InitStructure={0};

	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//ʹ��PORTEʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
	GPIO_Init(GPIOE,&GPIO_InitStructure); //��ʼ��

	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1); //����λ��ʱ��1
  
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	

	//��ʼ��TIM1���벶�����
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC3S=03 	ѡ������� IC3ӳ�䵽TI3��
  TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI3��
  TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM1_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM1, &TIM1_ICInitStructure);
		
	TIM_ITConfig(TIM1,TIM_IT_Update|TIM_IT_CC3,ENABLE);//��������ж� ,����CC3IE�����ж�
	
  TIM_Cmd(TIM1,ENABLE ); 	//ʹ�ܶ�ʱ��1

  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x00;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
	
}

void TIM1_CH4_Cap_Init(u32 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure={0};
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure={0};
	NVIC_InitTypeDef NVIC_InitStructure={0};

	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//ʹ��PORTEʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
	GPIO_Init(GPIOE,&GPIO_InitStructure); //��ʼ��

	GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1); //����λ��ʱ��1
  
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	

	//��ʼ��TIM1���벶�����
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC4S=04 	ѡ������� IC4ӳ�䵽TI4��
  TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI4��
  TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM1_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM1, &TIM1_ICInitStructure);
		
	TIM_ITConfig(TIM1,TIM_IT_Update|TIM_IT_CC4,ENABLE);//��������ж� ,����CC4IE�����ж�
	
  TIM_Cmd(TIM1,ENABLE ); 	//ʹ�ܶ�ʱ��1

  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x00;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
	
}

//����״̬
//[7]:0,û�гɹ��Ĳ���;1,�ɹ�����һ��.
//[6]:0,��û����������;1,�Ѿ�������������.
//[5:0]:����͵�ƽ������Ĵ���(����32λ��ʱ����˵,1us��������1,���ʱ��:4294��)
u8  TIM1CH1_CAPTURE_STA=0;	//���벶��״̬		    				
u32	TIM1CH1_CAPTURE_VAL=0;	//���벶��ֵ(TIM5/TIM2��32λ)

u8  TIM1CH2_CAPTURE_STA=0;	//���벶��״̬		    				
u32	TIM1CH2_CAPTURE_VAL=0;	//���벶��ֵ(TIM5/TIM2��32λ)

u8  TIM1CH3_CAPTURE_STA=0;	//���벶��״̬		    				
u32	TIM1CH3_CAPTURE_VAL=0;	//���벶��ֵ(TIM5/TIM2��32λ)

u8  TIM1CH4_CAPTURE_STA=0;	//���벶��״̬		    				
u32	TIM1CH4_CAPTURE_VAL=0;	//���벶��ֵ(TIM5/TIM2��32λ)
//��ʱ��2�жϷ������	 
void TIM1_CC_IRQHandler(void)
{
	if((TIM1CH1_CAPTURE_STA&0X80)==1)//����Ѿ�����˲�׽
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC1); //�����׽�жϱ�־λ�������־������ͨ���������һ�����
	else//���û����ɲ�׽
	{
		if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)//����������ж�
		{
			if(TIM1CH1_CAPTURE_STA&0X40)//�������񵽸ߵ�ƽ��
			{
				if((TIM1CH1_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ�����洢����
				{
					TIM1CH1_CAPTURE_STA|=0X80;		//ǿ�Ʊ�ǳɹ�������һ��
					TIM1CH1_CAPTURE_VAL=0XFFFF;
				}
				else
					TIM1CH1_CAPTURE_STA++;//�����������
			}
			//���־λ�ŵ���󣬸�ͨ���������һ�����
		}
		
		if(TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)//�������ж�
		{
			if(TIM1CH1_CAPTURE_STA&0X40)		//������׽�������� �˴�Ϊ������ 		
			{
				TIM1CH1_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
			  TIM1CH1_CAPTURE_VAL=TIM_GetCapture1(TIM1)+((TIM1CH1_CAPTURE_STA&0x3F)*0XFFFF)-TIM1CH1_CAPTURE_VAL;//��������.TIM2/5��32λ
	 			TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
			}
			else  								//��δ��ʼ,��һ�β���������
			{
				TIM1CH1_CAPTURE_STA=0;			//���
				TIM1CH1_CAPTURE_VAL=0;
				TIM1CH1_CAPTURE_STA|=0X40;		//��ǲ�����������
				TIM1CH1_CAPTURE_VAL=TIM_GetCapture1(TIM1);
	 			TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
			}
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC1); //��������жϱ�־λ
		}
	}
	
	if((TIM1CH2_CAPTURE_STA&0X80)==1)//����Ѿ�����˲�׽
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC2); //�����׽�жϱ�־λ�������־������ͨ���������һ�����
	else//���û����ɲ�׽
	{
		if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)//����������ж�
		{
			if(TIM1CH2_CAPTURE_STA&0X40)//�������񵽸ߵ�ƽ��
			{
				if((TIM1CH2_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ�����洢����
				{
					TIM1CH2_CAPTURE_STA|=0X80;		//ǿ�Ʊ�ǳɹ�������һ��
					TIM1CH2_CAPTURE_VAL=0XFFFF;
				}
				else
					TIM1CH2_CAPTURE_STA++;//�����������
			}
			//���־λ�ŵ���󣬸�ͨ���������һ�����
		}
		
		if(TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET)//�������ж�
		{
			if(TIM1CH2_CAPTURE_STA&0X40)		//������׽�������� �˴�Ϊ������ 		
			{
				TIM1CH2_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
			  TIM1CH2_CAPTURE_VAL=TIM_GetCapture2(TIM1)+(TIM1CH2_CAPTURE_STA&0x3F)*0XFFFF-TIM1CH2_CAPTURE_VAL;//��������.TIM2/5��32λ
	 			TIM_OC2PolarityConfig(TIM1,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
			}
			else  								//��δ��ʼ,��һ�β���������
			{
				TIM1CH2_CAPTURE_STA=0;			//���
				TIM1CH2_CAPTURE_VAL=0;
				TIM1CH2_CAPTURE_STA|=0X40;		//��ǲ�����������
				TIM1CH2_CAPTURE_VAL=TIM_GetCapture2(TIM1);
	 			TIM_OC2PolarityConfig(TIM1,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
			}
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC2); //��������жϱ�־λ
		}
	}
	
	if((TIM1CH3_CAPTURE_STA&0X80)==1)//����Ѿ�����˲�׽
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC3); //�����׽�жϱ�־λ�������־������ͨ���������һ�����
	else//���û����ɲ�׽
	{
		if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)//����������ж�
		{
			if(TIM1CH3_CAPTURE_STA&0X40)//�������񵽸ߵ�ƽ��
			{
				if((TIM1CH3_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ�����洢����
				{
					TIM1CH3_CAPTURE_STA|=0X80;		//ǿ�Ʊ�ǳɹ�������һ��
					TIM1CH3_CAPTURE_VAL=0XFFFF;
				}
				else
					TIM1CH3_CAPTURE_STA++;//�����������
			}
			//���־λ�ŵ���󣬸�ͨ���������һ�����
		}
		
		if(TIM_GetITStatus(TIM1, TIM_IT_CC3) != RESET)//�������ж�
		{
			if(TIM1CH3_CAPTURE_STA&0X40)		//������׽�������� �˴�Ϊ������ 		
			{
				TIM1CH3_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
			  TIM1CH3_CAPTURE_VAL=TIM_GetCapture3(TIM1)+(TIM1CH3_CAPTURE_STA&0x3F)*0XFFFF-TIM1CH3_CAPTURE_VAL;//��������.TIM2/5��32λ
	 			TIM_OC3PolarityConfig(TIM1,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
			}
			else  								//��δ��ʼ,��һ�β���������
			{
				TIM1CH3_CAPTURE_STA=0;			//���
				TIM1CH3_CAPTURE_VAL=0;
				TIM1CH3_CAPTURE_STA|=0X40;		//��ǲ�����������
				TIM1CH3_CAPTURE_VAL=TIM_GetCapture3(TIM1);
	 			TIM_OC3PolarityConfig(TIM1,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
			}
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC3); //��������жϱ�־λ
		}
	}
	
	if((TIM1CH4_CAPTURE_STA&0X80)==1)//����Ѿ�����˲�׽
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC4); //�����׽�жϱ�־λ�������־������ͨ���������һ�����
	else//���û����ɲ�׽
	{
		if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)//����������ж�
		{
			if(TIM1CH4_CAPTURE_STA&0X40)//�������񵽸ߵ�ƽ��
			{
				if((TIM1CH4_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ�����洢����
				{
					TIM1CH4_CAPTURE_STA|=0X80;		//ǿ�Ʊ�ǳɹ�������һ��
					TIM1CH4_CAPTURE_VAL=0XFFFF;
				}
				else
					TIM1CH4_CAPTURE_STA++;//�����������
			}
			//���־λ�ŵ���󣬸�ͨ���������һ�����
		}
		
		if(TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET)//�������ж�
		{
			if(TIM1CH4_CAPTURE_STA&0X40)		//������׽�������� �˴�Ϊ������ 		
			{
				TIM1CH4_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
			  TIM1CH4_CAPTURE_VAL=TIM_GetCapture4(TIM1)+(TIM1CH4_CAPTURE_STA&0x3F)*0XFFFF-TIM1CH4_CAPTURE_VAL;//��������.TIM2/5��32λ
	 			TIM_OC4PolarityConfig(TIM1,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
			}
			else  								//��δ��ʼ,��һ�β���������
			{
				TIM1CH4_CAPTURE_STA=0;			//���
				TIM1CH4_CAPTURE_VAL=0;
				TIM1CH4_CAPTURE_STA|=0X40;		//��ǲ�����������
				TIM1CH4_CAPTURE_VAL=TIM_GetCapture4(TIM1);
	 			TIM_OC4PolarityConfig(TIM1,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
			}
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC4); //��������жϱ�־λ
		}
	}
	
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update); //�������жϱ�־λ
}
