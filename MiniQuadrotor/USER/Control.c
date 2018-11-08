#include "Control.h"
#include "include.h"

float Pitch_A_P=76.3848,Pitch_A_I=1.31489,Pitch_A_D=14.5865;
float Pitch_V_P=0.01459,Pitch_V_I=0.00000,Pitch_V_D=0.06180;

float Roll_A_P=76.3848,Roll_A_I=1.31489,Roll_A_D=14.5865;
float Roll_V_P=0.01459,Roll_V_I=0.00000,Roll_V_D=0.06180;

float Yaw_A_P=76.3848,Yaw_A_I=0.00000,Yaw_A_D=14.5865;
float Yaw_V_P=0.01459,Yaw_V_I=0.00000,Yaw_V_D=0.06180;

double Pitch_Angle_Speed_Want=0.0,Roll_Angle_Speed_Want=0.0,Yaw_Angle_Speed_Want=0.0;

double Pitch_Angle_Err[3]={0.0},Roll_Angle_Err[3]={0.0},Yaw_Angle_Err[3]={0.0};
double Pitch_Angle_Inte[2]={0.0},Roll_Angle_Inte[2]={0.0},Yaw_Angle_Inte[2]={0.0};
double Pitch_Angle_Diff[2]={0.0},Roll_Angle_Diff[2]={0.0},Yaw_Angle_Diff[2]={0.0};

double Pitch_Angle_Speed_Err[3]={0.0},Roll_Angle_Speed_Err[3]={0.0},Yaw_Angle_Speed_Err[3]={0.0};
double Pitch_Angle_Speed_Inte[2]={0.0},Roll_Angle_Speed_Inte[2]={0.0},Yaw_Angle_Speed_Inte[2]={0.0};
double Pitch_Angle_Speed_Diff[2]={0.0},Roll_Angle_Speed_Diff[2]={0.0},Yaw_Angle_Speed_Diff[2]={0.0};

double Pitch_Out=0.0,Roll_Out=0.0,Yaw_Out=0.0;

void Pitch_PID(void)
{
	Pitch_Angle_Err[2]=Pitch_Angle_Err[1];
	Pitch_Angle_Err[1]=Pitch_Angle_Err[0];
	Pitch_Angle_Err[0]=Pitch_Angle_Want-Pitch[0]-Pitch_Bias;
	if(Pitch_Angle_Err[0]>45.00000)Pitch_Angle_Err[0]=45.00000;
	if(Pitch_Angle_Err[0]<-45.00000)Pitch_Angle_Err[0]=-45.00000;
	
	Pitch_Angle_Inte[1]=Pitch_A_I*(0.5*Pitch_Angle_Err[0]+0.5*Pitch_Angle_Err[1]);
	if(Pitch_Angle_Inte[1]>25.00000)Pitch_Angle_Inte[1]=25.00000;
	if(Pitch_Angle_Inte[1]<-25.00000)Pitch_Angle_Inte[1]=-25.00000;
	Pitch_Angle_Inte[0]+=Pitch_Angle_Inte[1];
	if(Pitch_Angle_Inte[0]>25.00000)Pitch_Angle_Inte[0]=25.00000;
	if(Pitch_Angle_Inte[0]<-25.00000)Pitch_Angle_Inte[0]=-25.00000;
	
	Pitch_Angle_Diff[1]=Pitch_Angle_Diff[0];
	Pitch_Angle_Diff[0]=Pitch_A_D*(Pitch_Angle_Err[0]-Pitch_Angle_Err[1]);
	if(Pitch_Angle_Diff[0]>20000.00000)Pitch_Angle_Diff[0]=20000.00000;
	if(Pitch_Angle_Diff[0]<-20000.00000)Pitch_Angle_Diff[0]=-20000.00000;
	Pitch_Angle_Diff[0]=0.8*Pitch_Angle_Diff[0]+0.2*Pitch_Angle_Diff[1];
	
	Pitch_Angle_Speed_Want = Pitch_A_P*Pitch_Angle_Err[0]+Pitch_Angle_Inte[0]+Pitch_Angle_Diff[0];//
	
	if(Pitch_Angle_Speed_Want>20000.00000)Pitch_Angle_Speed_Want=20000.00000;
	if(Pitch_Angle_Speed_Want<-20000.00000)Pitch_Angle_Speed_Want=-20000.00000;
	
	Pitch_Angle_Speed_Err[2]=Pitch_Angle_Speed_Err[1];
	Pitch_Angle_Speed_Err[1]=Pitch_Angle_Speed_Err[0];
	Pitch_Angle_Speed_Err[0]=Pitch_Angle_Speed_Want-Gyo_Raw[0][0];
	
	Pitch_Angle_Speed_Inte[1]=Pitch_V_I*(0.5*Pitch_Angle_Speed_Err[0]+0.5*Pitch_Angle_Speed_Err[1]);
	if(Pitch_Angle_Speed_Inte[1]>400.00000)Pitch_Angle_Speed_Inte[1]=400.00000;
	if(Pitch_Angle_Speed_Inte[1]<-400.00000)Pitch_Angle_Speed_Inte[1]=-400.00000;
	Pitch_Angle_Speed_Inte[0]+=Pitch_Angle_Speed_Inte[1];
	if(Pitch_Angle_Speed_Inte[0]>400.00000)Pitch_Angle_Speed_Inte[0]=400.00000;
	if(Pitch_Angle_Speed_Inte[0]<-400.00000)Pitch_Angle_Speed_Inte[0]=-400.00000;
	
	Pitch_Angle_Speed_Diff[1]=Pitch_Angle_Speed_Diff[0];
	Pitch_Angle_Speed_Diff[0]=Pitch_V_D*(Pitch_Angle_Speed_Err[0]-Pitch_Angle_Speed_Err[1]);
	if(Pitch_Angle_Speed_Diff[0]>400.00000)Pitch_Angle_Speed_Diff[0]=400.00000;
	if(Pitch_Angle_Speed_Diff[0]<-400.00000)Pitch_Angle_Speed_Diff[0]=-400.00000;
	Pitch_Angle_Speed_Diff[0]=0.6*Pitch_Angle_Speed_Diff[0]+0.4*Pitch_Angle_Speed_Diff[1];
	
	Pitch_Out = Pitch_V_P*Pitch_Angle_Speed_Err[0]+Pitch_Angle_Speed_Inte[0]+Pitch_Angle_Speed_Diff[0];//
	
	if(Pitch_Out>400.00000)Pitch_Out=400.00000;
	if(Pitch_Out<-400.00000)Pitch_Out=-400.00000;
}

void Roll_PID(void)
{
	Roll_Angle_Err[2]=Roll_Angle_Err[1];
	Roll_Angle_Err[1]=Roll_Angle_Err[0];
	Roll_Angle_Err[0]=Roll_Angle_Want-Roll[0]-Roll_Bias;
	if(Roll_Angle_Err[0]>45.00000)Roll_Angle_Err[0]=45.00000;
	if(Roll_Angle_Err[0]<-45.00000)Roll_Angle_Err[0]=-45.00000;
	
	Roll_Angle_Inte[1]=Roll_A_I*(0.5*Roll_Angle_Err[0]+0.5*Roll_Angle_Err[1]);
	if(Roll_Angle_Inte[1]>25.00000)Roll_Angle_Inte[1]=25.00000;
	if(Roll_Angle_Inte[1]<-25.00000)Roll_Angle_Inte[1]=-25.00000;
	Roll_Angle_Inte[0]+=Roll_Angle_Inte[1];
	if(Roll_Angle_Inte[0]>25.00000)Roll_Angle_Inte[0]=25.00000;
	if(Roll_Angle_Inte[0]<-25.00000)Roll_Angle_Inte[0]=-25.00000;
	
	Roll_Angle_Diff[1]=Roll_Angle_Diff[0];
	Roll_Angle_Diff[0]=Roll_A_D*(Roll_Angle_Err[0]-Roll_Angle_Err[1]);
	if(Roll_Angle_Diff[0]>20000.00000)Roll_Angle_Diff[0]=20000.00000;
	if(Roll_Angle_Diff[0]<-20000.00000)Roll_Angle_Diff[0]=-20000.00000;
	Roll_Angle_Diff[0]=0.8*Roll_Angle_Diff[0]+0.2*Roll_Angle_Diff[1];
	
	Roll_Angle_Speed_Want = Roll_A_P*Roll_Angle_Err[0]+Roll_Angle_Inte[0]+Roll_Angle_Diff[0];//
	
	if(Roll_Angle_Speed_Want>20000.00000)Roll_Angle_Speed_Want=20000.00000;
	if(Roll_Angle_Speed_Want<-20000.00000)Roll_Angle_Speed_Want=-20000.00000;
	
	Roll_Angle_Speed_Err[2]=Roll_Angle_Speed_Err[1];
	Roll_Angle_Speed_Err[1]=Roll_Angle_Speed_Err[0];
	Roll_Angle_Speed_Err[0]=Roll_Angle_Speed_Want-Gyo_Raw[1][0];
	
	Roll_Angle_Speed_Inte[1]=Roll_V_I*(0.5*Roll_Angle_Speed_Err[0]+0.5*Roll_Angle_Speed_Err[1]);
	if(Roll_Angle_Speed_Inte[1]>400.00000)Roll_Angle_Speed_Inte[1]=400.00000;
	if(Roll_Angle_Speed_Inte[1]<-400.00000)Roll_Angle_Speed_Inte[1]=-400.00000;
	Roll_Angle_Speed_Inte[0]+=Roll_Angle_Speed_Inte[1];
	if(Roll_Angle_Speed_Inte[0]>400.00000)Roll_Angle_Speed_Inte[0]=400.00000;
	if(Roll_Angle_Speed_Inte[0]<-400.00000)Roll_Angle_Speed_Inte[0]=-400.00000;
	
	Roll_Angle_Speed_Diff[1]=Roll_Angle_Speed_Diff[0];
	Roll_Angle_Speed_Diff[0]=Roll_V_D*(Roll_Angle_Speed_Err[0]-Roll_Angle_Speed_Err[1]);
	if(Roll_Angle_Speed_Diff[0]>400.00000)Roll_Angle_Speed_Diff[0]=400.00000;
	if(Roll_Angle_Speed_Diff[0]<-400.00000)Roll_Angle_Speed_Diff[0]=-400.00000;
	Roll_Angle_Speed_Diff[0]=0.6*Roll_Angle_Speed_Diff[0]+0.4*Roll_Angle_Speed_Diff[1];
	
	Roll_Out = Roll_V_P*Roll_Angle_Speed_Err[0]+Roll_Angle_Speed_Inte[0]+Roll_Angle_Speed_Diff[0];//
	
	if(Roll_Out>400.00000)Roll_Out=400.00000;
	if(Roll_Out<-400.00000)Roll_Out=-400.00000;
}

void Yaw_PID(void)
{
	Yaw_Angle_Err[2]=Yaw_Angle_Err[1];
	Yaw_Angle_Err[1]=Yaw_Angle_Err[0];
	Yaw_Angle_Err[0]=Yaw_Angle_Want-Yaw[0]-Yaw_Bias;
	if(Yaw_Angle_Err[0]>180.00000)Yaw_Angle_Err[0]-=360.00000;
	if(Yaw_Angle_Err[0]<-180.00000)Yaw_Angle_Err[0]+=360.00000;
	
	Yaw_Angle_Inte[1]=Yaw_A_I*(0.5*Yaw_Angle_Err[0]+0.5*Yaw_Angle_Err[1]);
	if(Yaw_Angle_Inte[1]>25.00000)Yaw_Angle_Inte[1]=25.00000;
	if(Yaw_Angle_Inte[1]<-25.00000)Yaw_Angle_Inte[1]=-25.00000;
	Yaw_Angle_Inte[0]+=Yaw_Angle_Inte[1];
	if(Yaw_Angle_Inte[0]>25.00000)Yaw_Angle_Inte[0]=25.00000;
	if(Yaw_Angle_Inte[0]<-25.00000)Yaw_Angle_Inte[0]=-25.00000;
	
	Yaw_Angle_Diff[1]=Yaw_Angle_Diff[0];
	Yaw_Angle_Diff[0]=Yaw_A_D*(Yaw_Angle_Err[0]-Yaw_Angle_Err[1]);
	if(Yaw_Angle_Diff[0]>20000.00000)Yaw_Angle_Diff[0]=20000.00000;
	if(Yaw_Angle_Diff[0]<-20000.00000)Yaw_Angle_Diff[0]=-20000.00000;
	Yaw_Angle_Diff[0]=0.8*Yaw_Angle_Diff[0]+0.2*Yaw_Angle_Diff[1];
	
	Yaw_Angle_Speed_Want = Yaw_A_P*Yaw_Angle_Err[0]+Yaw_Angle_Inte[0]+Yaw_Angle_Diff[0];//
	
	if(Yaw_Angle_Speed_Want>20000.00000)Yaw_Angle_Speed_Want=20000.00000;
	if(Yaw_Angle_Speed_Want<-20000.00000)Yaw_Angle_Speed_Want=-20000.00000;
	
	Yaw_Angle_Speed_Err[2]=Yaw_Angle_Speed_Err[1];
	Yaw_Angle_Speed_Err[1]=Yaw_Angle_Speed_Err[0];
	Yaw_Angle_Speed_Err[0]=Yaw_Angle_Speed_Want-Gyo_Raw[2][0];
	
	Yaw_Angle_Speed_Inte[1]=Yaw_V_I*(0.5*Yaw_Angle_Speed_Err[0]+0.5*Yaw_Angle_Speed_Err[1]);
	if(Yaw_Angle_Speed_Inte[1]>400.00000)Yaw_Angle_Speed_Inte[1]=400.00000;
	if(Yaw_Angle_Speed_Inte[1]<-400.00000)Yaw_Angle_Speed_Inte[1]=-400.00000;
	Yaw_Angle_Speed_Inte[0]+=Yaw_Angle_Speed_Inte[1];
	if(Yaw_Angle_Speed_Inte[0]>400.00000)Yaw_Angle_Speed_Inte[0]=400.00000;
	if(Yaw_Angle_Speed_Inte[0]<-400.00000)Yaw_Angle_Speed_Inte[0]=-400.00000;
	
	Yaw_Angle_Speed_Diff[1]=Yaw_Angle_Speed_Diff[0];
	Yaw_Angle_Speed_Diff[0]=Yaw_V_D*(Yaw_Angle_Speed_Err[0]-Yaw_Angle_Speed_Err[1]);
	if(Yaw_Angle_Speed_Diff[0]>400.00000)Yaw_Angle_Speed_Diff[0]=400.00000;
	if(Yaw_Angle_Speed_Diff[0]<-400.00000)Yaw_Angle_Speed_Diff[0]=-400.00000;
	Yaw_Angle_Speed_Diff[0]=0.6*Yaw_Angle_Speed_Diff[0]+0.4*Yaw_Angle_Speed_Diff[1];
	
	Yaw_Out = Yaw_V_P*Yaw_Angle_Speed_Err[0]+Yaw_Angle_Speed_Inte[0]+Yaw_Angle_Speed_Diff[0];//
	
	if(Yaw_Out>400.00000)Yaw_Out=400.00000;
	if(Yaw_Out<-400.00000)Yaw_Out=-400.00000;
}

void Motor_Out(void)
{
	Motor_Out_1=(int)(Throttle+Pitch_Out+Roll_Out-Yaw_Out);
	Motor_Out_2=(int)(Throttle+Pitch_Out-Roll_Out+Yaw_Out);
	Motor_Out_3=(int)(Throttle-Pitch_Out+Roll_Out+Yaw_Out);
	Motor_Out_4=(int)(Throttle-Pitch_Out-Roll_Out-Yaw_Out);
	
	if(Motor_Out_1>400)Motor_Out_1=400;
	if(Motor_Out_1<0)Motor_Out_1=0;
	if(Motor_Out_2>400)Motor_Out_2=400;
	if(Motor_Out_2<0)Motor_Out_2=0;
	if(Motor_Out_3>400)Motor_Out_3=400;
	if(Motor_Out_3<0)Motor_Out_3=0;
	if(Motor_Out_4>400)Motor_Out_4=400;
	if(Motor_Out_4<0)Motor_Out_4=0;
}

void Pitch_PID_Clear(void)
{
	Pitch_Angle_Speed_Want=0.00000;
	Pitch_Angle_Err[0]=Pitch_Angle_Err[1]=Pitch_Angle_Err[2]=0.00000;
	Pitch_Angle_Inte[0]=Pitch_Angle_Inte[1]=0.00000;
	Pitch_Angle_Diff[0]=Pitch_Angle_Diff[1]=0.00000;
	Pitch_Angle_Speed_Err[0]=Pitch_Angle_Speed_Err[1]=Pitch_Angle_Speed_Err[2]=0.00000;
	Pitch_Angle_Speed_Inte[0]=Pitch_Angle_Speed_Inte[1]=0.00000;
	Pitch_Angle_Speed_Diff[0]=Pitch_Angle_Speed_Diff[1]=0.00000;
	Pitch_Out=0.00000;
}

void Roll_PID_Clear(void)
{
	Roll_Angle_Speed_Want=0.00000;
	Roll_Angle_Err[0]=Roll_Angle_Err[1]=Roll_Angle_Err[2]=0.00000;
	Roll_Angle_Inte[0]=Roll_Angle_Inte[1]=0.00000;
	Roll_Angle_Diff[0]=Roll_Angle_Diff[1]=0.00000;
	Roll_Angle_Speed_Err[0]=Roll_Angle_Speed_Err[1]=Roll_Angle_Speed_Err[2]=0.00000;
	Roll_Angle_Speed_Inte[0]=Roll_Angle_Speed_Inte[1]=0.00000;
	Roll_Angle_Speed_Diff[0]=Roll_Angle_Speed_Diff[1]=0.00000;
	Roll_Out=0.00000;
}

void Yaw_PID_Clear(void)
{
	Yaw_Angle_Speed_Want=0.00000;
	Yaw_Angle_Err[0]=Yaw_Angle_Err[1]=Yaw_Angle_Err[2]=0.00000;
	Yaw_Angle_Inte[0]=Yaw_Angle_Inte[1]=0.00000;
	Yaw_Angle_Diff[0]=Yaw_Angle_Diff[1]=0.00000;
	Yaw_Angle_Speed_Err[0]=Yaw_Angle_Speed_Err[1]=Yaw_Angle_Speed_Err[2]=0.00000;
	Yaw_Angle_Speed_Inte[0]=Yaw_Angle_Speed_Inte[1]=0.00000;
	Yaw_Angle_Speed_Diff[0]=Yaw_Angle_Speed_Diff[1]=0.00000;
	Yaw_Out=0.00000;
}
