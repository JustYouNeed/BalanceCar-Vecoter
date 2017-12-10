# include "KalmanFilter.h"

//static float fAngle_Final = 0.0f;//最终计算出的角度
//static float fGyroc_y;	//Y轴陀螺仪数据暂存
//static float Angle_err = 0.0f;
//static float Q_Bias = 0.0f;//偏差
//static float Q_Angle = 0.1f;//角度数据置信度
//static float Q_Gyro = 0.03f;//角速度数据置信度
//static float R_Angle = 0.5f;//测量噪声的协方差
//static float dt	=	0.5f;//采样时间
//static char C_0 = 1.0f;
//static float PCt_0 = 0,PCt_1 = 0,E = 0;
//static float K_0 = 0,K_1 = 0,t_0 = 0,t_1 = 0;
//static float PDot[4] = {0,0,0,0};
//static float PP[2][2] = {{1,0},{0,1}};

void KalmanFilter_Init(KalmanFilter_Str * KalmanInitStructure)
{
	KalmanInitStructure->Angle_Final = 0.0f;
	KalmanInitStructure->Angle_Err = 0.0f;
	KalmanInitStructure->C_0 = 1.0f;
	KalmanInitStructure->dt = 0.005f;
	KalmanInitStructure->K[0] = 0;
	KalmanInitStructure->K[1] = 0;
	KalmanInitStructure->PCt[0] = 0;
	KalmanInitStructure->PCt[1] = 0;
	KalmanInitStructure->PDot[0] = 0;
	KalmanInitStructure->PDot[1] = 0;
	KalmanInitStructure->PDot[2] = 0;
	KalmanInitStructure->PDot[3] = 0;
	KalmanInitStructure->PP[0][0] = 1;
	KalmanInitStructure->PP[0][1] = 0;
	KalmanInitStructure->PP[1][0] = 0;
	KalmanInitStructure->PP[1][1] = 1;
	KalmanInitStructure->Q_Angle = 0.001f;
	KalmanInitStructure->Q_Bias = 0.0f;
	KalmanInitStructure->Q_Gyro = 0.03f;
	KalmanInitStructure->R_Angle = 0.5f;
	KalmanInitStructure->t[0] = 0;
	KalmanInitStructure->t[1] = 1;
}

float KalmanFilter(float Gyro,float Accel,KalmanFilter_Str * KalmanStructure)
{
	KalmanStructure->Angle_Final += (KalmanStructure->Q_Gyro - KalmanStructure->Q_Bias)*KalmanStructure->dt;
	KalmanStructure->Angle_Err = Accel - KalmanStructure->Angle_Final;
	
	
//	fAngle_Final = fAngle_Final + (Gyro - Q_Bias)*dt;//先验估计，对应第一个公式
//	Angle_err = Accel - fAngle_Final;//Zk先验估计
//	
	KalmanStructure->PDot[0] = KalmanStructure->Q_Angle - KalmanStructure->PP[0][0] - KalmanStructure->PP[1][0];
	KalmanStructure->PDot[1] = -KalmanStructure->PP[1][1];
	KalmanStructure->PDot[2] = -KalmanStructure->PP[1][1];
	KalmanStructure->PDot[3] = KalmanStructure->Q_Gyro;
	
//	PDot[0] = Q_Angle - PP[0][0] - PP[1][0];
//	PDot[1] = -PP[1][1];
//	PDot[2] = -PP[1][1];
//	PDot[3] = Q_Gyro;
	
	KalmanStructure->PP[0][0] += KalmanStructure->PDot[0]*KalmanStructure->dt;
	KalmanStructure->PP[0][1] += KalmanStructure->PDot[1]*KalmanStructure->dt;
	KalmanStructure->PP[1][0] += KalmanStructure->PDot[2]*KalmanStructure->dt;
	KalmanStructure->PP[1][1] += KalmanStructure->PDot[3]*KalmanStructure->dt;
	
	
//	PP[0][0] = PP[0][0] + PDot[0]*dt;
//	PP[0][1] = PP[0][1] + PDot[1]*dt;
//	PP[1][0] = PP[1][0] + PDot[2]*dt;
//	PP[1][1] = PP[1][1] + PDot[3]*dt;
	
	KalmanStructure->PCt[0] = KalmanStructure->C_0*KalmanStructure->PP[0][0];
	KalmanStructure->PCt[1] = KalmanStructure->C_0*KalmanStructure->PP[1][0];
	
//	PCt_0 = C_0*PP[0][0];
//	PCt_1 = C_0*PP[1][0];
	
	KalmanStructure->E = KalmanStructure->C_0*KalmanStructure->PCt[0] +  KalmanStructure->R_Angle;
	
	
//	E = C_0 * PCt_0 + R_Angle;
	
	KalmanStructure->K[0] = KalmanStructure->PCt[0]/KalmanStructure->E;
	KalmanStructure->K[1] = KalmanStructure->PCt[1]/KalmanStructure->E;
	
//	K_0 = PCt_0/E;
//	K_1 = PCt_1/E;
	
	KalmanStructure->t[0] = KalmanStructure->PCt[0];
	KalmanStructure->t[1] = KalmanStructure->C_0*KalmanStructure->PP[0][1];
	
//	t_0 = PCt_0;
//	t_1 = C_0 * PP[0][1];
	
	
	KalmanStructure->PP[0][0] -= KalmanStructure->K[0]*KalmanStructure->t[0];
	KalmanStructure->PP[0][1] -= KalmanStructure->K[0]*KalmanStructure->t[1];
	KalmanStructure->PP[1][0] -= KalmanStructure->K[1]*KalmanStructure->t[0];
	KalmanStructure->PP[1][1] -= KalmanStructure->K[1]*KalmanStructure->t[1];

//	PP[0][0] = PP[0][0] - K_0*t_0;
//	PP[0][1] = PP[0][1] - K_0*t_1;
//	PP[1][0] = PP[1][0] - K_1*t_0;
//	PP[1][1] = PP[1][1] - K_1*t_1;
	
	KalmanStructure->Angle_Final += KalmanStructure->K[0]*KalmanStructure->Angle_Err;
	KalmanStructure->Q_Bias += KalmanStructure->K[1]*KalmanStructure->Angle_Err;

	KalmanStructure->Angle_Dot = Gyro - KalmanStructure->Q_Bias;
//	fAngle_Final = fAngle_Final + K_0*Angle_err;
//	Q_Bias = Q_Bias + K_1*Angle_err;
//	fGyroc_y = Gyro - Q_Bias;
//	return fAngle_Final;
	return 0;
}

