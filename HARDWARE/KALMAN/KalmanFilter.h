# ifndef __KALMAN_H
# define __KALMAN_H
# include "sys.h"

typedef struct {
	float Angle_Final;//最终角度
	float Angle_Dot;
	float Angle_Err; 	//角度偏差
	float Q_Bias;//测量偏差
	float Q_Angle;//角度协方差
	float Q_Gyro;
	float Gyroc;
	float R_Angle;
	float dt;
	char C_0;
	float E;
	float PCt[2];
	float K[2];
	float t[2];
	float PDot[4];
	float PP[2][2];
}KalmanFilter_Str;

static KalmanFilter_Str KalmanFilter_Pitch;
static KalmanFilter_Str KalmanFilter_Roll;
static KalmanFilter_Str KalmanFilter_Yaw;

void KalmanFilter_Init(KalmanFilter_Str * KalmanInitStructure);
float KalmanFilter(float Accel,float Gyro,KalmanFilter_Str * KalmanStructure);


# endif
