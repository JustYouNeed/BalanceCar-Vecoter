# ifndef __MOTOR_H
# define __MOTOR_H

# include "sys.h"


# define ZHONGZHI 3
#	define PI 3.14159265

# define BalanceCar_Control_Interrupt EXTI15_10_IRQHandler

# define M1_ENA PCout(0)
# define M1_ENB PCout(1)
# define M2_ENA PCout(3)
# define M2_ENB PCout(2)

# define M1_STOP()			{M1_ENA = 1, M1_ENB = 1;}
# define M1_POSITIVE()	{M1_ENA = 1, M1_ENB = 0;}
# define M1_NEGETIVE()	{M1_ENA = 0, M1_ENB = 1;}

# define M2_STOP()			{M2_ENA = 1, M2_ENB = 1;}
# define M2_POSITIVE()	{M2_ENA = 1, M2_ENB = 0;}
# define M2_NEGETIVE()	{M2_ENA = 0, M2_ENB = 1;}


typedef enum{POSITIVE = 0x00, NEGETIVE, HALT,
							TURN_FRONT_LEFT, TURN_FRONT_RIGHT,
							TURN_BACK_LEFT, TURN_BACK_RIGHT,
							BL_UNCONECTE, BL_CONECTED}VECTOR_STATE;

typedef struct
{
	uint8_t Dirction;  //车子方向
	uint8_t M1_Dirction, M2_Dirction;
	uint8_t BL_State;  //蓝牙连接状态
	uint8_t Voltage;   //电池电压
	int32_t M1_Pwm, M2_Pwm;  //电机pwm
	int32_t  M1_Encoder, M2_Encoder;  //电机编码器
	
	float		Balance_Kp, Balance_Ki, Balance_Kd;
	float 	Velocity_Kp, Velocity_Ki, Velocity_Kd;
	float 	Turn_Kp, Turn_Ki, Turn_Kd;
	
	float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
}VECTOR_Dev;

extern VECTOR_Dev	VECTOR;


void Motor_Config(void);

static float Init_Angle = 0;
int BalanceCar_Control_Interrupt(void);
int Balance_PID(float angle,float Gyro);
int Velocity_PID(int iEncoder_Left, int iEncoder_Right);
int balance(float Angle,float Gyro);
int velocity(int encoder_left,int encoder_right);
int speed(int encoder_left,int encoder_right);

void TIM2_PWM_Config(u16 arr,u16 psc);
void Motor_SetPwm(int motor1,int motor2);

int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right);
# endif

