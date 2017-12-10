# include "motor.h"
# include "led.h"
# include "mpu6050.h"
# include "inv_mpu.h"
# include "inv_mpu_dmp_motion_driver.h"
# include "encoder.h"




int iVelocity_Pwm = 0, iTurn_Pwm, iBlance_Pwm = 0;
int time = 0;
static float err_sum = 0;
int i =0 ;
int Run;
static float sfEncoder_Integral,sfTarget_Velocity = 50;			//
static float sfBalance_ErrSum;//
//////////////////////////////////////////////////
//函数功能：MPU6050的中断函数，定时时间50ms
//入口参数：无
//返回值：无
///////////////////////////////////////////////////
int BalanceCar_Control_Interrupt(void)
{
	if(PBin(12) == 0)//中断发生
	{
		EXTI_ClearITPendingBit(EXTI_Line12);
	//	EXTI->PR=1<<12;   		//清除LINE12上的中断标志位  
		time++;
		if(time%20==0)
		{
			LED0=~LED0;

			time = 0;
		}
		if(mpu_dmp_get_data(&VECTOR.pitch,&VECTOR.roll,&VECTOR.yaw)==0)
		{
			MPU_Get_Accelerometer(&VECTOR.aacx,&VECTOR.aacy,&VECTOR.aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&VECTOR.gyrox,&VECTOR.gyroy,&VECTOR.gyroz);	//得到陀螺仪数据
		}else return 0;

		if(VECTOR.pitch<80 && VECTOR.pitch>-80 && !Run){     //只有角度在50度到-50度才进行平衡
			VECTOR.M1_Encoder = -Read_Encoder(3);
			VECTOR.M2_Encoder = Read_Encoder(4);
			
			iBlance_Pwm = Balance_PID(VECTOR.pitch, VECTOR.gyroy);
			iVelocity_Pwm = Velocity_PID(VECTOR.M1_Encoder, VECTOR.M2_Encoder);
			
			VECTOR.M1_Pwm = iBlance_Pwm + iVelocity_Pwm;
			VECTOR.M2_Pwm = iBlance_Pwm + iVelocity_Pwm;;

			if(!Pick_Up(VECTOR.aacz, VECTOR.pitch, VECTOR.M1_Encoder, VECTOR.M2_Encoder))
				Motor_SetPwm(VECTOR.M1_Pwm, VECTOR.M2_Pwm);
			else
				Motor_SetPwm(0,0);			
		}else{        //当小车角度大于50度或小于-50度时直接关闭小车
			M1_STOP();
			M2_STOP();
			Motor_SetPwm(0,0);
			err_sum = 0;
			sfEncoder_Integral = 0;
			sfBalance_ErrSum = 0;
			Run = 0;
		}
	}
	return 1;
}
/////////////////////////////////////
//函数功能：直立环PID，控制小车平衡
//入口参数：当前小车角度，角速度
//返回值：  输出到电机上的PWM值
/////////////////////////////////////
int Balance_PID(float angle,float Gyro)
{
	float fBias;   //每个时刻的偏差
	int iBalance_PWM; //计算后的PWM

	fBias = angle + 4;//偏差等于当前角度减去初始化时的偏差角再减去无电机控制时能直立的角度
	sfBalance_ErrSum += fBias; //以和代替积分
	if(sfBalance_ErrSum > 7200) sfBalance_ErrSum = 7200;  //积分限幅
	else if(sfBalance_ErrSum<-7200)	sfBalance_ErrSum = -7200;
	iBalance_PWM = VECTOR.Balance_Kp * fBias + VECTOR.Balance_Ki * sfBalance_ErrSum + VECTOR.Balance_Kd * Gyro;
	return iBalance_PWM;
}

//////////////////////////////////////
//函数功能：速度环PID，控制小车在平衡的同时速度为零
//入口参数：小车电机左右编码器的读数
//返回值：输出到电机上的PWM
//////////////////////////////////////////////////////////
int Velocity_PID(int iEncoder_Left, int iEncoder_Right)
{
	static float sfVelocity,sfEncoder_Least,sfEncoder,sfMovement = 0;		//速度环计算结果，最新速度偏差，

	
	if(VECTOR.Dirction == POSITIVE)	sfMovement = -sfTarget_Velocity;
	else if(VECTOR.Dirction == NEGETIVE) sfMovement = sfTarget_Velocity;
	else	sfMovement = 0;
	sfEncoder_Least = iEncoder_Left + iEncoder_Right - 0;
	sfEncoder *= 0.7;													//
	sfEncoder += sfEncoder_Least * 0.3;									//
	sfEncoder_Integral += sfEncoder;	
	sfEncoder_Integral -= sfMovement;
	
	if(sfEncoder_Integral > 7200) sfEncoder_Integral = 7200;			//积分限幅
	else if(sfEncoder_Integral < -7200) sfEncoder_Integral = -7200;		//
	
	sfVelocity = sfEncoder * VECTOR.Velocity_Kp + sfEncoder_Integral * VECTOR.Velocity_Ki;	//速度环PID计算
	
	return sfVelocity;
}

/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/**************************************************************************
函数功能：检测小车是否被拿起
入口参数：int
返回  值：unsigned int
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count0,count1,count2;
	if(flag==0)                                                                   //第一步
	 {
	      if(myabs(encoder_left)+myabs(encoder_right)<30)                         //条件1，小车接近静止
				count0++;
        else 
        count0=0;		
        if(count0>10)				
		    flag=1,count0=0; 
	 } 
	 if(flag==1)                                                                  //进入第二步
	 {
		    if(++count1>200)       count1=0,flag=0;                                 //超时不再等待2000ms
	      if(Acceleration>26000&&(Angle>(-20+ZHONGZHI))&&(Angle<(20+ZHONGZHI)))   //条件2，小车是在0度附近被拿起
		    flag=2; 
	 } 
	 if(flag==2)                                                                  //第三步
	 {
		  if(++count2>100)       count2=0,flag=0;                                   //超时不再等待1000ms
	    if(myabs(encoder_left+encoder_right)>200)                                 //条件3，小车的轮胎因为正反馈达到最大的转速   
      {
				flag=0;                                                                                     
				return 1;                                                               //检测到小车被拿起
			}
	 }
	return 0;
}

