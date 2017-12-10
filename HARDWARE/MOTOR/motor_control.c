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
//�������ܣ�MPU6050���жϺ�������ʱʱ��50ms
//��ڲ�������
//����ֵ����
///////////////////////////////////////////////////
int BalanceCar_Control_Interrupt(void)
{
	if(PBin(12) == 0)//�жϷ���
	{
		EXTI_ClearITPendingBit(EXTI_Line12);
	//	EXTI->PR=1<<12;   		//���LINE12�ϵ��жϱ�־λ  
		time++;
		if(time%20==0)
		{
			LED0=~LED0;

			time = 0;
		}
		if(mpu_dmp_get_data(&VECTOR.pitch,&VECTOR.roll,&VECTOR.yaw)==0)
		{
			MPU_Get_Accelerometer(&VECTOR.aacx,&VECTOR.aacy,&VECTOR.aacz);	//�õ����ٶȴ���������
			MPU_Get_Gyroscope(&VECTOR.gyrox,&VECTOR.gyroy,&VECTOR.gyroz);	//�õ�����������
		}else return 0;

		if(VECTOR.pitch<80 && VECTOR.pitch>-80 && !Run){     //ֻ�нǶ���50�ȵ�-50�ȲŽ���ƽ��
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
		}else{        //��С���Ƕȴ���50�Ȼ�С��-50��ʱֱ�ӹر�С��
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
//�������ܣ�ֱ����PID������С��ƽ��
//��ڲ�������ǰС���Ƕȣ����ٶ�
//����ֵ��  ���������ϵ�PWMֵ
/////////////////////////////////////
int Balance_PID(float angle,float Gyro)
{
	float fBias;   //ÿ��ʱ�̵�ƫ��
	int iBalance_PWM; //������PWM

	fBias = angle + 4;//ƫ����ڵ�ǰ�Ƕȼ�ȥ��ʼ��ʱ��ƫ����ټ�ȥ�޵������ʱ��ֱ���ĽǶ�
	sfBalance_ErrSum += fBias; //�Ժʹ������
	if(sfBalance_ErrSum > 7200) sfBalance_ErrSum = 7200;  //�����޷�
	else if(sfBalance_ErrSum<-7200)	sfBalance_ErrSum = -7200;
	iBalance_PWM = VECTOR.Balance_Kp * fBias + VECTOR.Balance_Ki * sfBalance_ErrSum + VECTOR.Balance_Kd * Gyro;
	return iBalance_PWM;
}

//////////////////////////////////////
//�������ܣ��ٶȻ�PID������С����ƽ���ͬʱ�ٶ�Ϊ��
//��ڲ�����С��������ұ������Ķ���
//����ֵ�����������ϵ�PWM
//////////////////////////////////////////////////////////
int Velocity_PID(int iEncoder_Left, int iEncoder_Right)
{
	static float sfVelocity,sfEncoder_Least,sfEncoder,sfMovement = 0;		//�ٶȻ��������������ٶ�ƫ�

	
	if(VECTOR.Dirction == POSITIVE)	sfMovement = -sfTarget_Velocity;
	else if(VECTOR.Dirction == NEGETIVE) sfMovement = sfTarget_Velocity;
	else	sfMovement = 0;
	sfEncoder_Least = iEncoder_Left + iEncoder_Right - 0;
	sfEncoder *= 0.7;													//
	sfEncoder += sfEncoder_Least * 0.3;									//
	sfEncoder_Integral += sfEncoder;	
	sfEncoder_Integral -= sfMovement;
	
	if(sfEncoder_Integral > 7200) sfEncoder_Integral = 7200;			//�����޷�
	else if(sfEncoder_Integral < -7200) sfEncoder_Integral = -7200;		//
	
	sfVelocity = sfEncoder * VECTOR.Velocity_Kp + sfEncoder_Integral * VECTOR.Velocity_Ki;	//�ٶȻ�PID����
	
	return sfVelocity;
}

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/**************************************************************************
�������ܣ����С���Ƿ�����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count0,count1,count2;
	if(flag==0)                                                                   //��һ��
	 {
	      if(myabs(encoder_left)+myabs(encoder_right)<30)                         //����1��С���ӽ���ֹ
				count0++;
        else 
        count0=0;		
        if(count0>10)				
		    flag=1,count0=0; 
	 } 
	 if(flag==1)                                                                  //����ڶ���
	 {
		    if(++count1>200)       count1=0,flag=0;                                 //��ʱ���ٵȴ�2000ms
	      if(Acceleration>26000&&(Angle>(-20+ZHONGZHI))&&(Angle<(20+ZHONGZHI)))   //����2��С������0�ȸ���������
		    flag=2; 
	 } 
	 if(flag==2)                                                                  //������
	 {
		  if(++count2>100)       count2=0,flag=0;                                   //��ʱ���ٵȴ�1000ms
	    if(myabs(encoder_left+encoder_right)>200)                                 //����3��С������̥��Ϊ�������ﵽ����ת��   
      {
				flag=0;                                                                                     
				return 1;                                                               //��⵽С��������
			}
	 }
	return 0;
}

