# include "motor.h"
# include "encoder.h"
# include "oled.h"
# include "led.h"
# include "mpu6050.h"
# include "inv_mpu.h"
# include "inv_mpu_dmp_motion_driver.h"
# include "delay.h"

VECTOR_Dev	VECTOR;

void Motor_Config(void)
{
	VECTOR.Dirction = HALT;
	VECTOR.M1_Dirction = VECTOR.Dirction;
	VECTOR.M2_Dirction = VECTOR.Dirction;
	VECTOR.BL_State = BL_UNCONECTE;
	VECTOR.M1_Encoder = 0;
	VECTOR.M2_Encoder = 0;
	VECTOR.M1_Pwm = 0;
	VECTOR.M2_Pwm = 0;
	VECTOR.Voltage = 0;
	VECTOR.Balance_Kp = -210.0f;
	VECTOR.Balance_Ki = 0.0f;
	VECTOR.Balance_Kd = -0.5f;
	VECTOR.Velocity_Kp = -130.5f;
	VECTOR.Velocity_Ki = -0.69f;
	VECTOR.Velocity_Kd = 0.0f;
	VECTOR.Turn_Kp = 0.0f;
	VECTOR.Turn_Ki = 0.0f;
	VECTOR.Turn_Kd = 0.0f;
	
	TIM2_PWM_Config(7200-1,0);
	Encoder_Init_TIM3();
	Encoder_Init_TIM4();
	MPU_EXTI_Init(); //��ʼ��6050�ⲿ�ж�
}


void TIM2_PWM_Config(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	//ʹ�ܶ�ʱ��3ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOC |RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3; //TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIO
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);// 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);  //ʹ��GPIO����ʱ��ʹ��
   //���ø�����Ϊ�����������,���TIM1 CH1 CH4��PWM���岨��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11; //TIM_CH1 //TIM_CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0;                            //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx

  TIM_CtrlPWMOutputs(TIM1,ENABLE);	//MOE �����ʹ��	

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH1Ԥװ��ʹ��	 
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH4Ԥװ��ʹ��	 
	
	TIM_ARRPreloadConfig(TIM1, ENABLE); //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
	
	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM1
}

void Motor_SetPwm(int motor1,int motor2)
{
	if(motor1<0)
	{
		M1_ENA = 0;
		M1_ENB = 1;
		motor1 = -motor1;
	}else
	{
		M1_ENA = 1;
		M1_ENB = 0;
	}
	if(motor1>7200)motor1 = 7000;
	
	if(motor2<0)
	{
		M2_ENA = 0;
		M2_ENB = 1;
		motor2 = -motor2 + 20;
	}else
	{
		M2_ENA = 1;
		M2_ENB = 0;
	//	motor2+=25; 
	}
	if(motor2>7200)motor2 = 7000;
	
	TIM_SetCompare1(TIM1,motor1);
	TIM_SetCompare4(TIM1,motor2);
}