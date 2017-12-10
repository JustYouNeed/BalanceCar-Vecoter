#	include "led.h"
#	include "delay.h"
#	include "sys.h"
# include "usart.h"
# include "pwm.h"
# include "oled.h"
# include "mpu6050.h"
#	include "inv_mpu.h"
#	include "inv_mpu_dmp_motion_driver.h"
# include "bsp_battery.h"
# include "timer.h"
# include "encoder.h"
# include "motor.h"

void System_Config(void);
 
int motor1 = 0,motor2 = 0;


int main(void)
{	
	float temp;
	System_Config();
//	printf("AT\r\n");
//	delay_ms(1000);
//	printf("AT+PSWD=1996\r\n");
//	delay_ms(1000);
//	printf("AT+ROLE=0\r\n");
//	delay_ms(1000);
//	printf("AT+UART=9600,0,0\r\n");
//	delay_ms(1000);
//	printf("AT+NAME=VECOTOR\r\n");
//	delay_ms(100);
//	printf("AT+RESET\r\n");
	//usart_init();

	
	OLED_Clear();
	OLED_ShowString(14,0,(u8*)"Balance Mode");
	
	OLED_ShowString(0,4,(u8*)"Cur Angle: 0");
	OLED_ShowString(104,4,(u8*)".0 ");

	OLED_ShowString(0,6,(u8*)"Distance: 0");
	OLED_ShowString(96,6,(u8*)".0 ");

	while(1)
	{
		temp = VECTOR.pitch;
		if(temp<0)
		{
			temp = -temp;
			OLED_ShowChar(80,4,'-');
		}else 
		{
			OLED_ShowChar(80,4,' ');
		}
		OLED_ShowNum(88,4,temp,2,16);
		OLED_ShowNum(112,4,(temp*10),1,16);
		delay_ms(100);
		VECTOR.Voltage = BSP_GetBatteryValue();
		temp = VECTOR.Voltage;
		OLED_ShowNum(80,6,temp,2,16);
		OLED_ShowNum(112,6,temp*10,1,16);
		if(VECTOR.M1_Pwm<0)motor1 = -VECTOR.M1_Pwm;
		if(VECTOR.M2_Pwm<0)motor2 = -VECTOR.M1_Pwm;
		printf("{A%d:%d:%d:%d}$",motor1,motor2,100,VECTOR.Dirction);
	}
}


void System_Config()
{
	delay_init();	    	 //延时函数初始化	  
	LED_Init();		  	//初始化与LED连接的硬件接口
	uart_init(9600);
	OLED_Config();
	BSP_BatteryConfig();
	
	OLED_ShowString(0,0,(u8*)"System Config...");
	delay_ms(100);
	
	OLED_ShowString(0,2,(u8*)"6050 Config...");
	delay_ms(100);
	
/************* MPU6050 初始化 *******************/
	while( MPU_Init())
	{
		delay_ms(100);
		OLED_ShowString(0,2,(u8*)"6050 error");
	}
/************************************************/	

	delay_ms(200);
	OLED_Clear();
	OLED_ShowString(0,0,(u8*)"System Config...");
	OLED_ShowString(0,2,(u8*)"6050 OK");
	delay_ms(300);
	OLED_ShowString(0,4,(u8*)"MPU Config...");
	
/************MPU DMP 初始化***********************/
	while(mpu_dmp_init())
	{
		OLED_Clear();
		OLED_SetFontSize(24);
		OLED_ShowString(6,0,(u8*)"MPU Error!");
		OLED_SetFontSize(16);
		OLED_ShowString(0,4,(u8*)"please restart!!");
		delay_ms(50);
	}
/************************************************/
	
	OLED_SetFontSize(16);
	delay_ms(200);
	OLED_Clear();
	OLED_ShowString(0,0,(u8*)"System Config...");
	OLED_ShowString(0,2,(u8*)"6050 OK");
	OLED_ShowString(0,4,(u8*)"MPU OK");
	delay_ms(2000);	
	
	OLED_Clear();
	OLED_ShowString(0,0,(u8*)"System Config...");
	OLED_ShowString(0,2,(u8*)"6050 OK");
	OLED_ShowString(0,4,(u8*)"MPU OK");
	OLED_ShowString(0, 6, (u8*)"Motor Config...");
	Motor_Config();
	delay_ms(2000);
	OLED_Clear();
	OLED_ShowString(0,0,(u8*)"System Config...");
	OLED_ShowString(0,2,(u8*)"6050 OK");
	OLED_ShowString(0,4,(u8*)"MPU OK");
	OLED_ShowString(0, 6, (u8*)"Motor OK");
	delay_ms(2000);
}

