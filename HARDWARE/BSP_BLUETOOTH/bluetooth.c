# include "bluetooth.h"
# include "delay.h"
# include "stdlib.h"
void BSP_BT_Config(u32 bound)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}

void BSP_BT_Sendbyte(uint8_t byte)
{
	while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET); 
	USART_SendData(USART3,byte); 
}

void BSP_BT_SendBuff(uint8_t *buff)
{
	uint8_t i = 0;
	while(buff[i++])
	{
		BSP_BT_Sendbyte(buff[i]);
	}
}

void BSP_BT_SetBoundRate(u32 bound)
{
	uint8_t buff[32];
	printf("AT\r\n");
	delay_ms(1000);
	sprintf(buff,"%d",bound);
	printf("AT+UART=%s,0,0\r\n",buff);
}