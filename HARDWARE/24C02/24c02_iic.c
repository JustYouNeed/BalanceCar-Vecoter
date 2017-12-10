# include "24c02.h"

void I2C_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);//使能GPIOB时钟

  //GPIOB8,B9初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//普通输出模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化
	
	SDA_W = 1;
	SCL = 1;
}

void I2C_Start(void)
{
	XX_SDA_OUT();
	SDA_W = 1;
	SCL = 1;
	delay_us(10);
	SDA_W = 0;
	delay_us(10);
	SCL = 0;
}

void I2C_Stop(void)
{
	XX_SDA_OUT();
	SCL = 0;
	SDA_W  = 0;
	delay_us(10);
	SDA_W = 1;
	SCL = 1;
	delay_us(10);
}

u8 I2C_Wait_Ack(void)
{
	u8 ucErrTime=0;
	XX_SDA_IN();      //SDA设置为输入  
	SDA_W=1;delay_us(1);	   
	SCL=1;delay_us(1);	 
	while(SDA_R)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			I2C_Stop();
			return 1;
		}
	}
	SCL=0;//时钟输出0 	   
	return 0;  
} 

void I2C_Ack(void)
{
	SCL=0;
	XX_SDA_OUT();
	SDA_W=0;
	delay_us(5);
	SCL=1;
	delay_us(5);
	SCL=0;
}
//不产生ACK应答		    
void I2C_NAck(void)
{
	SCL=0;
	XX_SDA_OUT();
	SDA_W=1;
	delay_us(5);
	SCL=1;
	delay_us(5);
	SCL=0;
}	

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void I2C_Send_Byte(u8 txd)
{                        
	u8 t;   
	XX_SDA_OUT(); 	    
	SCL=0;//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{              
			SDA_W=(txd&0x80)>>7;
			txd<<=1; 	  
	delay_us(5);   //对TEA5767这三个延时都是必须的
	SCL=1;
	delay_us(5); 
	SCL=0;	
	delay_us(2);
	}	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 I2C_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	XX_SDA_IN();//SDA设置为输入
	for(i=0;i<8;i++ )
	{
		SCL=0; 
		delay_us(5);
		SCL=1;
        receive<<=1;
        if(SDA_R)receive++;   
		delay_us(3); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}

