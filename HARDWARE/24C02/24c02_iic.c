# include "24c02.h"

void I2C_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);//ʹ��GPIOBʱ��

  //GPIOB8,B9��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��
	
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
	XX_SDA_IN();      //SDA����Ϊ����  
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
	SCL=0;//ʱ�����0 	   
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
//������ACKӦ��		    
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

//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void I2C_Send_Byte(u8 txd)
{                        
	u8 t;   
	XX_SDA_OUT(); 	    
	SCL=0;//����ʱ�ӿ�ʼ���ݴ���
	for(t=0;t<8;t++)
	{              
			SDA_W=(txd&0x80)>>7;
			txd<<=1; 	  
	delay_us(5);   //��TEA5767��������ʱ���Ǳ����
	SCL=1;
	delay_us(5); 
	SCL=0;	
	delay_us(2);
	}	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 I2C_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	XX_SDA_IN();//SDA����Ϊ����
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
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}

