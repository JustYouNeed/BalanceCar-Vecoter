# ifndef __24C02_H
# define __24C02_H
# include "sys.h"
# include "delay.h"

# define 	SDA_W	PDout(2)
# define	SDA_R	PDin(2)
# define 	SCL		PCout(12)

#define XX_SDA_IN()  {GPIOD->CRL&=0XFFf0FfFF;GPIOD->CRL|=8<<16;}
#define XX_SDA_OUT() {GPIOD->CRL&=0XFFf0FfFF;GPIOD->CRL|=3<<16;}

void XX_Config(void);




void I2C_Config(void);
void I2C_Start(void);
void I2C_Stop(void);
u8 I2C_Wait_Ack(void);
void I2C_Ack(void);
void I2C_NAck(void);
void I2C_Send_Byte(u8 txd);
u8 I2C_Read_Byte(unsigned char ack);
# endif
