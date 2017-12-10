# ifndef __PWM_H
# define __PWM_H

# include "stm32f10x.h"
# include "sys.h"

# define M1_ENA PCout(0)
# define M1_ENB PCout(1)
# define M2_ENA PCout(3)
# define M2_ENB PCout(2)

# define DE1_A  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)

void TIM2_PWM_Config(u16 arr,u16 psc);



# endif

