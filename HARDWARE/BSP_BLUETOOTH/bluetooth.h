# ifndef __BT_H
# define __BT_H
# include "sys.h"

# define BT_CTR			PAout(12)
# define BT_STATE		PCin(6)

void BSP_BT_Config(u32 bound);
void BSP_BT_SetBoundRate(u32 bound);
void BSP_BT_SetName(const uint8_t * name);
void BSP_BT_SetRole(uint8_t role);
void BSP_BT_SetPin(uint8_t pin);
void BSP_BT_Reset(void);

# endif

