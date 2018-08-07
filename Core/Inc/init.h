#ifndef __INIT_H__
#define __INIT_H__

#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_dma.h"

//void  Configure_TIMPWMOutput(void);
void  Configure_DMA(void);
void Configure_TIM_PWM_DMA(void);

#endif
