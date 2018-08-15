#ifndef __INIT_H__
#define __INIT_H__

#include "math.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_dma.h"

#define CPU_Freq 32000000
//SystemCoreClock
//32000000	// частота ядра микроконтроллера
#define PWM_Freq 100000   	// частота модуляции
#define MOD_Freq 50      	// частота переменного тока

//#define steps		(PWM_Freq/MOD_Freq/2)
#define steps		(PWM_Freq/MOD_Freq)
#define precision	(CPU_Freq/PWM_Freq/2)
#define pi		3.1415926535f

//uint16_t sin_ar[steps];

//void  Configure_TIMPWMOutput(void);
void  Configure_DMA(void);
void Configure_TIM_PWM_DMA(void);
void  Configure_TIM_test1(void);

void fill_sine();

#endif
