#include "init.h"


#define CC_VALUE_NB       10
static uint32_t aCCValue[CC_VALUE_NB] = {0};


static uint32_t TimOutClock = 1;

void Configure_TIMPWMOutput_int(void) {
	LL_TIM_InitTypeDef tim_initstruct;
	LL_TIM_OC_InitTypeDef tim_oc_initstruct;

	/*************************/
	/* GPIO AF configuration */
	/*************************/
	/* Enable the peripheral clock of GPIOs */
	//LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

//  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7|LL_GPIO_PIN_8|LL_GPIO_PIN_9;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* GPIO TIM4_CH1 configuration */
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_6, LL_GPIO_PULL_DOWN);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);

	/***********************************************/
	/* Configure the NVIC to handle TIM4 interrupt */
	/***********************************************/
	NVIC_SetPriority(TIM4_IRQn, 0);
	NVIC_EnableIRQ(TIM4_IRQn);

	/******************************/
	/* Peripheral clocks enabling */
	/******************************/
	/* Enable the timer peripheral clock */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

	/***************************/
	/* Time base configuration */
	/***************************/
	/* Set fields of initialization structure */
	/* - Set the pre-scaler value to have TIM4 counter clock equal to 10 kHz  */
	/* - Set the auto-reload value to have a counter frequency of 100 Hz        */
	/* TIM4CLK = SystemCoreClock / (APB prescaler & multiplier)               */
	TimOutClock = SystemCoreClock / 1;

	tim_initstruct.Prescaler = __LL_TIM_CALC_PSC(SystemCoreClock, 10000);
	tim_initstruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	tim_initstruct.Autoreload = __LL_TIM_CALC_ARR(TimOutClock,
			tim_initstruct.Prescaler, 100);
	tim_initstruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	tim_initstruct.RepetitionCounter = (uint8_t) 0x00;

	/* Initialize TIM instance according to parameters defined in               */
	/* initialization structure.                                                */
	LL_TIM_Init(TIM4, &tim_initstruct);

	/* Enable TIM4_ARR register preload. Writing to or reading from the         */
	/* auto-reload register accesses the preload register. The content of the   */
	/* preload register are transferred into the shadow register at each update */
	/* event (UEV).                                                             */
	LL_TIM_EnableARRPreload(TIM4);

	/*********************************/
	/* Output waveform configuration */
	/*********************************/
	/* Set fields of initialization structure */
	/*  - Set compare value to half of the counter period (50% duty cycle ) */
	tim_oc_initstruct.OCMode = LL_TIM_OCMODE_PWM1;
	tim_oc_initstruct.OCState = LL_TIM_OCSTATE_DISABLE;
	tim_oc_initstruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	tim_oc_initstruct.CompareValue = ((LL_TIM_GetAutoReload(TIM4) + 1) / 2);
	tim_oc_initstruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	tim_oc_initstruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
	tim_oc_initstruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
	tim_oc_initstruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;

	/* Initialize TIM instance according to parameters defined in               */
	/* initialization structure.                                                */
	LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &tim_oc_initstruct);

	/* Enable TIM4_CCR1 register preload. Read/Write operations access the      */
	/* preload register. TIM4_CCR1 preload value is loaded in the active        */
	/* at each update event.                                                    */
	LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);

	/**************************/
	/* TIM4 interrupts set-up */
	/**************************/
	/* Enable the capture/compare interrupt for channel 1*/
	LL_TIM_EnableIT_CC1(TIM4);

	/**********************************/
	/* Start output signal generation */
	/**********************************/
	/* Enable output channel 1 */
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);

	/* Enable counter */
	LL_TIM_EnableCounter(TIM4);

	/* Force update generation */
	LL_TIM_GenerateEvent_UPDATE(TIM4);
}

void Configure_TIMPWMOutput_4ch(void) {
	/*************************/
	/* GPIO AF configuration */
	/*************************/
	/* Enable the peripheral clock of GPIOs */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

	LL_GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | LL_GPIO_PIN_8	| LL_GPIO_PIN_9;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* GPIO TIM2_CH1 configuration */
	//LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9
/*
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_6, LL_GPIO_PULL_DOWN);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);
*/

/*	  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
	 	  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_7, LL_GPIO_PULL_DOWN);
	 	  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_HIGH);*/

	/***********************************************/
	/* Configure the NVIC to handle TIM2 interrupt */
	/***********************************************/
	NVIC_SetPriority(TIM4_IRQn, 0);
	NVIC_EnableIRQ(TIM4_IRQn);

	/******************************/
	/* Peripheral clocks enabling */
	/******************************/
	/* Enable the timer peripheral clock */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

	/***************************/
	/* Time base configuration */
	/***************************/
	/* Set counter mode */
	/* Reset value is LL_TIM_COUNTERMODE_UP */
	//LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
	/* Set the pre-scaler value to have TIM2 counter clock equal to 10 kHz */
	LL_TIM_SetPrescaler(TIM4, __LL_TIM_CALC_PSC(SystemCoreClock, 10000));

	/* Enable TIM2_ARR register preload. Writing to or reading from the         */
	/* auto-reload register accesses the preload register. The content of the   */
	/* preload register are transferred into the shadow register at each update */
	/* event (UEV).                                                             */
	LL_TIM_EnableARRPreload(TIM4);

	/* Set the auto-reload value to have a counter frequency of 100 Hz */
	/* TIM2CLK = SystemCoreClock / (APB prescaler & multiplier)               */
	TimOutClock = SystemCoreClock / 1;
	LL_TIM_SetAutoReload(TIM4,
			__LL_TIM_CALC_ARR(TimOutClock, LL_TIM_GetPrescaler(TIM4), 100));

	/*********************************/
	/* Output waveform configuration */
	/*********************************/
	/* Set output mode */
	/* Reset value is LL_TIM_OCMODE_FROZEN */
	LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);

	/* Set output channel polarity */
	/* Reset value is LL_TIM_OCPOLARITY_HIGH */
	//LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
	/* Set compare value to half of the counter period (50% duty cycle ) */
	LL_TIM_OC_SetCompareCH1(TIM4, ((LL_TIM_GetAutoReload(TIM4) + 1) / 2));
	LL_TIM_OC_SetCompareCH2(TIM4, ((LL_TIM_GetAutoReload(TIM4) + 1) / 4));
	LL_TIM_OC_SetCompareCH3(TIM4, ((LL_TIM_GetAutoReload(TIM4) + 1) / 6));
	LL_TIM_OC_SetCompareCH4(TIM4, ((LL_TIM_GetAutoReload(TIM4) + 1) / 8));

	/* Enable TIM2_CCR1 register preload. Read/Write operations access the      */
	/* preload register. TIM2_CCR1 preload value is loaded in the active        */
	/* at each update event.                                                    */
	LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH2);
	LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH3);
	LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH4);

	/**************************/
	/* TIM2 interrupts set-up */
	/**************************/
	/* Enable the capture/compare interrupt for channel 1*/
	LL_TIM_EnableIT_CC1(TIM4);

	/**********************************/
	/* Start output signal generation */
	/**********************************/
	/* Enable output channel 1 */
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH4);

	/* Enable counter */
	LL_TIM_EnableCounter(TIM4);

	/* Force update generation */
	LL_TIM_GenerateEvent_UPDATE(TIM4);
}


void  Configure_DMA(void)
{
  /******************************************************/
  /* Configure NVIC for DMA transfer related interrupts */
  /******************************************************/
  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  /*****************************/
  /* Peripheral clock enabling */
  /*****************************/
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /******************************/
  /* DMA transfer Configuration */
  /******************************/
  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
                                                LL_DMA_PRIORITY_HIGH              |
                                                LL_DMA_MODE_CIRCULAR              |
                                                LL_DMA_PERIPH_NOINCREMENT         |
                                                LL_DMA_MEMORY_INCREMENT           |
                                                LL_DMA_PDATAALIGN_WORD            |
                                                LL_DMA_MDATAALIGN_WORD);

  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&aCCValue, (uint32_t)&TIM4->CCR1, LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, CC_VALUE_NB);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);

  /***************************/
  /* Enable the DMA transfer */
  /***************************/
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}

void Configure_TIM_PWM_DMA(void)
{
  /*************************/
  /* GPIO AF configuration */
  /*************************/
  /* Enable the peripheral clock of GPIOs */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /* GPIO TIM4_CH3 configuration */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_6, LL_GPIO_PULL_DOWN);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);

  /******************************************************/
  /* Configure the NVIC to handle TIM4 update interrupt */
  /******************************************************/
  NVIC_SetPriority(TIM4_IRQn, 0);
  NVIC_EnableIRQ(TIM4_IRQn);

  /******************************/
  /* Peripheral clocks enabling */
  /******************************/
  /* Enable the peripheral clock of TIM4 */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

  /***************************/
  /* Time base configuration */
  /***************************/
  /* Set counter mode */
  /* Reset value is LL_TIM_COUNTERMODE_UP */
  //LL_TIM_SetCounterMode(TIM4, LL_TIM_COUNTERMODE_UP);

  /* Set the TIM4 auto-reload register to get a PWM frequency at 17.57 KHz */
  /* Note that the timer pre-scaler isn't used, therefore the timer counter   */
  /* clock frequency is equal to the timer frequency.                        */
    /* In this example TIM4 input clock (TIM4CLK) frequency is set to APB1 clock*/
  /*  (PCLK1), since APB1 pre-scaler is equal to 2 and it is twice PCLK2.                        */
  /*    TIM4CLK = PCLK2                                                     */
  /*    PCLK2 = HCLK                                                        */
  /*    => TIM4CLK = HCLK = SystemCoreClock (72 Mhz)                       */

  /* TIM4CLK = SystemCoreClock / (APB prescaler & multiplier)              */
  TimOutClock = SystemCoreClock/1;
  LL_TIM_SetAutoReload(TIM4, __LL_TIM_CALC_ARR(TimOutClock, LL_TIM_COUNTERMODE_UP, 100));

  /* Set the repetition counter in order to generate one update event every 4 */
  /* counter cycles.                                                          */
  LL_TIM_SetRepetitionCounter(TIM4, 10-1);

  /*********************************/
  /* Output waveform configuration */
  /*********************************/
  /* Set output channel 3 in PWM1 mode */
  LL_TIM_OC_SetMode(TIM4,  LL_TIM_CHANNEL_CH1,  LL_TIM_OCMODE_PWM1);

  /* TIM4 channel 3 configuration:    */
  LL_TIM_OC_ConfigOutput(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH | LL_TIM_OCIDLESTATE_HIGH);

  /* Compute compare value to generate a duty cycle at 90% - 0% */
  aCCValue[0] = (uint32_t)(((uint32_t) 90 * (LL_TIM_GetAutoReload(TIM4) - 1)) / 100);
  aCCValue[1] = (uint32_t)(((uint32_t) 80 * (LL_TIM_GetAutoReload(TIM4) - 1)) / 100);
  aCCValue[2] = (uint32_t)(((uint32_t) 70 * (LL_TIM_GetAutoReload(TIM4) - 1)) / 100);
  aCCValue[3] = (uint32_t)(((uint32_t) 60 * (LL_TIM_GetAutoReload(TIM4) - 1)) / 100);
  aCCValue[4] = (uint32_t)(((uint32_t) 50 * (LL_TIM_GetAutoReload(TIM4) - 1)) / 100);
  aCCValue[5] = (uint32_t)(((uint32_t) 40 * (LL_TIM_GetAutoReload(TIM4) - 1)) / 100);
  aCCValue[6] = (uint32_t)(((uint32_t) 30 * (LL_TIM_GetAutoReload(TIM4) - 1)) / 100);
  aCCValue[7] = (uint32_t)(((uint32_t) 20 * (LL_TIM_GetAutoReload(TIM4) - 1)) / 100);
  aCCValue[8] = (uint32_t)(((uint32_t) 10 * (LL_TIM_GetAutoReload(TIM4) - 1)) / 100);
  aCCValue[9] = (uint32_t)(((uint32_t) 0 * (LL_TIM_GetAutoReload(TIM4) - 1)) / 100);

  /* Set PWM duty cycle  for TIM4 channel 3*/
  LL_TIM_OC_SetCompareCH3(TIM4, aCCValue[0]);

  /* Enable register preload for TIM4 channel 3 */
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);

  /****************************/
  /* TIM4 DMA requests set-up */
  /****************************/
  /* Enable DMA request on update event */
  LL_TIM_EnableDMAReq_UPDATE(TIM4);

  /* Enable TIM4 Channel 3 DMA request */
  LL_TIM_EnableDMAReq_CC1(TIM4);

  /**********************************/
  /* Start output signal generation */
  /**********************************/
  /* Enable TIM4 channel 3 */
  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);

  /* Enable TIM4 outputs */
  LL_TIM_EnableAllOutputs(TIM4);

  /* Enable counter */
  LL_TIM_EnableCounter(TIM4);

  /* Force update generation */
  LL_TIM_GenerateEvent_UPDATE(TIM4);
}
