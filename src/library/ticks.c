#include "ticks.h"

volatile u16 msec = 0;
volatile u16 sec = 0;

/**
  * @brief  Initialization of ticks timer
  * @param  None
  * @retval None
  */
void ticks_init(void) {
	//Set TIM2 RCC
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	//Set TIM2 to count at 1KHz
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 1000;
	TIM_TimeBaseStructure.TIM_Prescaler = 71;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
	
	//Turn on TIM2 Interrupt
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	msec = sec = 0;
}

/**
  * @brief  Ticks timer interrupt handler
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void) {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == RESET) return;
	TIM_ClearFlag(TIM2, TIM_IT_Update);
	if (++msec >= 1000) {
		msec = 0;
		sec++;
	}
}

/**
  * @brief  Reset the ticks timer
  * @param  None
  * @retval None
  */
void ticks_reset(void) {
	msec = sec = 0;
}

/**
  * @brief  Return no of seconds passed
  * @param  None
  * @retval Seconds passed
  */
u16 get_second_ticks(void) {
	return sec;
}

/**
  * @brief  Return no of milliseconds passed
  * @param  None
  * @retval Milliseconds passed
  */
u32 get_real_ticks(void) {
	return sec*1000+msec;
}
