/*
 * precise_clock.c
 *
 *  Created on: Aug 11, 2016
 *      Author: petera
 */

#include "system.h"

static volatile u32_t llf_counter;

// chain TIM5 to TIM8, making a 32 but CPU clock timer
// TIM5 will be clocked by TIM8 rollovers
// TIM5 is slave (LF clock) and TIM8 is master (HF clock)
// CPU clocks = (TIM5.counter << 16) | TIM8.counter
void PC_init(void) {
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  llf_counter = 0;
  // power the timers
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

  // time base configuration
  TIM_TimeBaseStructure.TIM_Period = 0xffff;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

  // reset counters
  TIM_SetCounter(TIM8, 0);
  TIM_SetCounter(TIM5, 0);
  // TIM8 is master, trigger output on update (timer rollover)
  TIM_SelectMasterSlaveMode(TIM8, TIM_MasterSlaveMode_Enable);
  TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_Update);
  // master timer must click each rollover
  TIM_SelectOnePulseMode(TIM8, TIM_OPMode_Repetitive); // todo check this
  // TIM5 is slave, and counts up at rising edges on selected trigger
  TIM_SelectSlaveMode(TIM5, TIM_SlaveMode_External1);
  // select TIM8 as trigger for TIM5
  TIM_ITRxExternalClockConfig(TIM5, TIM_TS_ITR3); // see table 86, p 408, in RM0008

  // enable interrupts for TIM5 rollovers, giving us an interrupt each 2^32:th cpu cycle
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
  // disable rollover interrupts for TIM8
  TIM_ITConfig(TIM8, TIM_IT_Update, DISABLE);
  // enable timers
  TIM_Cmd(TIM8, ENABLE);
  TIM_Cmd(TIM5, ENABLE);
}

sys_time PC_get_timer(void) {
  u16_t hf1 = TIM8->CNT;
  u16_t lf = TIM5->CNT;
  u16_t hf2 = TIM8->CNT;
  while (hf1 > hf2) {
    hf1 = hf2;
    lf = TIM5->CNT;
    hf2 = TIM8->CNT;
  }
#if CONFIG_SYS_TIME_64_BIT
  return (sys_time)(((sys_time)llf_counter << 32) | (((lf << 16) | hf2) & 0xffffffffull));
#else
  return (sys_time)((lf << 16) | hf2);
#endif
}

__attribute__(( used )) void TIM5_IRQHandler(void) {
  TRACE_IRQ_ENTER(TIM5_IRQn);
  if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    llf_counter++;
  }
  TRACE_IRQ_EXIT(TIM5_IRQn);
}


