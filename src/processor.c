/*
 * processor.c
 *
 *  Created on: Aug 1, 2012
 *      Author: petera
 */

#include "processor.h"
#include "system.h"
#include "gpio.h"
#include "app.h"

static void RCC_config() {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

#ifdef CONFIG_UART1
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#endif
#ifdef CONFIG_UART2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
#endif
#ifdef CONFIG_UART3
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
#endif
#ifdef CONFIG_UART4
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
#endif

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  RCC_APB1PeriphClockCmd(STM32_SYSTEM_TIMER_RCC, ENABLE);

  RCC_PCLK1Config(RCC_HCLK_Div2); // APB1 = HCLK/2
  RCC_PCLK2Config(RCC_HCLK_Div1); // APB2 = HCLK/1


#ifdef CONFIG_SPI
#ifdef CONFIG_SPI1
  /* Enable SPI1_MASTER clock and GPIO clock for SPI1_MASTER */
  RCC_APB2PeriphClockCmd(SPI1_MASTER_GPIO_CLK, ENABLE);
  RCC_APB2PeriphClockCmd(SPI1_MASTER_CLK, ENABLE);

  /* Enable SPI1_MASTER DMA clock */
  RCC_AHBPeriphClockCmd(SPI1_MASTER_DMA_CLK, ENABLE);
#endif
#endif

#ifdef CONFIG_I2C
  RCC_APB1PeriphClockCmd(I2C1_CLK, ENABLE);
#endif
}

static void NVIC_config(void)
{
  // STM32 7 6 5 4 3 2 1 0
  //       I I I I X X X X
  //
  // priogrp 7 =>
  // STM32 7 6 5 4 3 2 1 0
  //       P S S S X X X X
  // preempt prio 0..1
  // subprio      0..7

  // Configure the NVIC Preemption Priority Bits
  // use 1 bit for preemption and 3 bits for subgroups
  u8_t prioGrp = 6;

  // Config pendsv interrupt, lowest
  NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(prioGrp, 1, 7));

  // Config & enable uarts interrupt
#ifdef CONFIG_UART2
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(prioGrp, 1, 6));
  NVIC_EnableIRQ(USART2_IRQn);
#endif
#ifdef CONFIG_UART1
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(prioGrp, 1, 6));
  NVIC_EnableIRQ(USART1_IRQn);
#endif

#ifdef CONFIG_SPI
  // Config & enable the SPI-DMA interrupt
#ifdef CONFIG_SPI1
  NVIC_SetPriority(SPI1_MASTER_Rx_IRQ_Channel, NVIC_EncodePriority(prioGrp, 1, 6));
  NVIC_EnableIRQ(SPI1_MASTER_Rx_IRQ_Channel);
  NVIC_SetPriority(SPI1_MASTER_Tx_IRQ_Channel, NVIC_EncodePriority(prioGrp, 1, 6));
  NVIC_EnableIRQ(SPI1_MASTER_Tx_IRQ_Channel);
#endif
#endif

#ifdef CONFIG_I2C
  NVIC_SetPriority(I2C2_EV_IRQn, NVIC_EncodePriority(prioGrp, 1, 6));
  NVIC_EnableIRQ(I2C2_EV_IRQn);
  NVIC_SetPriority(I2C2_ER_IRQn, NVIC_EncodePriority(prioGrp, 1, 6));
  NVIC_EnableIRQ(I2C2_ER_IRQn);
#endif

#ifdef CONFIG_RTC
  NVIC_SetPriority(RTCAlarm_IRQn, NVIC_EncodePriority(prioGrp, 0, 1));
  NVIC_EnableIRQ(RTCAlarm_IRQn);
#endif

#ifdef CONFIG_OS
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(prioGrp, 1, 6));
#endif
}

static void UART1_config() {
#ifdef CONFIG_UART1
  gpio_config(PORTA, PIN9, CLK_50MHZ, AF, AF0, PUSHPULL, NOPULL);
  gpio_config(PORTA, PIN10, CLK_50MHZ, IN, AF0, OPENDRAIN, NOPULL);
#endif
}
static void UART2_config() {
#ifdef CONFIG_UART2
  gpio_config(PORTA, PIN2, CLK_50MHZ, AF, AF0, PUSHPULL, NOPULL);
  gpio_config(PORTA, PIN3, CLK_50MHZ, IN, AF0, OPENDRAIN, NOPULL);
#endif
}

static void TIM_config(void) {
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  u16_t prescaler = 0;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = SYS_CPU_FREQ/SYS_MAIN_TIMER_FREQ;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(STM32_SYSTEM_TIMER, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(STM32_SYSTEM_TIMER, prescaler, TIM_PSCReloadMode_Immediate);

  /* TIM IT enable */
  TIM_ITConfig(STM32_SYSTEM_TIMER, TIM_IT_Update, ENABLE);

  /* TIM enable counter */
  TIM_Cmd(STM32_SYSTEM_TIMER, ENABLE);
}

static void I2C_config(void) {
#ifdef CONFIG_I2C
  gpio_config(PORTB, PIN10, CLK_50MHZ, AF, AF0, OPENDRAIN, NOPULL);
  gpio_config(PORTB, PIN11, CLK_50MHZ, AF, AF0, OPENDRAIN, NOPULL);
#endif
}

static void SPI_config() {
#ifdef CONFIG_SPI
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
  DMA_InitTypeDef  DMA_InitStructure;
#ifdef CONFIG_SPI1
  // SPI1

  /* Configure SPI1_MASTER pins: NSS, SCK and MOSI */
  GPIO_InitStructure.GPIO_Pin = SPI1_MASTER_PIN_SCK | SPI1_MASTER_PIN_MOSI | SPI1_MASTER_PIN_MISO;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(SPI1_MASTER_GPIO, &GPIO_InitStructure);

  /* SPI1_MASTER configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // APB2/64
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1_MASTER, &SPI_InitStructure);

#ifndef CONFIG_SPI_POLL
  /* Configure SPI DMA common */
  // SPI1_BASE(APB2PERIPH_BASE(PERIPH_BASE(0x40000000) + 0x00010000) + 3000)
  // DataRegister offset = 0x0c = SPI1_BASE + 0x0c
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(SPI1_MASTER_BASE + 0x0c);
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

  /* Configure SPI DMA rx */
  DMA_DeInit(SPI1_MASTER_Rx_DMA_Channel);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 0;
  DMA_Init(SPI1_MASTER_Rx_DMA_Channel, &DMA_InitStructure);

  /* Configure SPI DMA tx */
  DMA_DeInit(SPI1_MASTER_Tx_DMA_Channel);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 0;
  DMA_Init(SPI1_MASTER_Tx_DMA_Channel, &DMA_InitStructure);

  /* Enable DMA SPI RX channel transfer complete interrupt */
  DMA_ITConfig(SPI1_MASTER_Rx_DMA_Channel, DMA_IT_TC | DMA_IT_TE, ENABLE);
  // Do not enable DMA SPI TX channel transfer complete interrupt,
  // always use tx/rx transfers and only await DMA RX finished irq
  DMA_ITConfig(SPI1_MASTER_Tx_DMA_Channel, DMA_IT_TE, ENABLE);

  /* Enable SPI_MASTER DMA Rx/Tx request */
  SPI_I2S_DMACmd(SPI1_MASTER, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx , ENABLE);
#endif // CONFIG_SPI_POLL
#endif //CONFIG_SPI1

#ifdef CONFIG_SPI2
  // SPI2

  /* Configure SPI2_MASTER pins: NSS, SCK and MOSI */
  GPIO_InitStructure.GPIO_Pin = SPI2_MASTER_PIN_SCK | SPI2_MASTER_PIN_MOSI | SPI2_MASTER_PIN_MISO;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(SPI2_MASTER_GPIO, &GPIO_InitStructure);

  /* SPI2_MASTER configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // APB2/64
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2_MASTER, &SPI_InitStructure);

#ifndef CONFIG_SPI_POLL
  /* Configure SPI DMA common */
  // SPI1_BASE(APB2PERIPH_BASE(PERIPH_BASE(0x40000000) + 0x00010000) + 3000)
  // DataRegister offset = 0x0c = SPI1_BASE + 0x0c
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(SPI2_MASTER_BASE + 0x0c);
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

  /* Configure SPI2 DMA rx */
  DMA_DeInit(SPI2_MASTER_Rx_DMA_Channel);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 0;
  DMA_Init(SPI2_MASTER_Rx_DMA_Channel, &DMA_InitStructure);

  /* Configure SPI DMA tx */
  DMA_DeInit(SPI2_MASTER_Tx_DMA_Channel);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 0;
  DMA_Init(SPI2_MASTER_Tx_DMA_Channel, &DMA_InitStructure);

  /* Enable DMA SPI RX channel transfer complete interrupt */
  DMA_ITConfig(SPI2_MASTER_Rx_DMA_Channel, DMA_IT_TC | DMA_IT_TE, ENABLE);
  // Do not enable DMA SPI TX channel transfer complete interrupt,
  // always use tx/rx transfers and only await DMA RX finished irq
  DMA_ITConfig(SPI2_MASTER_Tx_DMA_Channel, DMA_IT_TE, ENABLE);

  /* Enable SPI_MASTER DMA Rx/Tx request */
  SPI_I2S_DMACmd(SPI2_MASTER, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx , ENABLE);
#endif // CONFIG_SPI_POLL
#endif //CONFIG_SPI2
#endif // CONFIG_SPI
}


void PROC_base_init() {
  RCC_config();
  NVIC_config();
  TIM_config();
}

void PROC_periph_init() {
  DBGMCU_Config(STM32_SYSTEM_TIMER_DBGMCU, ENABLE);

  SPI_config();
  I2C_config();
  UART1_config();
  UART2_config();

#ifdef CONFIG_SPI
  GPIO_InitTypeDef GPIO_InitStructure;
  // spiflash config CS pin
  GPIO_InitStructure.GPIO_Pin = SPI_FLASH_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(SPI_FLASH_GPIO_PORT, &GPIO_InitStructure);
#endif
}

