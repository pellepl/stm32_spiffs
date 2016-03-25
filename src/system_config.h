/*
 * system_config.h
 *
 *  Created on: Jul 24, 2012
 *      Author: petera
 */

#ifndef SYSTEM_CONFIG_H_
#define SYSTEM_CONFIG_H_

#include "config_header.h"
#include "types.h"
#include "stm32f10x.h"


#define APP_NAME "WISLEEP"

/****************************************/
/***** Functionality block settings *****/
/****************************************/


// enable uart
#define CONFIG_UART1
#define CONFIG_UART2
#define CONFIG_UART_CNT   2 // update according to enabled uarts

#define CONFIG_SPI1

/*********************************************/
/***** Hardware build time configuration *****/
/*********************************************/

/** Processor specifics **/

#ifndef USER_HARDFAULT
// enable user hardfault handler
#define USER_HARDFAULT 1
#endif

// hardware debug (blinking leds etc)
#define HW_DEBUG


/** General **/

// internal flash start address
#define FLASH_START       FLASH_BASE
// internal flash page erase size
#define FLASH_PAGE_SIZE   0x400 // md
// internal flash protection/unprotection for firmware
#define FLASH_PROTECT     FLASH_WRProt_AllPages
// internal flash total size in bytes
#define FLASH_TOTAL_SIZE  (128*1024) // md

/** UART **/

#ifdef CONFIG_UART2
#define UART2_GPIO_PORT       GPIOA
#define UART2_GPIO_RX         GPIO_Pin_3
#define UART2_GPIO_TX         GPIO_Pin_2
#endif
#ifdef CONFIG_UART1
#define UART1_GPIO_PORT       GPIOA
#define UART1_GPIO_RX         GPIO_Pin_10
#define UART1_GPIO_TX         GPIO_Pin_9
#endif

/** SPI **/

#ifdef CONFIG_SPI

// make SPI driver use polling method, otherwise DMA requests are used
// warning - polling method should only be used for debugging and may be
// unstable. Do not sent multitudes of data using this config
//#define CONFIG_SPI_POLL

// on some stm32f1s it seems to be a bad idea closing down the SPI between
// operations
#define CONFIG_SPI_KEEP_RUNNING

#ifdef CONFIG_SPI1

#define SPI1_MASTER_GPIO              GPIOA
#define SPI1_MASTER_GPIO_CLK          RCC_APB2Periph_GPIOA
#define SPI1_MASTER_PIN_SCK           GPIO_Pin_5
#define SPI1_MASTER_PIN_MISO          GPIO_Pin_6
#define SPI1_MASTER_PIN_MOSI          GPIO_Pin_7

#define SPI1_MASTER                   SPI1
#define SPI1_MASTER_BASE              SPI1_BASE
#define SPI1_MASTER_CLK               RCC_APB2Periph_SPI1
#define SPI1_MASTER_DMA               DMA1
#define SPI1_MASTER_DMA_CLK           RCC_AHBPeriph_DMA1
// according to userguide table 78
#define SPI1_MASTER_Rx_DMA_Channel    DMA1_Channel2
#define SPI1_MASTER_Tx_DMA_Channel    DMA1_Channel3
#define SPI1_MASTER_Rx_IRQ_Channel    DMA1_Channel2_IRQn
#define SPI1_MASTER_Tx_IRQ_Channel    DMA1_Channel3_IRQn

#endif // CONFIG_SPI1

#ifdef CONFIG_SPI2

#define SPI2_MASTER_GPIO              GPIOB
#define SPI2_MASTER_GPIO_CLK          RCC_APB2Periph_GPIOB
#define SPI2_MASTER_PIN_SCK           GPIO_Pin_13
#define SPI2_MASTER_PIN_MISO          GPIO_Pin_14
#define SPI2_MASTER_PIN_MOSI          GPIO_Pin_15

#define SPI2_MASTER                   SPI2
#define SPI2_MASTER_BASE              SPI2_BASE
#define SPI2_MASTER_CLK               RCC_APB1Periph_SPI2
#define SPI2_MASTER_DMA               DMA1
#define SPI2_MASTER_DMA_CLK           RCC_AHBPeriph_DMA1
// according to userguide table 78
#define SPI2_MASTER_Rx_DMA_Channel    DMA1_Channel4
#define SPI2_MASTER_Tx_DMA_Channel    DMA1_Channel5
#define SPI2_MASTER_Rx_IRQ_Channel    DMA1_Channel4_IRQn
#define SPI2_MASTER_Tx_IRQ_Channel    DMA1_Channel5_IRQn

#endif // CONFIG_SPI2

#endif // CONFIG_SPI

/** I2C **/

#ifdef CONFIG_I2C

#define I2C1_CLK                      RCC_APB1Periph_I2C2
#define I2C1_PORT                     I2C2

#define I2C_MAX_ID                    1

#endif


/****************************************************/
/******** Application build time configuration ******/
/****************************************************/

/** TICKER **/
// STM32 system timer
#define CONFIG_STM32_SYSTEM_TIMER   2
// system timer frequency
#define SYS_MAIN_TIMER_FREQ   40000
// system timer counter type
typedef u16_t system_counter_type;
// system tick frequency
#define SYS_TIMER_TICK_FREQ   1000
// os ticker cpu clock div
#define SYS_OS_TICK_DIV       8

/** ARCH **/

#define CONFIG_ARCH_CRITICAL_DISABLE_IRQ \
  do { \
    __set_BASEPRI(0x02<<(8-__NVIC_PRIO_BITS)); \
  } while (0)

#define CONFIG_ARCH_CRITICAL_ENABLE_IRQ \
  do { \
    __set_BASEPRI(0x00); \
} while (0)

/** UART **/

#define UARTSTDIN       1
#define UARTSTDOUT      1
#define UARTWIFIIN      0
#define UARTWIFIOUT     0

#define UART2_SPEED 460800
#define UART1_SPEED 921600

#define USE_COLOR_CODING

/** IO **/
#define CONFIG_IO_MAX   2

#define IOSTD        1
#define IODBG        IOSTD
#define IOWIFI       0

/** MATH **/

#define CONFIG_TRIGQ_TABLE

/** TASK KERNEL **/

#define CONFIG_TASK_POOL 32
//#define CONFIG_TASK_NONCRITICAL_TIMER
//#define CONFIG_TASKQ_DBG_CRITICAL
#define CONFIG_TASKQ_MUTEX

/** APP **/

#define WS2812B_NBR_OF_LEDS 16
#define CONFIG_RTC_CLOCK_HZ 32768
#define CONFIG_RTC_PRESCALER 32


/** DEBUG **/

#define DBG_ATTRIBUTE __attribute__(( section(".dbg") ))

// disable all asserts
//#define ASSERT_OFF

// disable all debug output
//#define DBG_OFF

#define CONFIG_DEFAULT_DEBUG_MASK     (0xffffffff)
#define DBG_MS_PREFIX          1

// enable or disable tracing
//#define DBG_TRACE_MON
#define TRACE_SIZE            (64)

#define VALID_RAM(x) \
  (((void*)(x) >= RAM_BEGIN && (void*)(x) < RAM_END))

#define VALID_FLASH(x) \
  ((void*)(x) >= (void*)FLASH_BEGIN && (void*)(x) < (void*)(FLASH_END))

#define VALID_DATA(x) \
  (VALID_RAM(x) || VALID_FLASH(x))

#define OS_DBG_BADR(x) \
    (!VALID_RAM(x))


#endif /* SYSTEM_CONFIG_H_ */
