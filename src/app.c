/*
 * app.c
 *
 *  Created on: Jan 2, 2014
 *      Author: petera
 */

#include "app.h"
#include "io.h"
#include "cli.h"
#include "taskq.h"
#include "miniutils.h"
#include "gpio.h"
#include "linker_symaccess.h"
#include "wdog.h"
#include "rtc.h"
#include "spi_flash_m25p16.h"
#include "spiffs.h"
#include "spiffs_config.h"
#include "spi_dev.h"
#include "os.h"
#include <stdarg.h>

static volatile u8_t cpu_claims;
static u8_t cli_buf[16];
volatile bool cli_rd;
static task_timer heartbeat_timer;
static spiffs fs;
volatile bool spiflash_busy;
volatile s32_t spiflash_result;
//static os_mutex spiffs_mutex;
static u32_t spiffs_stack[256];
static os_thread spiffs_thread;
static os_cond spiffs_cond;
static u8_t spiffs_fd[32*8];
static u8_t spiffs_cache[32+(32+SPIFFS_CFG_LOG_PAGE_SZ()*8)];
static u8_t spiffs_work[SPIFFS_CFG_LOG_PAGE_SZ()*2];

static void spif_spiffs_cb(spi_flash_dev *d, int res) {
  spiflash_result = res;
  spiflash_busy = FALSE;
  OS_cond_signal(&spiffs_cond);
}

static s32_t spiflash_await(void) {
  while (spiflash_busy) {
    OS_cond_wait(&spiffs_cond, NULL);
  }
  return spiflash_result;
}

static s32_t _spiffs_erase(u32_t addr, u32_t len) {
  int res;
  WDOG_feed();
  spiflash_busy = TRUE;
  if ((res = SPI_FLASH_erase(SPI_FLASH, spif_spiffs_cb, addr, len)) != SPI_OK) {
    spiflash_busy = FALSE;
  } else {
    res = spiflash_await();
  }
  return res;
}

static s32_t _spiffs_read(u32_t addr, u32_t size, u8_t *dst) {
  int res;
  spiflash_busy = TRUE;
  if ((res = SPI_FLASH_read(SPI_FLASH, spif_spiffs_cb, addr, size, dst)) != SPI_OK) {
    spiflash_busy = FALSE;
  } else {
    res = spiflash_await();
  }
  return res;
}

static s32_t _spiffs_write(u32_t addr, u32_t size, u8_t *src) {
  int res;
  spiflash_busy = TRUE;
  if ((res = SPI_FLASH_write(SPI_FLASH, spif_spiffs_cb, addr, size, src)) != SPI_OK) {
    spiflash_busy = FALSE;
  } else {
    res = spiflash_await();
  }
  return res;
}

static s32_t _spiffs_open(void) {
  int res;
  spiflash_busy = TRUE;

  if ((res = SPI_FLASH_open(SPI_FLASH, spif_spiffs_cb)) != SPI_OK) {
    spiflash_busy = FALSE;
  } else {
    res = spiflash_await();
  }
  return res;
}

static void *spiffs_mount(void *arg) {
  (void)arg;
  s32_t res;

  res = _spiffs_open();
  if (res != SPI_OK) {
    print("Could not open spiflash, err %i\n", res);
    return (void *)1;
  }

  spiffs_config cfg;
  cfg.hal_erase_f = _spiffs_erase;
  cfg.hal_read_f = _spiffs_read;
  cfg.hal_write_f = _spiffs_write;
  if ((res = SPIFFS_mount(&fs,
      &cfg,
      spiffs_work,
      spiffs_fd, sizeof(spiffs_fd),
      spiffs_cache, sizeof(spiffs_cache),
      NULL)) != SPIFFS_OK &&
      SPIFFS_errno(&fs) == SPIFFS_ERR_NOT_A_FS) {
    print("formatting spiffs...\n");
    if (SPIFFS_format(&fs) != SPIFFS_OK) {
      print("SPIFFS format failed: %i\n", SPIFFS_errno(&fs));
      return (void *)2;
    }
    print("ok\n");
    res = SPIFFS_mount(&fs,
          &cfg,
          spiffs_work,
          spiffs_fd, sizeof(spiffs_fd),
          spiffs_cache, sizeof(spiffs_cache),
          NULL);
  }
  if (res != SPIFFS_OK) {
    print("SPIFFS mount failed: %i\n", SPIFFS_errno(&fs));
    return (void *)3;
  } else {
    print("SPIFFS mounted\n");
  }
  return (void *)0;
}


static void cli_task_on_input(u32_t len, void *p) {
  u8_t io = (u8_t)((u32_t)p);
  while (IO_rx_available(io)) {
    u32_t rlen = IO_get_buf(io, cli_buf, MIN(IO_rx_available(io), sizeof(cli_buf)));
    cli_recv((char *)cli_buf, rlen);
  }
  cli_rd = FALSE;
}


static void cli_rx_avail_irq(u8_t io, void *arg, u16_t available) {
  if (!cli_rd) {
    task *t = TASK_create(cli_task_on_input, 0);
    TASK_run(t, 0, (void *)((u32_t)io));
    cli_rd = TRUE;
  }
}

static void heartbeat(u32_t ignore, void *ignore_more) {
  WDOG_feed();
}

static void sleep_stop_restore(void)
{
  // Enable HSE
  RCC_HSEConfig(RCC_HSE_ON);

  // Wait till HSE is ready
  ErrorStatus HSEStartUpStatus;
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  ASSERT(HSEStartUpStatus == SUCCESS);
  // Enable PLL
  RCC_PLLCmd(ENABLE);

  // Wait till PLL is ready
  while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
  // Select PLL as system clock source
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  // Wait till PLL is used as system clock source
  while(RCC_GetSYSCLKSource() != 0x08);
}

static void app_spin(void) {
  while (1) {
    // execute all pending tasks
    while (TASK_tick());

    bool do_sleep;
    u64_t wu_tick = 0;

    // get nearest task timer
    u64_t cur_tick = RTC_get_tick();

    sys_time taskq_wakeup_ms;
    volatile u64_t taskq_wu_tick = (u64_t)(-1ULL);
    task_timer *task_wu_timer;
    s32_t taskq_no_wakeup = TASK_next_wakeup_ms(&taskq_wakeup_ms, &task_wu_timer);
    s64_t taskq_diff_tick = 0;
    if (!taskq_no_wakeup) {
      taskq_wu_tick = RTC_MS_TO_TICK(taskq_wakeup_ms);
      taskq_diff_tick = taskq_wu_tick - cur_tick;
      ASSERT(taskq_diff_tick < 0x100000000LL);
      if (taskq_diff_tick <= 0) {
        // got at least one timer that already should've triggered: fire and loop
        TASK_timer();
        continue;
      }
    }

    // now, all task stuff should've been handled

    // get nearest thread timer
    sys_time os_wakeup_ms;
    u64_t os_wu_tick;
    os_wakeup_res os_wup_res = OS_get_next_wakeup(&os_wakeup_ms);

    bool os_sleepers = os_wup_res == OS_WUP_SLEEP_RUNNING || os_wup_res == OS_WUP_SLEEP;
    if (os_sleepers) {
      os_wu_tick = RTC_MS_TO_TICK(os_wakeup_ms);
    }

    // calculate sleep time
    do_sleep = taskq_no_wakeup && os_wup_res == OS_WUP_SLEEP_FOREVER;
    if (!do_sleep) {
      if (os_sleepers && !taskq_no_wakeup) {
        if (os_wu_tick < taskq_wu_tick) {
          wu_tick = os_wu_tick;
          DBG(D_APP, D_DEBUG, "RTC alarm @ %016llx from thread/(task_timer)\n", wu_tick);
          //print("RTC alarm @ %016llx from thread/(task_timer)\n", wu_tick);
        } else {
          wu_tick = taskq_wu_tick;
          DBG(D_APP, D_DEBUG, "RTC alarm @ %016llx from (thread)/task_timer %s\n", wu_tick, task_wu_timer->name);
          //print("RTC alarm @ %016llx from (thread)/task_timer %s\n", wu_tick, task_wu_timer->name);
        }
        wu_tick = MIN(os_wu_tick, taskq_wu_tick);
      } else if (os_sleepers) {
        wu_tick = os_wu_tick;
        DBG(D_APP, D_DEBUG, "RTC alarm @ %016llx from thread\n", wu_tick);
        //print("RTC alarm @ %016llx from thread\n", wu_tick);
      } else {
        wu_tick = taskq_wu_tick;
        DBG(D_APP, D_DEBUG, "RTC alarm @ %016llx from task_timer %s\n", taskq_wu_tick, task_wu_timer->name);
        //print("RTC alarm @ %016llx from task_timer %s\n", taskq_wu_tick, task_wu_timer->name);
      }
    }

    if (os_wup_res == OS_WUP_RUNNING || os_wup_res == OS_WUP_SLEEP_RUNNING) {
      // threads running
      OS_force_ctx_switch();
      continue;
    }

    if (do_sleep) {
      // wait forever
      RTC_cancel_alarm();
    } else {
      // wake us at timer value
      RTC_set_alarm_tick(wu_tick);
    }

    // snooze or sleep
    if (cpu_claims || (do_sleep && wu_tick - cur_tick < RTC_MS_TO_TICK(APP_PREVENT_SLEEP_IF_LESS_MS))) {
      // resources held or too soon to wake up to go sleep, just snooze
      DBG(D_APP, D_INFO, "..snoozing for %i ms, %i resources claimed\n", (u32_t)(taskq_wakeup_ms - RTC_TICK_TO_MS(RTC_get_tick())), cpu_claims);
      //print("..snoozing for %i ms, %i resources claimed\n", (u32_t)(wakeup_ms - RTC_TICK_TO_MS(RTC_get_tick())), cpu_claims);
      while (RTC_get_tick() <= wu_tick && cpu_claims && !TASK_tick()) {
        __WFI();
      }
    } else {
      // no one holding any resource, sleep
      DBG(D_APP, D_INFO, "..sleeping for %i ms\n   ", (u32_t)(RTC_TICK_TO_MS(wu_tick - RTC_get_tick())));
      irq_disable();
      IO_tx_flush(IOSTD);
      irq_enable();

      PWR_ClearFlag(PWR_FLAG_PVDO);
      PWR_ClearFlag(PWR_FLAG_WU);
      PWR_ClearFlag(PWR_FLAG_SB);

      EXTI_ClearITPendingBit(EXTI_Line17);
      RTC_ClearITPendingBit(RTC_IT_ALR);
      RTC_WaitForLastTask();

      // sleep
      PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFI);

      // wake, reconfigure
      sleep_stop_restore();

      DBG(D_APP, D_DEBUG, "awaken from sleep\n");
    }

    // check if any timers fired and insert them into task queue
    TASK_timer();
  } // while forever
}

static u8_t cli_spif_buf[256];

static void spif_cb_generic(spi_flash_dev *d, int res) {
  if (res != SPI_OK) {
    print("spiflash cb res:%i state:%i\n", res, d->state);
  } else {
    print("spiflash cb OK\n");
  }
}
static void spif_cb_rd(spi_flash_dev *d, int res) {
  spif_cb_generic(d, res);
  printbuf(IOSTD, cli_spif_buf, sizeof(cli_spif_buf));
}

void APP_init(void) {
  WDOG_start(APP_WDOG_TIMEOUT_S);

  if (PWR_GetFlagStatus(PWR_FLAG_WU) == SET) {
    PWR_ClearFlag(PWR_FLAG_WU);
    print("PWR_FLAG_WU\n");
  }
  if (PWR_GetFlagStatus(PWR_FLAG_SB) == SET) {
    PWR_ClearFlag(PWR_FLAG_SB);
    print("PWR_FLAG_SB\n");
  }
  if (PWR_GetFlagStatus(PWR_FLAG_PVDO) == SET) {
    PWR_ClearFlag(PWR_FLAG_PVDO);
    print("PWR_FLAG_PVDO\n");
  }
  if (RCC_GetFlagStatus(RCC_FLAG_SFTRST) == SET) print("SFT\n");
  if (RCC_GetFlagStatus(RCC_FLAG_PORRST) == SET) print("POR\n");
  if (RCC_GetFlagStatus(RCC_FLAG_PINRST) == SET) print("PIN\n");
  if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET) print("IWDG\n");
  if (RCC_GetFlagStatus(RCC_FLAG_LPWRRST) == SET) print("LPWR\n");
  if (RCC_GetFlagStatus(RCC_FLAG_WWDGRST) == SET) print("WWDG\n");
  RCC_ClearFlag();

  cpu_claims = 0;

  task *heatbeat_task = TASK_create(heartbeat, TASK_STATIC);
  TASK_start_timer(heatbeat_task, &heartbeat_timer, 0, 0, 0, APP_HEARTBEAT_MS, "heartbeat");

  cli_rd = FALSE;
  IO_set_callback(IOSTD, cli_rx_avail_irq, NULL);

  APP_claim(0); // TODO do not sleep

  SPI_FLASH_M25P16_app_init();
  OS_thread_create(
      &spiffs_thread,
      OS_THREAD_FLAG_PRIVILEGED,
      spiffs_mount,
      0,
      spiffs_stack,
      sizeof(spiffs_stack),
      "spiffs");

  app_spin();
}

void APP_shutdown(void) {
}

void APP_dump(void) {
  print("APP specifics\n-------------\n");
  print("\n");
}

void APP_claim(u8_t resource) {
  (void)resource;
  irq_disable();
  ASSERT(cpu_claims < 0xff);
  cpu_claims++;
  irq_enable();
}

void APP_release(u8_t resource) {
  (void)resource;
  irq_disable();
  ASSERT(cpu_claims > 0);
  cpu_claims--;
  irq_enable();
}

void APP_rtc_cb(void) {
  OS_time_tick(SYS_get_time_ms());
}


static s32_t cli_info(u32_t argc) {
  RCC_ClocksTypeDef clocks;
  print("DEV:%08x REV:%08x\n", DBGMCU_GetDEVID(), DBGMCU_GetREVID());
  RCC_GetClocksFreq(&clocks);
  print(
      "HCLK:  %i\n"
      "PCLK1: %i\n"
      "PCLK2: %i\n"
      "SYSCLK:%i\n"
      "ADCCLK:%i\n",
      clocks.HCLK_Frequency,
      clocks.PCLK1_Frequency,
      clocks.PCLK2_Frequency,
      clocks.SYSCLK_Frequency,
      clocks.ADCCLK_Frequency);
  print(
      "RTCCLK:%i\n"
      "RTCDIV:%i\n"
      "RTCTCK:%i ticks/sec\n",
      CONFIG_RTC_CLOCK_HZ,
      CONFIG_RTC_PRESCALER,
      CONFIG_RTC_CLOCK_HZ/CONFIG_RTC_PRESCALER
      );

  print(
      "RTCCNT:%016llx\n"
      "RTCALR:%016llx\n",
      RTC_get_tick(), RTC_get_alarm_tick());
  return CLI_OK;
}

static s32_t cli_spif_rd(u32_t argc, u32_t addr, u32_t len) {
  len = MAX(len, sizeof(cli_spif_buf));
  return SPI_FLASH_read(SPI_FLASH, spif_cb_rd, addr, len, cli_spif_buf);
}

static s32_t cli_spif_chip_er(u32_t argc) {
  return SPI_FLASH_mass_erase(SPI_FLASH, spif_cb_generic);
}

#define TEST_THREAD
#include "os.h"
static u32_t stack[256];
#ifdef TEST_THREAD
os_thread t_thread;
static void *tt_func(void *arg) {
  int t = 10;
  while (1) {
    while (--t) {
      print("test thread arg %08x #%i\n",arg, t);
      OS_thread_sleep((t+1)*100);
    }
    t = 10;
  }
  return NULL;
}
static int cli_test_thread(u32_t argc) {
  OS_thread_create(
      &t_thread,
      OS_THREAD_FLAG_PRIVILEGED,
      tt_func,
      (void *)0x12345678,
      stack,
      sizeof(stack),
      "test_thread");
  return 0;
}
#endif


CLI_EXTERN_MENU(common)

CLI_MENU_START_MAIN
CLI_EXTRAMENU(common)
CLI_FUNC("spi_rd", cli_spif_rd, "Reads spi flash memory\nspi_rd <addr> <len>\n")
CLI_FUNC("spi_chip_er", cli_spif_chip_er, "Erase all spi flash memory\n")
#ifdef TEST_THREAD
CLI_FUNC("thr", cli_test_thread, "Tests a thread\n")
#endif
CLI_FUNC("info", cli_info, "Prints system info")
CLI_FUNC("help", cli_help, "Prints help")
CLI_MENU_END
