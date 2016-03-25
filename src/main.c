#include "stm32f10x.h"
#include "system.h"
#include "uart_driver.h"
#include "io.h"
#include "timer.h"
#include "miniutils.h"
#include "taskq.h"
#include "processor.h"
#include "linker_symaccess.h"
#include "app.h"
#include "gpio.h"
#include "shared_mem.h"
#ifdef CONFIG_SPI
#include "spi_driver.h"
#endif
#ifdef CONFIG_I2C
#include "i2c_driver.h"
#endif
#ifdef CONFIG_WDOG
#include "wdog.h"
#endif
#ifdef CONFIG_RTC
#include "rtc.h"
#endif

#include "cli.h"

static void app_assert_cb(void) {
  APP_shutdown();
}

// main entry from bootstrap
int main(void) {
  enter_critical();
  PROC_base_init();
  SYS_init();
  UART_init();
  UART_assure_tx(_UART(UARTSTDOUT), TRUE);
  PROC_periph_init();
  exit_critical();

  SYS_set_assert_callback(app_assert_cb);

  IO_define(IOSTD, io_uart, UARTSTDIN);

#ifdef CONFIG_SPI
  SPI_init();
#endif
#ifdef CONFIG_I2C
  I2C_init();
#endif
#ifdef CONFIG_WDOG
  WDOG_init();
#endif
#ifdef CONFIG_RTC
  if (RTC_init(NULL)) { // TODO
    rtc_datetime dt = {
        .date.year = 2016,
        .date.month = 0,
        .date.month_day = 1,
        .time.hour = 12,
        .time.minute = 0,
        .time.second = 0,
        .time.millisecond = 0
    };
    RTC_set_date_time(&dt);
  }
#endif

  print("\n\n\nHardware initialization done\n");

  print("Stack 0x%08x -- 0x%08x\n", STACK_START, STACK_END);

  print("Subsystem initialization done\n");

  TASK_init();

  cli_init();

  print ("\n");
  print(APP_NAME);
  print("\n\n");
  print("build     : %i\n", SYS_build_number());
  print("build date: %i\n", SYS_build_date());

  print("reset reason: ");
  if (SHMEM_validate()) {
    print("%i\n", SHMEM_get()->reboot_reason);
  } else {
    print("cold start\n");
    SYS_dbg_level(D_WARN);
    SYS_dbg_mask_set(0);
  }
  SHMEM_set_reboot_reason(REBOOT_UNKNOWN);

  rand_seed(0xd0decaed ^ SYS_get_tick());

  APP_init();

  return 0;
}

// assert failed handler from stmlib? TODO

void assert_failed(uint8_t* file, uint32_t line) {
  SYS_assert((char*)file, (s32_t)line);
}
