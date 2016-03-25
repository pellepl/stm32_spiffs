/*
 * app_wifi.c
 *
 *  Created on: Feb 16, 2016
 *      Author: petera
 */

#include "system.h"
#include "io.h"
#include "taskq.h"

#include "umac.h"
#include "cli.h"


static volatile bool um_uart_rd;

static u8_t dbg_buf[64];
static u8_t dbg_ix = 0;
static bool dbg_add = FALSE;
static u8_t rx_buf[768];
static u8_t tx_buf[768];
static umac um;
static task_timer umac_timer;
static task *umac_timer_task;

static void rx_pkt(u32_t a, void *p);

static void um_impl_request_future_tick(umtick delta) {
  TASK_start_timer(umac_timer_task, &umac_timer, 0, NULL, delta, 0, "umtim");
}

static void um_impl_cancel_future_tick(void) {
  TASK_stop_timer(&umac_timer);
}

static umtick um_impl_now_tick(void) {
  return SYS_get_time_ms();
}

static void um_impl_tx_byte(u8_t c) {
  IO_put_char(IOWIFI, c);
}

static void um_impl_tx_buf(u8_t *b, u16_t len) {
  IO_put_buf(IOWIFI, b, len);
}

static void um_impl_rx_pkt(umac_pkt *pkt) {
  task *rx_task = TASK_create(rx_pkt, 0);
  TASK_run(rx_task, 0, pkt);
}

static void um_impl_tx_pkt_acked(u8_t seqno, u8_t *data, u16_t len) {

}

static void um_impl_timeout(umac_pkt *pkt) {

}

static void um_impl_nonprotocol_data(uint8_t c) {
  if (c == '\n') {
    if (!dbg_add) {
      print("[ESPDBG] ");
    }
    IO_put_buf(IODBG, dbg_buf, dbg_ix);
    print("\n");
    dbg_ix = 0;
    dbg_add = FALSE;
    return;
  }

  if (dbg_ix >= sizeof(dbg_buf)-1) {
    if (!dbg_add) {
      print("[ESPDBG] ");
    }
    IO_put_buf(IODBG, dbg_buf, dbg_ix);
    dbg_ix = 0;
    dbg_add = TRUE;
  }
  dbg_buf[dbg_ix++] = c;
}

static void task_tick(u32_t a, void *p) {
  umac_tick(&um);
}

static void rx_pkt(u32_t a, void *p) {
  umac_pkt *pkt = (umac_pkt *)p;
  u16_t ix = 0;
  switch (pkt->data[0])  {
  case 0x01:
  while (ix < pkt->length) {
    u8_t *d = &pkt->data[ix];
    print("%02x:%02x:%02x:%02x:%02x:%02x ",
        d[0], d[1], d[2], d[3], d[4], d[5]);
    print("%02x %2i %+3i ", d[38], d[39], (s8_t)d[40]);
    u32_t len = MAX(32, strlen((char *)&d[6]));
    IO_put_buf(IODBG, &d[6], len);
    print("\n");
    ix += 6+32+3;
  }
  break;
  }
}

static void um_task_on_input(u32_t io, void *p) {
  while (IO_rx_available(io)) {
    u8_t chunk[32];
    u32_t rlen = IO_get_buf(io, chunk, MIN(IO_rx_available(io), sizeof(chunk)));
    umac_report_rx_buf(&um, chunk, rlen);
  }
  um_uart_rd = FALSE;
}

static void um_rx_avail_irq(u8_t io, void *arg, u16_t available) {
  if (!um_uart_rd) {
    task *t = TASK_create(um_task_on_input, 0);
    TASK_run(t, io, NULL);
    um_uart_rd = TRUE;
  }
}

void WB_init(void) {
  IO_assure_tx(IOWIFI, TRUE);
  um_uart_rd = FALSE;
  UART_config(
      _UART(UARTWIFIIN),
      921600,
      UART_DATABITS_8,
      UART_STOPBITS_1,
      UART_PARITY_NONE,
      UART_FLOWCONTROL_NONE,
      TRUE);
  umac_timer_task = TASK_create(task_tick, TASK_STATIC);
  umac_cfg cfg = {
      .timer_fn = um_impl_request_future_tick,
      .cancel_timer_fn = um_impl_cancel_future_tick,
      .now_fn = um_impl_now_tick,
      .tx_byte_fn = um_impl_tx_byte,
      .tx_buf_fn = um_impl_tx_buf,
      .rx_pkt_fn = um_impl_rx_pkt,
      .rx_pkt_ack_fn = um_impl_tx_pkt_acked,
      .timeout_fn = um_impl_timeout,
      .nonprotocol_data_fn = um_impl_nonprotocol_data
  };
  umac_init(&um, &cfg, rx_buf);
  IO_set_callback(IOWIFI, um_rx_avail_irq, NULL);
}


static s32_t cli_time(u32_t argc) {
  tx_buf[0] = 0x00;
  umac_tx_pkt(&um, TRUE, tx_buf, 1);
  return CLI_OK;
}

static s32_t cli_scan(u32_t argc) {
  tx_buf[0] = 0x01;
  umac_tx_pkt(&um, TRUE, tx_buf, 1);
  return CLI_OK;
}

CLI_MENU_START(wifi)
CLI_FUNC("time", cli_time, "Send time packet")
CLI_FUNC("scan", cli_scan, "Scans APs")
CLI_MENU_END

