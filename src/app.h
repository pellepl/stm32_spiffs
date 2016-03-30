/*
 * app.h
 *
 *  Created on: Jan 2, 2014
 *      Author: petera
 */

#ifndef APP_H_
#define APP_H_

#include "system.h"

#define APP_PREVENT_SLEEP_IF_LESS_MS    20
#define APP_WDOG_TIMEOUT_S              23
#define APP_HEARTBEAT_MS                20000
#define APP_CLI_POLL_MS                 1000
#define APP_CLI_INACT_SHUTDOWN_S        3

// initializes application
void APP_init(void);
void APP_shutdown(void);
void APP_claim(u8_t resource);
void APP_release(u8_t resource);
void APP_rtc_cb(void);
#endif /* APP_H_ */
