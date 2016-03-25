/*
 * app.h
 *
 *  Created on: Jan 2, 2014
 *      Author: petera
 */

#ifndef APP_H_
#define APP_H_

#include "system.h"

#define WIFIDO_VERSION        0x00010000

#define PIN_UART_TX           PORTA, PIN2
#define PIN_UART_RX           PORTA, PIN3
#define PIN_LED               PORTC, PIN13
#define PIN_ACC_INT           PORTA, PIN0

#define APP_PREVENT_SLEEP_IF_LESS_MS    20
#define APP_WDOG_TIMEOUT_S              23
#define APP_HEARTBEAT_MS                20000
#define APP_CLI_POLL_MS                 1000
#define APP_TEMPERATURE_MS              1000*60*60
#define APP_CLI_INACT_SHUTDOWN_S        3
#define APP_KEEP_SENSORS_ALIVE_S        15

#define CLAIM_CLI             0x00
#define CLAIM_SEN             0x01
#define CLAIM_ACC             0x02
#define CLAIM_MAG             0x03
#define CLAIM_GYR             0x04
#define CLAIM_LMP             0x05
#define CLAIM_SWP             0x06

// initializes application
void APP_init(void);
void APP_shutdown(void);
void APP_claim(u8_t resource);
void APP_release(u8_t resource);
void APP_report_activity(bool activity, bool inactivity, bool tap, bool doubletap, bool issleep);
void APP_report_temperature(float temp);
void APP_report_data(
    s16_t ax, s16_t ay, s16_t az,
    s16_t mx, s16_t my, s16_t mz,
    s16_t gx, s16_t gy, s16_t gz);

void WB_init(void);

#endif /* APP_H_ */
