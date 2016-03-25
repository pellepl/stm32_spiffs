/*
 * sensor.h
 *
 *  Created on: Feb 8, 2016
 *      Author: petera
 */

#ifndef SRC_SENSOR_H_
#define SRC_SENSOR_H_

#include "system.h"

void SENS_init(void);
void SENS_enter_active(void);
void SENS_enter_idle(void);
void SENS_read_temp(void);

#endif /* SRC_SENSOR_H_ */
