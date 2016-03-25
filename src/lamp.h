/*
 * lamp.h
 *
 *  Created on: Feb 10, 2016
 *      Author: petera
 */

#ifndef _LAMP_H_
#define _LAMP_H_

void LAMP_init(void);
void LAMP_enable(bool ena);
bool LAMP_on(void);
void LAMP_cycle_delta(s16_t dcycle);
void LAMP_light_delta(s8_t dlight);


#endif /* _LAMP_H_ */
