/*
 * miniutils_config.h
 *
 *  Created on: Sep 7, 2013
 *      Author: petera
 */

#ifndef MINIUTILS_CONFIG_H_
#define MINIUTILS_CONFIG_H_

#include "system.h"
#include "io.h"

#define MINIUTILS_PRINT_FLOAT
#define MINIUTILS_PRINT_LONGLONG

#define PUTC(p, c)  \
  if ((int)(p) < 256) \
    /*UART_put_char(_UART((int)(p)), (c));*/ \
    IO_put_char((u8_t)(p), (u8_t)(c)); \
  else \
    *((char*)(p)++) = (c);
#define PUTB(p, b, l)  \
  if ((int)(p) < 256) \
    /*UART_put_buf(_UART((int)(p)), (u8_t*)(b), (int)(l));*/ \
    IO_put_buf((u8_t)(p), (u8_t *)(b), (l));\
  else { \
    int ____l = (l); \
    memcpy((char*)(p),(b),____l); \
    (p)+=____l; \
  }


#endif /* MINIUTILS_CONFIG_H_ */
