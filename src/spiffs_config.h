/*
 * spiffs_config.h
 *
 *  Created on: Jul 3, 2013
 *      Author: petera
 */

#ifndef SPIFFS_CONFIG_H_
#define SPIFFS_CONFIG_H_

#include "system.h"
#include "miniutils.h"

// compile time switches

#define SPIFFS_DBG(...)                 //print(__VA_ARGS__)
#define SPIFFS_GC_DBG(...)              //print(__VA_ARGS__)
#define SPIFFS_CACHE_DBG(...)           //print(__VA_ARGS__)
#define SPIFFS_CHECK_DBG(...)           //print(__VA_ARGS__)

#define SPIFFS_BUFFER_HELP              0
#define SPIFFS_CACHE                    1
#define SPIFFS_CACHE_WR                 1
#define SPIFFS_CACHE_STATS              0
#define SPIFFS_PAGE_CHECK               1
#define SPIFFS_GC_MAX_RUNS              20
#define SPIFFS_GC_STATS                 0
#define SPIFFS_GC_HEUR_W_DELET          (5)
#define SPIFFS_GC_HEUR_W_USED           (-1)
#define SPIFFS_GC_HEUR_W_ERASE_AGE      (50)
#define SPIFFS_OBJ_NAME_LEN             (32)
#define SPIFFS_COPY_BUFFER_STACK        (256)
#define SPIFFS_USE_MAGIC                (1)
#define SPIFFS_USE_MAGIC_LENGTH         (1)
#define SPIFFS_LOCK(fs)
#define SPIFFS_UNLOCK(fs)
#define SPIFFS_SINGLETON                1
#define SPIFFS_CFG_PHYS_SZ(ignore)      (1024*1024)
#define SPIFFS_CFG_PHYS_ERASE_SZ(ignore)      (65536)
#define SPIFFS_CFG_PHYS_ADDR(ignore)    (0)
#define SPIFFS_CFG_LOG_PAGE_SZ(ignore)  (256)
#define SPIFFS_CFG_LOG_BLOCK_SZ(ignore) (65536)
#define SPIFFS_ALIGNED_OBJECT_INDEX_TABLES    0
#define SPIFFS_HAL_CALLBACK_EXTRA       0
#define SPIFFS_FILEHDL_OFFSET           0
#define SPIFFS_READ_ONLY                0
#define SPIFFS_TEST_VISUALISATION       1

#define spiffs_printf(...)                print(__VA_ARGS__)

#define SPIFFS_TEST_VIS_FREE_STR          "_"
#define SPIFFS_TEST_VIS_DELE_STR          "/"
#define SPIFFS_TEST_VIS_INDX_STR(id)      "i"
#define SPIFFS_TEST_VIS_DATA_STR(id)      "d"

typedef u16_t spiffs_block_ix;
typedef u16_t spiffs_page_ix;
typedef u16_t spiffs_obj_id;
typedef u16_t spiffs_span_ix;

typedef intptr_t ptrdiff_t;

#endif /* SPIFFS_CONFIG_H_ */
