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
#include "app.h"

// compile time switches

#define SPIFFS_DBG(...)                 if (_spiffs_dbg_generic)  print("      "__VA_ARGS__)
#define SPIFFS_GC_DBG(...)              if (_spiffs_dbg_gc)       print("      "__VA_ARGS__)
#define SPIFFS_CACHE_DBG(...)           if (_spiffs_dbg_cache)    print("      "__VA_ARGS__)
#define SPIFFS_CHECK_DBG(...)           if (_spiffs_dbg_check)    print("      "__VA_ARGS__)

#define CFG_PHYS_SZ                     (1024*1024)
#define CFG_PHYS_ERASE_SZ               (65536)
#define CFG_PHYS_ADDR                   (0)
#define CFG_LOG_PAGE_SZ                 (256)
#define CFG_LOG_BLOCK_SZ                (65536)
#ifndef SPIFFS_SINGLETON
#define SPIFFS_SINGLETON                0
#endif
#if SPIFFS_SINGLETON == 1
#define SPIFFS_CFG_PHYS_SZ(ignore)      CFG_PHYS_SZ
#define SPIFFS_CFG_PHYS_ERASE_SZ(ignore)  CFG_PHYS_ERASE_SZ
#define SPIFFS_CFG_PHYS_ADDR(ignore)    CFG_PHYS_ADDR
#define SPIFFS_CFG_LOG_PAGE_SZ(ignore)  CFG_LOG_PAGE_SZ
#define SPIFFS_CFG_LOG_BLOCK_SZ(ignore) CFG_LOG_BLOCK_SZ
#endif

#ifndef SPIFFS_BUFFER_HELP
#define SPIFFS_BUFFER_HELP              0
#endif
#ifndef SPIFFS_CACHE
#define SPIFFS_CACHE                    1
#endif
#ifndef SPIFFS_CACHE_WR
#define SPIFFS_CACHE_WR                 1
#endif
#ifndef SPIFFS_PAGE_CHECK
#define SPIFFS_PAGE_CHECK               1
#endif
#ifndef SPIFFS_GC_MAX_RUNS
#define SPIFFS_GC_MAX_RUNS              20
#endif
#ifndef SPIFFS_GC_HEUR_W_DELET
#define SPIFFS_GC_HEUR_W_DELET          (5)
#endif
#ifndef SPIFFS_GC_HEUR_W_USED
#define SPIFFS_GC_HEUR_W_USED           (-1)
#endif
#ifndef SPIFFS_GC_HEUR_W_ERASE_AGE
#define SPIFFS_GC_HEUR_W_ERASE_AGE      (50)
#endif
#ifndef SPIFFS_OBJ_NAME_LEN
#define SPIFFS_OBJ_NAME_LEN             (32)
#endif
#ifndef SPIFFS_COPY_BUFFER_STACK
#define SPIFFS_COPY_BUFFER_STACK        (256)
#endif
#ifndef SPIFFS_USE_MAGIC
#define SPIFFS_USE_MAGIC                (1)
#endif
#ifndef SPIFFS_USE_MAGIC_LENGTH
#define SPIFFS_USE_MAGIC_LENGTH         (1)
#endif
#ifndef SPIFFS_LOCK
#define SPIFFS_LOCK(fs)
#endif
#ifndef SPIFFS_UNLOCK
#define SPIFFS_UNLOCK(fs)
#endif
#ifndef SPIFFS_ALIGNED_OBJECT_INDEX_TABLES
#define SPIFFS_ALIGNED_OBJECT_INDEX_TABLES    1
#endif
#ifndef SPIFFS_HAL_CALLBACK_EXTRA
#define SPIFFS_HAL_CALLBACK_EXTRA       1
#endif
#ifndef SPIFFS_FILEHDL_OFFSET
#define SPIFFS_FILEHDL_OFFSET           1
#endif
#ifndef SPIFFS_READ_ONLY
#define SPIFFS_READ_ONLY                0
#endif
#ifndef SPIFFS_TEMPORAL_FD_CACHE
#define SPIFFS_TEMPORAL_FD_CACHE        1
#endif
#ifndef SPIFFS_TEMPORAL_CACHE_HIT_SCORE
#define SPIFFS_TEMPORAL_CACHE_HIT_SCORE 4
#endif
#ifndef SPIFFS_IX_MAP
#define SPIFFS_IX_MAP                   1
#endif

#define SPIFFS_TEST_VISUALISATION       1
#define SPIFFS_CACHE_STATS              0
#define SPIFFS_GC_STATS                 0

#define spiffs_printf(...)                print(__VA_ARGS__)

#define SPIFFS_TEST_VIS_FREE_STR          "_"
#define SPIFFS_TEST_VIS_DELE_STR          TEXT_BAD("/")
#define SPIFFS_TEST_VIS_INDX_STR(id)      TEXT_NOTE("I")
#define SPIFFS_TEST_VIS_DATA_STR(id)      TEXT_GOOD("d")

typedef u16_t spiffs_block_ix;
typedef u16_t spiffs_page_ix;
typedef u16_t spiffs_obj_id;
typedef u16_t spiffs_span_ix;

typedef intptr_t ptrdiff_t;

#endif /* SPIFFS_CONFIG_H_ */
