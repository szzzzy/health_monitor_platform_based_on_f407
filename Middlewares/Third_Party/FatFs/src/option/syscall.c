/*
 * FatFs syscall hooks.
 *
 * This project builds FatFs without RTOS reentrancy, so no mutex glue is
 * required. Keep this source file because the Keil project references it.
 */
#include "ffconf.h"

#if _FS_REENTRANT != 0
#error "Implement FatFs syscall hooks before enabling _FS_REENTRANT."
#endif
