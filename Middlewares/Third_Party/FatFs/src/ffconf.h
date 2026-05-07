/*----------------------------------------------------------------------------/
/  FatFs - FAT file system module configuration file  R0.12c
/
/  BME 数据记录项目适配：
/    - FAT32, 单分区, 长文件名（栈分配）
/    - 全写入 API (f_write/f_sync/f_printf/f_mkfs)
/    - 使用 RTC 提供文件时间戳
/----------------------------------------------------------------------------*/

#define _FFCONF 68300  /* Revision ID */

/* Function Configurations */

#define _FS_READONLY    0   /* 0:Read/Write */
#define _FS_MINIMIZE    0   /* 0:All functions enabled */
#define _USE_STRFUNC    2   /* 2:Enable string functions with LF-CRLF conversion */
#define _USE_FIND       1   /* 1:Enable f_findfirst/f_findnext */
#define _USE_MKFS       1   /* 1:Enable f_mkfs */
#define _USE_FASTSEEK   0   /* 0:Disable */
#define _USE_EXPAND     0   /* 0:Disable */
#define _USE_CHMOD      0   /* 0:Disable */
#define _USE_LABEL      1   /* 1:Enable volume label functions */
#define _USE_FORWARD    0   /* 0:Disable */

/* Locale and Namespace */

#define _CODE_PAGE      437 /* U.S. ASCII */
#define _USE_LFN        2   /* 2:Enable LFN with stack-allocated buffer */
#define _MAX_LFN        255
#define _LFN_UNICODE    0   /* ANSI/OEM */
#define _STRF_ENCODE    3   /* UTF-8 for string I/O */

#define _FS_RPATH       0   /* 0:Disable relative path */

/* Drive/Volume */

#define _VOLUMES        1
#define _STR_VOLUME_ID  1
#define _VOLUME_STRS    "SD"
#define _MULTI_PARTITION 0
#define _MIN_SS         512
#define _MAX_SS         512
#define _USE_TRIM       0

#define _FS_EXFAT       0   /* 0:Disable exFAT */
#define _FS_NOFSINFO    0   /* 0:Do not write FSInfo sector */

/* System */

#define _FS_TINY        0   /* 0:Normal (separate sector buffer per file) */
#define _FS_NORTC       0   /* 0:Use get_fattime() for file timestamps */
#define _FS_LOCK        2   /* 2:Allow 2 concurrently open files */
#define _FS_REENTRANT   0   /* 0:No RTOS */
