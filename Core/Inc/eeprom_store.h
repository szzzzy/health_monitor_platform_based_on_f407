#ifndef __EEPROM_STORE_H__
#define __EEPROM_STORE_H__

/**
 * @file eeprom_store.h
 * @brief M24C02 持久化布局、RAM 镜像和延迟同步接口。
 *
 * 布局有效性由 magic/version/CRC16 共同判定；错误环中每条记录另带 CRC8。
 * 所有时长统计使用 0.1 小时为单位，出厂日期使用 YYYYMMDD 十进制编码。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* EEPROM 错误环中的持久化类型码。 */
#define EEPROM_ERR_NONE           0U
#define EEPROM_ERR_HARDFAULT      1U
#define EEPROM_ERR_MAX30102_FAIL  2U
#define EEPROM_ERR_SD_FAIL        3U
#define EEPROM_ERR_ECG_LEAD_OFF   4U
#define EEPROM_ERR_I2C_BUS        5U
#define EEPROM_ERR_SENSOR_STALE   6U
#define EEPROM_ERR_OLED_FAIL      7U
#define EEPROM_ERR_CRASH_RECORD   8U

/* 布局版本、数据区范围与固定容量。 */
#define EEPROM_MAGIC_VALUE        0x424D4501UL
#define EEPROM_LAYOUT_VERSION     2U
#define EEPROM_OFFSET_DATA        0x08U
#define EEPROM_DATA_SIZE          0xF8U
#define EEPROM_ERROR_LOG_COUNT    8U
#define EEPROM_HW_REV_STORED_LEN  4U
#define EEPROM_DEFAULT_LED_RED_PA 0x80U
#define EEPROM_DEFAULT_LED_IR_PA  0x80U

typedef struct {
    uint8_t  type;          /* EEPROM_ERR_* 类型码 */
    uint8_t  phase;         /* 发生错误时的任务/子系统阶段码 */
    uint32_t context;       /* 类型相关上下文，如 HAL 错误码或崩溃来源 */
    uint32_t run_hours_x10; /* 发生时累计运行时长，单位：0.1 h */
    uint8_t  crc8;          /* 持久化记录前 10 字节的 CRC8 */
} eeprom_error_entry_t;

typedef struct {
    uint32_t magic;         /* EEPROM_MAGIC_VALUE */
    uint16_t version;       /* EEPROM_LAYOUT_VERSION */
    uint16_t crc16;         /* 数据区 [0x08,0xFF] 的 CRC16-CCITT */

    char     serial[24];
    char     hw_rev[EEPROM_HW_REV_STORED_LEN + 1U];
    uint32_t mfg_date_ymd;          /* 出厂日期，YYYYMMDD；0 表示未设置 */
    uint8_t  max30102_led_red_pa;    /* RED LED 脉冲幅度寄存器值 */
    uint8_t  max30102_led_ir_pa;     /* IR LED 脉冲幅度寄存器值 */

    uint32_t total_run_hours_x10;   /* 累计运行时长，单位：0.1 h */
    uint32_t total_boot_count;
    uint32_t sensor_read_error_count;
    uint32_t sensor_recovery_count;

    eeprom_error_entry_t error_log[EEPROM_ERROR_LOG_COUNT];
    uint8_t error_log_wr_ptr;       /* 下一条写入槽位，范围 0..7 */
    uint8_t error_log_count;        /* 饱和累计记录数；显示时最多取最近 8 条 */

    bool initialized;               /* RAM 镜像已从有效布局装载或成功写入 */
} eeprom_data_t;

extern eeprom_data_t g_eeprom;

/** @brief 启动时读取并校验 EEPROM；空白、版本不符或 CRC 错误时装载默认值。 */
void eeprom_store_init(void);
/** @brief 尝试把当前 RAM 镜像完整提交到 EEPROM。 */
bool eeprom_store_sync(void);
/** @brief 用出厂默认值替换 RAM 镜像并尝试持久化。 */
bool eeprom_store_reset_defaults(void);
/** @brief 向 8 槽错误环追加一条记录，并尝试持久化。 */
bool eeprom_store_log_error(uint8_t type, uint8_t phase, uint32_t context);
/** @brief 更新运行时统计镜像并尝试持久化；时长单位为 0.1 h。 */
bool eeprom_store_update_runtime(uint32_t run_hours_x10,
                                 uint32_t sensor_read_errors,
                                 uint32_t recoveries);
/** @brief 设置测量活跃门控；活跃期间所有物理 EEPROM 写入会转为 dirty。 */
void eeprom_store_set_measurement_active(bool active);
/** @brief 在非测量安全窗口重试 dirty 镜像；遵守写失败后的退避截止时间。 */
bool eeprom_store_service_deferred(void);
/** @brief 返回底层 I2C 写失败累计次数。 */
uint32_t eeprom_store_get_write_error_count(void);

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_STORE_H__ */
