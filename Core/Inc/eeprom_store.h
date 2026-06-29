#ifndef __EEPROM_STORE_H__
#define __EEPROM_STORE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Error types stored in the EEPROM error ring. */
#define EEPROM_ERR_NONE           0U
#define EEPROM_ERR_HARDFAULT      1U
#define EEPROM_ERR_MAX30102_FAIL  2U
#define EEPROM_ERR_SD_FAIL        3U
#define EEPROM_ERR_ECG_LEAD_OFF   4U
#define EEPROM_ERR_I2C_BUS        5U
#define EEPROM_ERR_SENSOR_STALE   6U
#define EEPROM_ERR_OLED_FAIL      7U
#define EEPROM_ERR_CRASH_RECORD   8U

/* Layout constants. */
#define EEPROM_MAGIC_VALUE        0x424D4501UL
#define EEPROM_LAYOUT_VERSION     2U
#define EEPROM_OFFSET_DATA        0x08U
#define EEPROM_DATA_SIZE          0xF8U
#define EEPROM_ERROR_LOG_COUNT    8U
#define EEPROM_HW_REV_STORED_LEN  4U
#define EEPROM_DEFAULT_LED_RED_PA 0x80U
#define EEPROM_DEFAULT_LED_IR_PA  0x80U

typedef struct {
    uint8_t  type;
    uint8_t  phase;
    uint32_t context;
    uint32_t run_hours_x10;
    uint8_t  crc8;
} eeprom_error_entry_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t crc16;

    char     serial[24];
    char     hw_rev[EEPROM_HW_REV_STORED_LEN + 1U];
    uint32_t mfg_date_ymd;
    uint8_t  max30102_led_red_pa;
    uint8_t  max30102_led_ir_pa;

    uint32_t total_run_hours_x10;
    uint32_t total_boot_count;
    uint32_t sensor_read_error_count;
    uint32_t sensor_recovery_count;

    eeprom_error_entry_t error_log[EEPROM_ERROR_LOG_COUNT];
    uint8_t error_log_wr_ptr;
    uint8_t error_log_count;

    bool initialized;
} eeprom_data_t;

extern eeprom_data_t g_eeprom;

void eeprom_store_init(void);
bool eeprom_store_sync(void);
bool eeprom_store_reset_defaults(void);
void eeprom_store_log_error(uint8_t type, uint8_t phase, uint32_t context);
void eeprom_store_update_runtime(uint32_t run_hours_x10,
                                  uint32_t sensor_read_errors,
                                  uint32_t recoveries);

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_STORE_H__ */
