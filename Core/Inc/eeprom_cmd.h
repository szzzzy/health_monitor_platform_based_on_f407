#ifndef __EEPROM_CMD_H__
#define __EEPROM_CMD_H__

/**
 * @file eeprom_cmd.h
 * @brief EEPROM 维护命令的协议分派接口。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "app_state.h"

/**
 * @brief  尝试处理一条 eeprom 子命令。
 * @return 属于 EEPROM 命令域时返回 true（包括未知子命令）；否则返回 false。
 */
bool eeprom_cmd_process(AppState_t *app, const char *line);

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_CMD_H__ */
