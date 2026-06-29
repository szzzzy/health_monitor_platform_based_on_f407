#ifndef __EEPROM_CMD_H__
#define __EEPROM_CMD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "app_state.h"

bool eeprom_cmd_process(AppState_t *app, const char *line);

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_CMD_H__ */
