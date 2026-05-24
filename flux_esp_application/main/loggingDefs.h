#include <esp_log.h>
#include "esp_bit_defs.h"

#define DBG_NONE      0
#define DBG_INFO      BIT(0)
#define DBG_DEBUG     BIT(1)
#define DBG_WARNING   BIT(2)
#define DBG_ERROR     BIT(3)
#define DBG_VERBOSE   BIT(4)

#define LOG_MSG(lvl, logLvl, ...)           \
if (DBG & (lvl)){                           \
    ESP_LOG##logLvl(TAG, __VA_ARGS__);      \
}

#define LOG_I(...) LOG_MSG(DBG_INFO,    I, __VA_ARGS__) //Info
#define LOG_D(...) LOG_MSG(DBG_DEBUG,   D, __VA_ARGS__) //Debug
#define LOG_V(...) LOG_MSG(DBG_VERBOSE, V, __VA_ARGS__) //Verbose
#define LOG_W(...) LOG_MSG(DBG_WARNING, W, __VA_ARGS__) //Warning
#define LOG_E(...) LOG_MSG(DBG_ERROR,   E, __VA_ARGS__) //Error
