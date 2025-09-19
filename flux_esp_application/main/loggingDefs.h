#include <esp_log.h>

#define LOG_I(...) ESP_LOGI(TAG, __VA_ARGS__) //Info
#define LOG_E(...) ESP_LOGE(TAG, __VA_ARGS__) //Error
#define LOG_D(...) ESP_LOGD(TAG, __VA_ARGS__) //Debug
#define LOG_V(...) ESP_LOGV(TAG, __VA_ARGS__) //Verbose
#define LOG_W(...) ESP_LOGW(TAG, __VA_ARGS__) //Warning