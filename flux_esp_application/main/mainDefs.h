// Use for common functions/Marcos for all/most files
#ifndef MAINDEFS_H
#define MAINDEFS_H

#include <stdbool.h>
#include <stdint.h>
#include "loggingDefs.h"

typedef enum {
    RESP_ERR = -1,
    RESP_OK,
} resp_t;

#define CHECK_PTR_RET_ERR(ptr, ...)         \
    if (ptr == NULL) {                      \
        LOG_E("INVALID PTR %d", __LINE__);  \
        LOG_E(__VA_ARGS__);                 \
        return RESP_ERR;                    \
    }                                       \

#define CHECK_PTR_RET(ptr, ...)             \
    if (ptr == NULL) {                      \
        LOG_E("INVALID PTR %d", __LINE__);  \
        LOG_E(__VA_ARGS__);                 \
        return;                             \
    }                                       \

#define RETURN_IF_ERR(sts)          \
    if ((sts) == RESP_ERR) {        \
        return;                     \
    }

#define RETURN_IF_ERR_LOG(sts, ...) \
    if ((sts) == RESP_ERR) {        \
        LOG_E(__VA_ARGS__);         \
        return;                     \
    }

#define RETURN_VAL_IF_ERR(sts, ret) \
    if ((sts) == RESP_ERR) {        \
        return (ret);               \
    }

#define RETURN_VAL_IF_ERR_LOG(sts, ret, ...) \
    if ((sts) == RESP_ERR) {                 \
        LOG_E(__VA_ARGS__);                  \
        return (ret);                        \
    }

#endif // MAINDEFS_H