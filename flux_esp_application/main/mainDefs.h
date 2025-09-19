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

#define RETURN_IF_ERR(sts)          \
    if ((sts) == RESP_ERR) {        \
        return RESP_ERR;            \
    }

#define RETURN_IF_ERR_LOG(sts, ...) \
    if ((sts) == RESP_ERR) {        \
        LOG_E(__VA_ARGS__);         \
        return RESP_ERR;            \
    }

#endif // MAINDEFS_H