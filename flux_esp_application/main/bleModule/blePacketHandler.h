#ifndef H_BLE_PACKET_HANDLER_
#define H_BLE_PACKET_HANDLER_

#include <stdint.h>
#include "fluxBleService.h"

void blePacketHandler_onWrite(int chrIdx, uint8_t *buf, uint16_t len);

#endif
