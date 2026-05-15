#ifndef H_BLE_NOTIFY_PACKETS_
#define H_BLE_NOTIFY_PACKETS_

#include <stdbool.h>
#include <stdint.h>
#include "protocolTypes.h"

void bleNotify_telemetry(int stepIndex, int32_t stepElapsedMs,
                         const notify_probe_t *probes, int probeCount,
                         const notify_valve_t *valves, int valveCount);

void bleNotify_stepEvent(int stepIndex, const char *event);

void bleNotify_alert(const char *severity, const char *code,
                     bool hasProbe, int probeIndex,
                     bool hasValve, int valveIndex,
                     bool hasValue, float value);

#endif