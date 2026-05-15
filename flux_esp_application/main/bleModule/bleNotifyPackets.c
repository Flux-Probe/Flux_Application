#include <stdio.h>
#include <string.h>
#include "bleNotifyPackets.h"
#include "fluxBleService.h"
#include "loggingDefs.h"
#include "esp_timer.h"

#define TAG "NOTIFY"

static inline long long ts_ms(void)
{
    return (long long)(esp_timer_get_time() / 1000);
}

static const char *probe_status_str(probe_status_e s)
{
    switch (s) {
        case PROBE_STATUS_OPEN:  return "open";
        case PROBE_STATUS_SHORT: return "short";
        default:                 return "ok";
    }
}

void bleNotify_stepEvent(int stepIndex, const char *event)
{
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"STEP_EVENT\",\"stepIndex\":%d,\"event\":\"%s\",\"ts\":%lld}",
             stepIndex, event, ts_ms());
    bleNotify_send(buf);
}

void bleNotify_alert(const char *severity, const char *code,
                     bool hasProbe, int probeIndex,
                     bool hasValve, int valveIndex,
                     bool hasValue, float value)
{
    char buf[192];
    int pos = snprintf(buf, sizeof(buf),
                       "{\"type\":\"ALERT\",\"severity\":\"%s\",\"code\":\"%s\"",
                       severity, code);
    if (hasProbe) pos += snprintf(buf + pos, sizeof(buf) - pos, ",\"probeIndex\":%d", probeIndex);
    if (hasValve) pos += snprintf(buf + pos, sizeof(buf) - pos, ",\"valveIndex\":%d", valveIndex);
    if (hasValue) pos += snprintf(buf + pos, sizeof(buf) - pos, ",\"value\":%.1f", value);
    snprintf(buf + pos, sizeof(buf) - pos, ",\"ts\":%lld}", ts_ms());
    bleNotify_send(buf);
}

void bleNotify_telemetry(int stepIndex, int32_t stepElapsedMs,
                         const notify_probe_t *probes, int probeCount,
                         const notify_valve_t *valves, int valveCount)
{
    static char buf[512];
    int pos = snprintf(buf, sizeof(buf),
                       "{\"type\":\"TELEMETRY\",\"ts\":%lld,\"stepIndex\":%d"
                       ",\"stepElapsedMs\":%d,\"probes\":[",
                       ts_ms(), stepIndex, (int)stepElapsedMs);

    for (int i = 0; i < probeCount && pos < (int)sizeof(buf) - 1; i++) {
        if (i > 0) buf[pos++] = ',';
        pos += snprintf(buf + pos, sizeof(buf) - pos,
                        "{\"probeIndex\":%d,\"tempC\":%.1f,\"status\":\"%s\"}",
                        probes[i].probeIndex, probes[i].tempC,
                        probe_status_str(probes[i].status));
    }

    pos += snprintf(buf + pos, sizeof(buf) - pos, "],\"valves\":[");

    for (int i = 0; i < valveCount && pos < (int)sizeof(buf) - 1; i++) {
        if (i > 0) buf[pos++] = ',';
        pos += snprintf(buf + pos, sizeof(buf) - pos,
                        "{\"valveIndex\":%d,\"currentPosition\":%d,\"targetPosition\":%d}",
                        valves[i].valveIndex, valves[i].currentPosition,
                        valves[i].targetPosition);
    }

    snprintf(buf + pos, sizeof(buf) - pos, "]}");

    if (bleNotify_send(buf) != 0) {
        LOG_W("Telemetry packet dropped (too large or not connected)");
    }
}