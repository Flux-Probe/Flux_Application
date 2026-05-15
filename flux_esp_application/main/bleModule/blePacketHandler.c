#include <stdio.h>
#include <string.h>
#include "blePacketHandler.h"
#include "protocolTypes.h"
#include "mbedtls/base64.h"
#include "cJSON.h"
#include "loggingDefs.h"

#define TAG               "PKT_HANDLER"
#define DECODE_BUF_SIZE   512
#define ASSEMBLE_BUF_SIZE 4096

static char assemble_buf[ASSEMBLE_BUF_SIZE];
static int  assemble_len   = 0;
static int  expected_total = 0;
static int  next_chunk_idx = 0;

static void handleRecipeLoad(const cJSON *root);
static void handleStepAdvance(const cJSON *root);
static void handleRecipeAbort(const cJSON *root);
static void handleSettingsUpdate(const cJSON *root);
static void dispatchPacket(const char *json);

/**
 * Function to process raw packets coming from mobile app through BLE.
 */
void blePacketHandler_onWrite(int chrIdx, uint8_t *buf, uint16_t len)
{
    // ble-plx decodes its base64 argument before transmitting, so buf is already raw UTF-8 text
    buf[len] = '\0';
    const char *text = (const char *)buf;

    int chunk_idx, chunk_total, header_end;
    /* Checking the header of a packet. Looking for "C{CURR_IDX}/{TOT_IDX}|data" */
    if (sscanf(text, "C%d/%d|%n", &chunk_idx, &chunk_total, &header_end) == 2) {
        /* First idx read */
        if (chunk_idx == 0) {
            assemble_len   = 0;
            expected_total = chunk_total;
            next_chunk_idx = 0;
        } else if (chunk_idx != next_chunk_idx || chunk_total != expected_total) {
            LOG_E("Unexpected chunk %d/%d (expected %d/%d), resetting",
                  chunk_idx, chunk_total, next_chunk_idx, expected_total);
            assemble_len   = 0;
            expected_total = 0;
            next_chunk_idx = 0;
            return;
        }

        const char *slice     = text + header_end;
        int         slice_len = (int)len - header_end;

        if (assemble_len + slice_len >= ASSEMBLE_BUF_SIZE) {
            LOG_E("Assemble buffer overflow, resetting");
            assemble_len   = 0;
            expected_total = 0;
            next_chunk_idx = 0;
            return;
        }

        /* Copy to overall static buffer*/
        memcpy(assemble_buf + assemble_len, slice, slice_len);
        assemble_len += slice_len;
        next_chunk_idx++;

        /* Only reassemble once the entire packet is read */
        if (chunk_idx == chunk_total - 1) {
            LOG_I("Chunked assembled");
            assemble_buf[assemble_len] = '\0';
            dispatchPacket(assemble_buf);
            assemble_len   = 0;
            expected_total = 0;
            next_chunk_idx = 0;
        }
    } else {
        // Single packet — not chunked
        LOG_I("Single Packet");
        dispatchPacket(text);
    }
}

static void handleRecipeLoad(const cJSON *root)
{
    static recipe_load_t recipe;
    memset(&recipe, 0, sizeof(recipe));

    const cJSON *recipeId = cJSON_GetObjectItemCaseSensitive(root, "recipeId");
    if (cJSON_IsString(recipeId)) {
        strncpy(recipe.recipeId, recipeId->valuestring, MAX_RECIPE_ID_LEN - 1);
    }

    const cJSON *probeCount = cJSON_GetObjectItemCaseSensitive(root, "probeCount");
    if (cJSON_IsNumber(probeCount)) recipe.probeCount = probeCount->valueint;

    const cJSON *valveCount = cJSON_GetObjectItemCaseSensitive(root, "valveCount");
    if (cJSON_IsNumber(valveCount)) recipe.valveCount = valveCount->valueint;

    const cJSON *disconnectBehaviour = cJSON_GetObjectItemCaseSensitive(root, "disconnectBehaviour");
    if (cJSON_IsString(disconnectBehaviour)) {
        recipe.disconnectBehaviour =
            strcmp(disconnectBehaviour->valuestring, "safe_state") == 0
                ? DISCONNECT_SAFE_STATE
                : DISCONNECT_CONTINUE;
    }

    LOG_I("Recipe ID: %s", recipe.recipeId);
    LOG_I("probeCount: %d", recipe.probeCount);
    LOG_I("valveCount: %d", recipe.valveCount);
    LOG_I("Disc: %d", recipe.disconnectBehaviour);

    const cJSON *steps = cJSON_GetObjectItemCaseSensitive(root, "steps");
    if (cJSON_IsArray(steps)) {
        const cJSON *step_json;
        cJSON_ArrayForEach(step_json, steps) {
            if (recipe.stepCount >= MAX_STEPS) {
                LOG_W("Step count exceeds MAX_STEPS (%d), truncating", MAX_STEPS);
                break;
            }
            esp_step_t *step = &recipe.steps[recipe.stepCount];

            const cJSON *stepIndex = cJSON_GetObjectItemCaseSensitive(step_json, "stepIndex");
            if (cJSON_IsNumber(stepIndex)) step->stepIndex = stepIndex->valueint;

            step->espControlled   = cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(step_json, "espControlled"));
            step->alertOnComplete = cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(step_json, "alertOnComplete"));

            const cJSON *duration = cJSON_GetObjectItemCaseSensitive(step_json, "durationSeconds");
            if (cJSON_IsNumber(duration)) {
                step->hasDuration     = true;
                step->durationSeconds = duration->valueint;
            }

            const cJSON *targetTemps = cJSON_GetObjectItemCaseSensitive(step_json, "targetTemps");

            if (cJSON_IsArray(targetTemps)) {
                const cJSON *t;
                cJSON_ArrayForEach(t, targetTemps) {
                    if (step->targetTempCount >= MAX_TARGETS_PER_STEP) break;
                        probe_target_t *target = &step->targetTemps[step->targetTempCount];

                    const cJSON *pi = cJSON_GetObjectItemCaseSensitive(t, "probeIndex");
                    if (cJSON_IsNumber(pi)) target->probeIndex = pi->valueint;

                    const cJSON *minT = cJSON_GetObjectItemCaseSensitive(t, "minTemp");
                    if (cJSON_IsNumber(minT)) { target->hasMinTemp = true; target->minTemp = (float)minT->valuedouble; }

                    const cJSON *maxT = cJSON_GetObjectItemCaseSensitive(t, "maxTemp");
                    if (cJSON_IsNumber(maxT)) { target->hasMaxTemp = true; target->maxTemp = (float)maxT->valuedouble; }

                    const cJSON *unit = cJSON_GetObjectItemCaseSensitive(t, "unit");
                    target->unit = (cJSON_IsString(unit) && strcmp(unit->valuestring, "F") == 0)
                                       ? TEMP_UNIT_F : TEMP_UNIT_C;

                    step->targetTempCount++;
                }
            }
            LOG_I("Step: %d | %d", recipe.stepCount | recipe.steps[recipe.stepCount].stepIndex);
            LOG_I(" Alert: %d", recipe.steps[recipe.stepCount].alertOnComplete);
            LOG_I(" espCont: %d", recipe.steps[recipe.stepCount].espControlled);
            LOG_I(" Has Dur: %d", recipe.steps[recipe.stepCount].hasDuration);
            LOG_I(" Duration: %d", recipe.steps[recipe.stepCount].durationSeconds);

            /*
            const cJSON *valveActions = cJSON_GetObjectItemCaseSensitive(step_json, "valveActions");
            if (cJSON_IsArray(valveActions)) {
                const cJSON *a;
                cJSON_ArrayForEach(a, valveActions) {
                    if (step->valveActionCount >= MAX_ACTIONS_PER_STEP) break;
                    valve_action_t *action = &step->valveActions[step->valveActionCount];

                    const cJSON *vi = cJSON_GetObjectItemCaseSensitive(a, "valveIndex");
                    if (cJSON_IsNumber(vi)) action->valveIndex = vi->valueint;

                    const cJSON *pos = cJSON_GetObjectItemCaseSensitive(a, "targetPosition");
                    if (cJSON_IsNumber(pos)) action->targetPosition = pos->valueint;

                    const cJSON *trigger = cJSON_GetObjectItemCaseSensitive(a, "triggerOn");
                    if (cJSON_IsString(trigger)) {
                        if      (strcmp(trigger->valuestring, "temp_reached")  == 0) action->triggerOn = TRIGGER_TEMP_REACHED;
                        else if (strcmp(trigger->valuestring, "timer_elapsed") == 0) action->triggerOn = TRIGGER_TIMER_ELAPSED;
                        else                                                          action->triggerOn = TRIGGER_STEP_START;
                    }

                    step->valveActionCount++;
                }
            }
            */

            recipe.stepCount++;
        }
    }

    LOG_I("RECIPE_LOAD: id=%s steps=%d probes=%d valves=%d",
          recipe.recipeId, recipe.stepCount, recipe.probeCount, recipe.valveCount);

    // TODO: pass to recipe executor
}

static void handleStepAdvance(const cJSON *root)
{
    const cJSON *stepIdx = cJSON_GetObjectItemCaseSensitive(root, "toStepIndex");
    if (!cJSON_IsNumber(stepIdx)) {
        LOG_E("STEP_ADVANCE missing 'toStepIndex'");
        return;
    }

    int toStepIndex = stepIdx->valueint;
    LOG_I("STEP_ADVANCE: toStepIndex=%d", toStepIndex);

    // TODO: pass to recipe executor
}

static void handleRecipeAbort(const cJSON *root)
{
    const cJSON *safeState = cJSON_GetObjectItemCaseSensitive(root, "safeState");
    if (!cJSON_IsBool(safeState)) {
        LOG_E("ABORT missing 'safeState'");
        return;
    }

    bool safeStateVal = cJSON_IsTrue(safeState);
    LOG_I("ABORT: safeState=%d", safeStateVal);

    // TODO: pass to recipe executor
}

static void handleSettingsUpdate(const cJSON *root)
{
    // TODO: Implement settings update
}

static void dispatchPacket(const char *json)
{
    int jlen = strlen(json);
    LOG_I("JSON [0-200]:   %.200s", json);
    LOG_I("JSON [200-400]: %.200s", jlen > 200 ? json + 200 : "");
    LOG_I("JSON [400+]:    %s",     jlen > 400 ? json + 400 : "");
    cJSON *root = cJSON_Parse(json);
    if (!root) {
        const char *err = cJSON_GetErrorPtr();
        LOG_E("JSON parse failed near: %.30s", err ? err : "(null)");
        return;
    }

    const cJSON *type_item = cJSON_GetObjectItemCaseSensitive(root, "type");
    if (!cJSON_IsString(type_item)) {
        LOG_E("Packet missing 'type' field");
        cJSON_Delete(root);
        return;
    }

    const char *type = type_item->valuestring;
    LOG_I("Packet type: %s", type);

    if (strcmp(type, "RECIPE_LOAD") == 0) {
        handleRecipeLoad(root);
    } else if (strcmp(type, "STEP_ADVANCE") == 0) {
        handleStepAdvance(root);
    } else if (strcmp(type, "ABORT") == 0) {
        handleRecipeAbort(root);
    } else if (strcmp(type, "SETTINGS_UPDATE") == 0) {
        handleSettingsUpdate(root);
    } else {
        LOG_W("Unknown packet type: %s", type);
    }

    cJSON_Delete(root);
}