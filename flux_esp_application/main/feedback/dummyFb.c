
#include "feedback.h"
#include "esp_timer.h"

#define TAG "DUMMY_FB"
#define DBG dbgFlag
static uint16_t dbgFlag = DBG_INFO | DBG_WARNING | DBG_ERROR;

typedef struct {
    int16_t maxFbVal;
} dummyFbCtx_t;

static resp_t readDummyData(feedback_t *fb, float *readVal)
{
    CHECK_PTR_RET_ERR(fb);
    dummyFbCtx_t *cfg = (dummyFbCtx_t *)fb->privCtx;
    CHECK_PTR_RET_ERR(cfg);
    uint32_t timeMs = esp_timer_get_time() / 1000000;
    int16_t tmp = timeMs % cfg->maxFbVal;
    *readVal = (float) tmp;
    return RESP_OK;
}

static resp_t resetDummyData(feedback_t *fb, float resetVal)
{
    return RESP_OK;
}

feedback_t *dummyFbInit(int16_t maxTime)
{
    dummyFbCtx_t *dummyPrivCtx = (dummyFbCtx_t *) calloc(1, sizeof(dummyFbCtx_t));
    feedback_t *dummyFb = (feedback_t *) calloc(1, sizeof(feedback_t));
    if (!dummyFb || !dummyPrivCtx) {
        return NULL;
    }
    dummyPrivCtx->maxFbVal = maxTime;

    dummyFb->privCtx = (void *)dummyPrivCtx;
    dummyFb->readData = readDummyData;
    dummyFb->readRawData = readDummyData;
    dummyFb->resetData = resetDummyData;

    return dummyFb;
}