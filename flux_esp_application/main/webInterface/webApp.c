/**
 *
 */
#include "webApp.h"

#include <math.h>

esp_err_t handleRoot(httpd_req_t *req);
esp_err_t handleCmd(httpd_req_t *req);
esp_err_t handleSetpct(httpd_req_t *req);
esp_err_t handleAngle(httpd_req_t *req);
esp_err_t handleMtrState(httpd_req_t *req);
esp_err_t handleMtrMode(httpd_req_t *req);
esp_err_t handleMtrIdx(httpd_req_t *req);
// esp_err_t handleDebug(httpd_req_t *req);
// esp_err_t handleTemp(httpd_req_t *req);
esp_err_t handlePing(httpd_req_t *req);

// HTML root page
extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[]   asm("_binary_index_html_end");

#define MAX_URIS 7
#define MAX_URI_PARAM_LEN 64

#define TAG "WEBAPP"

int tgtIdx = 0;

httpd_uri_t methodUris[] = {
    {
        .uri = "/cmd",
        .method = HTTP_GET,
        .handler = handleCmd
    },
    {
        .uri = "/setpct",
        .method = HTTP_GET,
        .handler = handleSetpct,
    },
    {
        .uri = "/angle",
        .method = HTTP_GET,
        .handler = handleAngle,
    },
    {
        .uri = "/ping",
        .method = HTTP_GET,
        .handler = handlePing,
    },
    {
        .uri = "/mtrSt",
        .method = HTTP_GET,
        .handler = handleMtrState,
    },
    {
        .uri = "/mtrMode",
        .method = HTTP_GET,
        .handler = handleMtrMode,
    },
    {
        .uri = "/mtrIdx",
        .method = HTTP_GET,
        .handler = handleMtrIdx,
    },
    // {
    //     .uri = "/temp",
    //     .method = HTTP_GET,
    //     .handler = handle_temp,
    // },
    // {
    //     .uri = "/debug",
    //     .method = HTTP_GET,
    //     .handler = handle_debug,
    // },
};

resp_t handleGetUri(httpd_req_t *req, char *param, char *key)
{
    char *buf;
    char tempParam[MAX_URI_PARAM_LEN] = {0};
    int len = httpd_req_get_url_query_len(req);
    resp_t sts = RESP_OK;
    if (len <= 0) {
        return RESP_ERR;
    }
    len = len + 1;
    buf = malloc(len);
    if (httpd_req_get_url_query_str(req, buf, len) == ESP_OK) {
        esp_err_t err = httpd_query_key_value(buf, key, tempParam, sizeof(tempParam));
        if (err == ESP_OK) {
            memcpy(param, tempParam, sizeof(tempParam));
            LOG_V("rawParam: %s| %s", tempParam, param);

        }
        else {
            LOG_E("Error getting value for key: %s| err: %s", key, esp_err_to_name(err));
            sts = RESP_ERR;
        }
    }
    else {
        LOG_E("Error getting query string");
        sts = RESP_ERR;
    }
    free(buf);
    return sts;
}

esp_err_t handleRoot(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, index_html_start, index_html_end - index_html_start);
}

esp_err_t handleCmd(httpd_req_t *req)
{
    motorCtrlCtx_t *mtrCtrl = (motorCtrlCtx_t *)req->user_ctx;
    motorCtx_t *mtr = &mtrCtrl->mtrs[tgtIdx];

    char param[MAX_URI_PARAM_LEN] = {0};
    LOG_I("Calling set cmd");
    resp_t sts = handleGetUri(req, param, "dir");

    if (sts == RESP_OK) {
        // Cancel closed loop, learning, etc.
        // setTargetPercent(mtr, NAN);
        setMotorDrive(mtr, MTR_STOP);

        if (strcmp(param, "fwd") == 0) {
            // mtr->driveDir = MTR_FORWARD;
            setMotorDrive(mtr, MTR_FORWARD);
        }
        else if (strcmp(param, "rev") == 0) {
            // mtr->driveDir = MTR_REVERSE;
            setMotorDrive(mtr, MTR_REVERSE);
        }
    }
    else {
        LOG_E("Error when parsing URI: handleCmd");
    }
    return httpd_resp_send(req, NULL, 0);  // 204
}

esp_err_t handleMtrState(httpd_req_t *req)
{
    motorCtrlCtx_t *mtrCtrl = (motorCtrlCtx_t *)req->user_ctx;
    motorCtx_t *mtr = &mtrCtrl->mtrs[tgtIdx];

    char param[MAX_URI_PARAM_LEN] = {0};
    LOG_I("Calling set mtrState");
    resp_t sts = handleGetUri(req, param, "state");

    if (sts == RESP_OK) {
        // Cancel closed loop, learning, etc.
        bool enable = atoi(param);
        LOG_I("Setting Enable: %d", enable);
        setMotorEnable(mtr, enable);
    }
    else {
        LOG_E("Error when parsing URI: handleMtrState");
    }
    return httpd_resp_send(req, NULL, 0);  // 204
}

esp_err_t handleMtrMode(httpd_req_t *req)
{
    motorCtrlCtx_t *mtrCtrl = (motorCtrlCtx_t *)req->user_ctx;
    motorCtx_t *mtr = &mtrCtrl->mtrs[tgtIdx];

    char param[MAX_URI_PARAM_LEN] = {0};
    LOG_I("Calling set mtr mode");
    resp_t sts = handleGetUri(req, param, "mode");

    if (sts == RESP_OK) {
        // Cancel closed loop, learning, etc.
        mtrDriveMode_e mode = atoi(param);
        LOG_I("Setting Mtr Mode: %d", mode);

        mtr->ctrlMode = mode;
    }
    else {
        LOG_E("Error when parsing URI: handleMtrState");
    }
    return httpd_resp_send(req, NULL, 0);  // 204
}

esp_err_t handleMtrIdx(httpd_req_t *req)
{
    motorCtrlCtx_t *mtrCtrl = (motorCtrlCtx_t *)req->user_ctx;

    char param[MAX_URI_PARAM_LEN] = {0};
    LOG_I("Calling set mtr mode");
    resp_t sts = handleGetUri(req, param, "idx");

    if (sts == RESP_OK) {
        // Cancel closed loop, learning, etc.
        int setIdx = atoi(param);
        LOG_I("Setting mtrIdx: %d", setIdx);
        if (setIdx >= mtrCtrl->numMotors) {
            LOG_W("Invalid motor Idx set");
        }
        else {
            tgtIdx = setIdx;
        }
    }
    else {
        LOG_E("Error when parsing URI: handleMtrState");
    }
    return httpd_resp_send(req, NULL, 0);  // 204
}


esp_err_t handleSetpct(httpd_req_t *req)
{
    motorCtrlCtx_t *mtrCtrl = (motorCtrlCtx_t *)req->user_ctx;
    motorCtx_t *mtr = &mtrCtrl->mtrs[tgtIdx];

    char param[MAX_URI_PARAM_LEN] = {0};
    LOG_V("Calling set pct");
    // resp_t sts = handleGetUri(req, param, "val");
    // if (sts == RESP_OK) {
    //     float tmpPct = atof(param)/100.0;
    //     setTargetPercent(mtr, tmpPct);
    // }
    // else {
    //     LOG_E("Error when parsing URI: handleCmd");
    // }
    return httpd_resp_send(req, NULL, 0);  // 204
}

esp_err_t handleAngle(httpd_req_t *req)
{
    motorCtrlCtx_t *mtrCtrl = (motorCtrlCtx_t *)req->user_ctx;
    motorCtx_t *mtr = &mtrCtrl->mtrs[tgtIdx];

    // float ang = readAS5600Deg(&mtr->encoderCfg);
    float ang   = mtr->position;
    float tgt   = mtr->cmd;
    // int dir     = mtr->driveDir;
    bool mtrSt  = mtr->enabled;
    int mtrMode = mtr->ctrlMode;
    // float pct   = percentOpen(ang);
    char resp[64];
    snprintf(resp, sizeof(resp), "%.1f,%.1f,%d,%d,%d", ang, tgt,
             mtrSt, mtrMode, tgtIdx);
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_send(req, resp, strlen(resp));
}

esp_err_t handlePing(httpd_req_t *req)
{
    return httpd_resp_sendstr(req, "pong");
}

void start_http_server(webapp_t *web, motorCtrlCtx_t *mtr)
{
    esp_log_level_set(TAG, ESP_LOG_INFO); // Setting debug
    httpd_config_t dfltCfg = HTTPD_DEFAULT_CONFIG();
    memcpy(&web->config, &dfltCfg, sizeof(httpd_config_t));

    web->server = NULL;

    if (httpd_start(&web->server, &web->config) == ESP_OK) {
        httpd_uri_t uri_root = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = handleRoot,
        };
        httpd_register_uri_handler(web->server, &uri_root);

        for (int uri = 0; uri < MAX_URIS; uri++) {
            methodUris[uri].user_ctx = mtr;
            httpd_register_uri_handler(web->server, &methodUris[uri]);
        }
    }
}