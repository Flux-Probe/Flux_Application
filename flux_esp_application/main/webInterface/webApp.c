#include "webApp.h"
#include <string.h>
#include <stdlib.h>

#define TAG           "WEBAPP"
#define DBG dbgFlag
static uint16_t dbgFlag = DBG_INFO | DBG_WARNING | DBG_ERROR;

#define MAX_PARAM_LEN  64

extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[]   asm("_binary_index_html_end");
extern const char style_css_start[]  asm("_binary_style_css_start");
extern const char style_css_end[]    asm("_binary_style_css_end");

// ── Helpers ───────────────────────────────────────────────────────────────────
static char *paramBuf;

static resp_t getUriParam(httpd_req_t *req, char *out, size_t outLen, const char *key)
{
    int len = httpd_req_get_url_query_len(req);
    if (len <= 0) return RESP_ERR;

    resp_t sts = RESP_ERR;
    if (httpd_req_get_url_query_str(req, paramBuf, len + 1) == ESP_OK) {
        if (httpd_query_key_value(paramBuf, key, out, outLen) == ESP_OK)
            sts = RESP_OK;
    }

    return sts;
}

// ── Handlers ──────────────────────────────────────────────────────────────────

static esp_err_t handleRoot(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, index_html_start, index_html_end - index_html_start);
}

static esp_err_t handlePing(httpd_req_t *req)
{
    return httpd_resp_sendstr(req, "pong");
}

static esp_err_t handleStyleCss(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/css");
    return httpd_resp_send(req, style_css_start, style_css_end - style_css_start);
}

/* GET /data → {"mtr0.pos":123.45,"mtr0.enabled":false,...}
   Chunked so buffer size doesn't grow with field count.
*/
static esp_err_t handleGetData(httpd_req_t *req)
{
    webApp_t *web = (webApp_t *)req->user_ctx;
    char val[48];
    char chunk[96];

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr_chunk(req, "{");

    for (int i = 0; i < web->monitorCount; i++) {
        web->monitors[i].get(val, sizeof(val), web->monitors[i].ctx);
        int n = snprintf(chunk, sizeof(chunk), "%s\"%s\":%s",
                         i > 0 ? "," : "", web->monitors[i].key, val);
        httpd_resp_send_chunk(req, chunk, n);
    }

    httpd_resp_sendstr_chunk(req, "}");
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* GET /set?key=mtr0.speed&val=200 → {"ok":true} or {"ok":false,"error":"..."}
   Calling specific control commands that are registered within the main.c file.
*/
static esp_err_t handleSet(httpd_req_t *req)
{
    webApp_t *web = (webApp_t *)req->user_ctx;
    char key[WEBAPP_KEY_LEN] = {0};
    char val[MAX_PARAM_LEN]  = {0};

    httpd_resp_set_type(req, "application/json");

    if (getUriParam(req, key, sizeof(key), "key") != RESP_OK ||
        getUriParam(req, val, sizeof(val), "val") != RESP_OK) {
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"missing key or val\"}");
    }

    for (int i = 0; i < web->controlCount; i++) {
        if (strncmp(web->controls[i].key, key, WEBAPP_KEY_LEN) == 0) {
            web->controls[i].set(val, web->controls[i].ctx);
            LOG_I("set %s = %s", key, val);
            return httpd_resp_sendstr(req, "{\"ok\":true}");
        }
    }

    LOG_W("set: unknown key '%s'", key);
    return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"unknown key\"}");
}

// ── Registration ──────────────────────────────────────────────────────────────

resp_t webAppRegisterMonitor(webApp_t *web, const char *key, webAppGetter_t get, void *ctx)
{
    if (!key || !ctx || !get) {
        return RESP_ERR;
    }
    if (web->monitorCount >= WEBAPP_MAX_MONITORS) {
        LOG_E("Monitor table full (max %d)", WEBAPP_MAX_MONITORS);
        return RESP_ERR;
    }
    webAppMonitor_t *entry = &web->monitors[web->monitorCount++];
    strncpy(entry->key, key, WEBAPP_KEY_LEN - 1);
    entry->key[WEBAPP_KEY_LEN - 1] = '\0';
    entry->get = get;
    entry->ctx = ctx;
    return RESP_OK;
}

resp_t webAppRegisterControl(webApp_t *web, const char *key, webAppSetter_t set, void *ctx)
{
    if (!key || !ctx || !set) {
        return RESP_ERR;
    }
    if (web->controlCount >= WEBAPP_MAX_CONTROLS) {
        LOG_E("Control table full (max %d)", WEBAPP_MAX_CONTROLS);
        return RESP_ERR;
    }
    webAppControl_t *entry = &web->controls[web->controlCount++];
    strncpy(entry->key, key, WEBAPP_KEY_LEN - 1);
    entry->key[WEBAPP_KEY_LEN - 1] = '\0';
    entry->set = set;
    entry->ctx = ctx;
    return RESP_OK;
}

static void initializeRoutes(webApp_t *web)
{
    httpd_uri_t routes[] = {
        { .uri = "/",          .method = HTTP_GET, .handler = handleRoot,     .user_ctx = NULL },
        { .uri = "/ping",      .method = HTTP_GET, .handler = handlePing,     .user_ctx = NULL },
        { .uri = "/style.css", .method = HTTP_GET, .handler = handleStyleCss, .user_ctx = NULL },
        { .uri = "/data",      .method = HTTP_GET, .handler = handleGetData,  .user_ctx = web  },
        { .uri = "/set",       .method = HTTP_GET, .handler = handleSet,      .user_ctx = web  },
    };

    for (int i = 0; i < (int)(sizeof(routes) / sizeof(routes[0])); i++) {
        httpd_register_uri_handler(web->server, &routes[i]);
    }
}

// ── Server init ───────────────────────────────────────────────────────────────
resp_t startHttpServer(webApp_t *web)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    memcpy(&web->config, &cfg, sizeof(httpd_config_t));
    web->server = NULL;

    paramBuf = malloc(MAX_PARAM_LEN);
    if (!paramBuf) {
        LOG_E("Error when allocating param buffer");
        return RESP_ERR;
    }

    if (httpd_start(&web->server, &web->config) != ESP_OK) {
        LOG_E("Failed to start HTTP server");
        return RESP_ERR;
    }

    /* Initializing each route available in the webApp */
    initializeRoutes(web);

    LOG_I("HTTP server ready — GET /  GET /style.css  GET /data  GET /set?key=X&val=Y  GET /ping");
    return RESP_OK;
}
