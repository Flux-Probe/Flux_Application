set(srcs "main.c"
         "bleModule/gattSvr.c"
         "bleModule/fluxBleService.c"
         "motorCtrl/motorCtrl.c"
         "as5600/as5600.c"
         "webInterface/wifiSetup.c"
         "webInterface/webApp.c"
         )

set(dirs "."
        "bleModule"
        "motorCtrl"
        "as5600"
        "webInterface"
        "../../"        #For Configuration files
        )

set(requires driver esp_timer esp_wifi
             esp_https_server)

set(privReq bt nvs_flash)

set(embedFiles "webInterface/index.html")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${dirs}"
                       REQUIRES "${requires}"
                       PRIV_REQUIRES "${privReq}"
                       EMBED_FILES "${embedFiles}"
                       )
