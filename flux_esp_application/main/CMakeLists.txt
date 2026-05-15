set(srcs "main.c"
         "bleModule/gattSvr.c"
         "bleModule/fluxBleService.c"
         "bleModule/blePacketHandler.c"
         "motorCtrl/motorCtrl.c"
         "motorDrivers/motorDriver_L293D.c"
         "feedback/as5600.c"
         "webInterface/wifiSetup.c"
         "webInterface/webApp.c"
         )

set(dirs "."
        "bleModule"
        "motorCtrl"
        "motorDrivers"
        "feedback"
        "webInterface"
        "../../"        #For Configuration files
        )

set(requires driver esp_timer esp_wifi esp_driver_mcpwm
             esp_https_server)

set(privReq bt nvs_flash mbedtls json)

set(embedFiles "webInterface/index.html")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${dirs}"
                       REQUIRES "${requires}"
                       PRIV_REQUIRES "${privReq}"
                       EMBED_FILES "${embedFiles}"
                       )
