set(srcs "main.c"
         "tempSensor/ads1115.c"
         "bleModule/gattSvr.c"
         "bleModule/fluxBleService.c"
         "bleModule/blePacketHandler.c"
         "bleModule/bleNotifyPackets.c"
         "motorCtrl/motorCtrl.c"
         "motorDrivers/motorDriver_L293D.c"
         "feedback/as5600.c"
         "feedback/dummyFb.c"
         "webInterface/wifiSetup.c"
         "webInterface/webApp.c"
         "webInterface/mtrWebBindings.c"
         )

set(dirs "."
        "bleModule"
        "motorCtrl"
        "motorDrivers"
        "feedback"
        "tempSensor"
        "webInterface"
        "../../"        #For Configuration files
        )

set(requires driver esp_timer esp_wifi esp_driver_mcpwm
             esp_https_server)

set(privReq bt nvs_flash mbedtls json)

set(embedFiles "webInterface/index.html"
               "webInterface/style.css")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${dirs}"
                       REQUIRES "${requires}"
                       PRIV_REQUIRES "${privReq}"
                       EMBED_FILES "${embedFiles}"
                       )
