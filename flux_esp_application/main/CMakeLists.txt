set(srcs "main.c"
         "bleModule/gattSvr.c"
         "bleModule/fluxBleService.c"
         "motorCtrl/motorCtrl.c"
         "as5600/as5600.c"
         )

set(requires driver esp_timer)
set(privReq bt nvs_flash)

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "."
                       "bleModule"
                       "motorCtrl"
                       "as5600"
                        REQUIRES "${requires}"
                        PRIV_REQUIRES "${privReq}"
                        )
