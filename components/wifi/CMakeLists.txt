idf_component_register(SRCS "wifi.c" "wifi.h"
                    REQUIRES driver esp_wifi freertos
                    INCLUDE_DIRS ".")