idf_component_register(SRCS "sdm-continuous.c"
                    PRIV_REQUIRES esp_driver_sdm esp_driver_gptimer
                    INCLUDE_DIRS "include")