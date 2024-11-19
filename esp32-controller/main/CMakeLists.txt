set(srcs
        "esp_hid_gap.c"
        "pca9685.c")
set(include_dirs ".")

idf_component_register(SRCS "robotic_arm_main.c" "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid driver
                       PRIV_REQUIRES nvs_flash)
