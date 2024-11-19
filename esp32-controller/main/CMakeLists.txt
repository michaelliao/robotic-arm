set(srcs
         "esp_hid_gap.c")
set(include_dirs ".")

idf_component_register(SRCS "robotic_arm_main.c" "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid
                       PRIV_REQUIRES nvs_flash)
