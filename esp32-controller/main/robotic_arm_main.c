/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <driver/i2c.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#if CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#else
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#endif

#if CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#define ESP_BD_ADDR_STR         "%02x:%02x:%02x:%02x:%02x:%02x"
#define ESP_BD_ADDR_HEX(addr)   addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
#else
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#endif

#include "esp_hidh.h"
#include "esp_hid_gap.h"

#include "pca9685.h"

// 注意: 将IO管脚修改为实际连接的PIN:
#define RA_I2C_MASTER_SDA_IO 8
#define RA_I2C_MASTER_SCL_IO 9

// 注意: I2C设备地址默认为0x40, 如有修改, 需改为实际地址:
#define RA_I2C_ADDRESS 0x40

// PWM频率为50Hz:
#define RA_PWM_FREQ 50

// PWM周期为20ms=20000us:
#define RA_SERVO_PWM_TIME (uint32_t)20000
// PWM脉冲时间为0.5~2.5ms=500~2500us:
#define RA_SERVO_PULSE_MIN (uint32_t)500
#define RA_SERVO_PULSE_MAX (uint32_t)2500
// 计算PWM对应的范围:
#define RA_SERVO_PWM_MIN (uint32_t)(4096 * RA_SERVO_PULSE_MIN / RA_SERVO_PWM_TIME - 1)
#define RA_SERVO_PWM_MAX (uint32_t)(4096 * RA_SERVO_PULSE_MAX / RA_SERVO_PWM_TIME - 1)
#define RA_SERVO_PWM_MID (uint32_t)((RA_SERVO_PWM_MIN + RA_SERVO_PWM_MAX) / 2)
#define RA_SERVO_PWM_RANGE (uint32_t)(RA_SERVO_PWM_MAX - RA_SERVO_PWM_MIN + 1)

// 角度范围:
#define RA_SERVO_ANGLE_RANGE (uint32_t)180

// I2C频率=100KHz:
#define RA_I2C_MASTER_FREQ 100000

// 舵机索引号:
#define RA_SERVO_A 0
#define RA_SERVO_B 1
#define RA_SERVO_C 2
#define RA_SERVO_D 3

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

// XBox手柄按钮:
#define XBOX_BUTTON_A 0x01
#define XBOX_BUTTON_B 0x02
#define XBOX_BUTTON_X 0x08
#define XBOX_BUTTON_Y 0x10
#define XBOX_BUTTON_L1 0x40
#define XBOX_BUTTON_R1 0x80
#define XBOX_EX_BUTTON_VIEW 0x04
#define XBOX_EX_BUTTON_MENU 0x08

#define CTRL_INPUT_DATA_LENGTH 16

#define SCAN_DURATION_SECONDS 5

static const char *TAG_HID = "HID";
static const char *TAG_PWM = "PWM";

// 根据手柄名称进行过滤,注意大小写:
static const char *CTRL_NAME = "Xbox Wireless Controller";

// 手柄输入队列:
QueueHandle_t ctrl_input_queue = NULL;

// 手柄输入数据:
// https://support.xbox.com/en-US/help/hardware-network/controller/xbox-one-wireless-controller
typedef struct __attribute__((packed)) {
    uint16_t left_stick_x; // 左摇杆
    uint16_t left_stick_y;
    uint16_t right_stick_x; // 右摇杆
    uint16_t right_stick_y;
    uint8_t left_trigger; // 左上L2按钮
    uint8_t left_trigger_level;
    uint8_t right_trigger; // 右上R2按钮
    uint8_t right_trigger_level;
    uint8_t any_1;
    uint8_t buttons; // A,B,X,Y,L1,R1按钮
    uint8_t ex_buttons; // VIEW, MENU按钮
    uint8_t any_2;
} ctrl_input_data_t;

typedef struct {
    // 允许的角度范围,例如60~120:
    int16_t min_angle;
    int16_t max_angle;
    // 初始角度,理想状态是90,但允许误差,如实际为95:
    int16_t initial_angle;
    // 目标角度:
    int16_t target_angle;
} servo_t;

static servo_t servos[4];

// 连接状态:
enum CTRL_STATE
{
    SCANNING, // 正在扫描
    OPENING, // 正在打开
    INPUT // 正常输入
};

static enum CTRL_STATE ctrl_state = SCANNING;

// 记录上一次的手柄输入数据:
static ctrl_input_data_t ctrl_input_data;
// 记录上一次的手柄输入时间:
static int64_t ctrl_input_time = 0;

static char *bda2str(esp_bd_addr_t bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

// 处理蓝牙手柄输入:
void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDH_OPEN_EVENT: {
        if (param->open.status == ESP_OK) {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
            ESP_LOGI(TAG_HID, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
            esp_hidh_dev_dump(param->open.dev, stdout);
            ctrl_state = INPUT;
        } else {
            ESP_LOGE(TAG_HID, " OPEN failed!");
            ctrl_state = SCANNING;
        }
        break;
    }
    case ESP_HIDH_BATTERY_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
        ESP_LOGI(TAG_HID, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
        break;
    }
    case ESP_HIDH_INPUT_EVENT: {
        // 处理手柄输入:
        const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
        if (param->input.length == CTRL_INPUT_DATA_LENGTH) {
            // 每200ms采样输入:
            int64_t now = esp_timer_get_time();
            if (now - ctrl_input_time > 200000) {
                // 复制输入数据并发送消息到队列:
                memcpy(&ctrl_input_data, (char *) param->input.data, CTRL_INPUT_DATA_LENGTH);
                xQueueSend(ctrl_input_queue, &ctrl_input_data, 0); // 如果上一次的输入数据还未处理，则不等待直接丢弃
                ESP_LOGI(TAG_HID, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
                ESP_LOG_BUFFER_HEX(TAG_HID, param->input.data, param->input.length);
                ctrl_input_time = now;
            }
        } else {
            ESP_LOGW(TAG_HID, "invalid input length: %d", param->input.length);
        }
        break;
    }
    case ESP_HIDH_FEATURE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
        ESP_LOGI(TAG_HID, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda),
                 esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id,
                 param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG_HID, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDH_CLOSE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
        ESP_LOGI(TAG_HID, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
        ctrl_state = SCANNING;
        break;
    }
    default:
        ESP_LOGI(TAG_HID, "default EVENT: %d", event);
        break;
    }
}

void hid_scan()
{
    size_t results_len = 0;
    esp_hid_scan_result_t *results = NULL;
    ESP_LOGI(TAG_HID, "SCAN...");
    //start scan for HID devices
    esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results);
    ESP_LOGI(TAG_HID, "SCAN: %u results", results_len);
    if (results_len) {
        esp_hid_scan_result_t *r = results;
        esp_hid_scan_result_t *cr = NULL;
        while (r) {
            printf("  %s: " ESP_BD_ADDR_STR ", ", (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ", ESP_BD_ADDR_HEX(r->bda));
            printf("RSSI: %d, ", r->rssi);
            printf("USAGE: %s, ", esp_hid_usage_str(r->usage));
#if CONFIG_BT_BLE_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BLE) {
                if (r->name && strcmp(r->name, CTRL_NAME) == 0) {
                    cr = r;
                }
                printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));
            }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_NIMBLE_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BLE) {
                if (r->name && strcmp(r->name, CTRL_NAME) == 0) {
                    cr = r;
                }
                printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                printf("ADDR_TYPE: '%d', ", r->ble.addr_type);
            }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_HID_HOST_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BT) {
                if (r->name && strcmp(r->name, CTRL_NAME) == 0) {
                    cr = r;
                }
                printf("COD: %s[", esp_hid_cod_major_str(r->bt.cod.major));
                esp_hid_cod_minor_print(r->bt.cod.minor, stdout);
                printf("] srv 0x%03x, ", r->bt.cod.service);
                print_uuid(&r->bt.uuid);
                printf(", ");
            }
#endif /* CONFIG_BT_HID_HOST_ENABLED */
            printf("NAME: %s ", r->name ? r->name : "");
            printf("\n");
            r = r->next;
        }
        if (cr) {
            //open the last result
            ctrl_state = OPENING;
            esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
        }
        //free the results
        esp_hid_scan_results_free(results);
    }
}

void hid_scan_task(void *pvParameters)
{
    for (;;) {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        if (ctrl_state == SCANNING) {
            hid_scan();
        }
    }
}

#if CONFIG_BT_NIMBLE_ENABLED
void ble_hid_host_task(void *param)
{
    ESP_LOGI(TAG_HID, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}
void ble_store_config_init(void);
#endif

static void init_i2c_master()
{
    ESP_LOGI(TAG_HID, "init i2c master: SDA = %d, SCL = %d.", RA_I2C_MASTER_SDA_IO, RA_I2C_MASTER_SCL_IO);
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = RA_I2C_MASTER_SDA_IO;
    conf.scl_io_num = RA_I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = RA_I2C_MASTER_FREQ;

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
}

static void init_chip()
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
}

static inline void set_servo_pwm(uint8_t channel, int16_t angle)
{
    ESP_LOGI(TAG_PWM, "set %d angle: %d", channel, angle);
    uint32_t pwm = RA_SERVO_PWM_RANGE * (uint32_t)angle / RA_SERVO_ANGLE_RANGE + RA_SERVO_PWM_MIN;
    pca9685_set_channel_pwm(channel, (uint16_t) pwm);
}

static inline void move_servo(uint32_t index, int16_t offset)
{
    servo_t *s = &servos[index];
    s->target_angle = s->target_angle + offset;
    if (s->target_angle < s->min_angle) {
        s->target_angle = s->min_angle;
    }
    if (s->target_angle > s->max_angle) {
        s->target_angle = s->max_angle;
    }
    set_servo_pwm(index, s->target_angle);
}

static void init_servos()
{
    // 旋转底座舵机A:
    servos[RA_SERVO_A].initial_angle = servos[RA_SERVO_A].target_angle = 90;
    servos[RA_SERVO_A].min_angle = 20;
    servos[RA_SERVO_A].max_angle = 160;
    // B轴舵机B:
    servos[RA_SERVO_B].initial_angle = servos[RA_SERVO_B].target_angle = 90;
    servos[RA_SERVO_B].min_angle = 70;
    servos[RA_SERVO_B].max_angle = 110;
    // CD连杆舵机C:
    servos[RA_SERVO_C].initial_angle = servos[RA_SERVO_C].target_angle = 90;
    servos[RA_SERVO_C].min_angle = 45;
    servos[RA_SERVO_C].max_angle = 135;
    // 夹爪轴舵机D:
    servos[RA_SERVO_D].initial_angle = servos[RA_SERVO_D].target_angle = 90;
    servos[RA_SERVO_D].min_angle = 70;
    servos[RA_SERVO_D].max_angle = 110;
    // 读取实际安装角度并调整:
    // TODO:
    // 重置所有舵机:
    for (uint32_t i=0; i<4; i++) {
        set_servo_pwm(i, servos[i].target_angle);
    }
}

// 摇杆输入的x,y范围是:左=0 ~ 中=0x8000 ~ 右=0xffff
// 将其转换为偏移量 左=-8 ~ 中=0 ~ 右=8
static inline int16_t get_offset(uint16_t v) {
    if (v <= 0x7000) {
        // 返回 -8 ~ -1:
        int16_t offset = (int16_t)((0x8000 - v) >> 12);
        return -offset;
    } else if (v >= 0x8fff) {
        // 返回 1 ~ 8:
        return (int16_t)((v - 0x7fff) >> 12);
    }
    return 0;
}

void control_task(void *pvParameters)
{
    // 从队列取出手柄信号并控制舵机:
    ctrl_input_data_t input;
    for (;;) {
        if (pdTRUE == xQueueReceive(ctrl_input_queue, &input, 0)) {
            ESP_LOGI("Queue", "Receive input: %d", input.buttons);
            // 判断左右移动:
            int16_t left = get_offset(input.left_stick_x);
            if (left != 0) {
                move_servo(RA_SERVO_A, left);
            }
            // 判断前后移动:
            int16_t forward = get_offset(input.left_stick_y);
            if (forward != 0) {
                move_servo(RA_SERVO_B, -forward);
            }
            // 判断上下移动:
            int16_t up = get_offset(input.right_stick_y);
            if (up != 0) {
                move_servo(RA_SERVO_C, -up);
            }
            // L1合上夹爪，R1松开夹爪:
            if ((input.buttons & XBOX_BUTTON_L1) == XBOX_BUTTON_L1) {
                move_servo(RA_SERVO_D, -5);
            } else if ((input.buttons & XBOX_BUTTON_R1) == XBOX_BUTTON_R1) {
                move_servo(RA_SERVO_D, 5);
            }
            // 同时按下VIEW和MENU则保存当前位置:
            if ((input.ex_buttons & (XBOX_EX_BUTTON_VIEW | XBOX_EX_BUTTON_MENU)) == (XBOX_EX_BUTTON_VIEW | XBOX_EX_BUTTON_MENU)) {
                ESP_LOGI(TAG_HID, "save current position as initial.");
            }
        }
        // 每读取一次输入后，等待100ms，即每秒仅允许10次输入
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG_HID, "starting robotic arm...");
    init_chip();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    memset(&ctrl_input_data, 0, sizeof(ctrl_input_data_t));
    ctrl_input_queue = xQueueCreate(1, sizeof(ctrl_input_data_t));

    init_i2c_master();
    pca9685_set_adress(RA_I2C_ADDRESS);
    pca9685_reset();
    pca9685_set_freq(RA_PWM_FREQ);
    ESP_LOGI(TAG_PWM, "pca9685 init ok.");
    init_servos();

    char bda_str[18] = {0};
    esp_err_t ret;
#if HID_HOST_MODE == HIDH_IDLE_MODE
    ESP_LOGE(TAG_HID, "Please turn on BT HID host or BLE!");
    return;
#endif
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    ESP_LOGI(TAG_HID, "setting hid gap, mode:%d", HID_HOST_MODE);
    ESP_ERROR_CHECK( esp_hid_gap_init(HID_HOST_MODE) );
#if CONFIG_BT_BLE_ENABLED
    ESP_ERROR_CHECK( esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler) );
#endif /* CONFIG_BT_BLE_ENABLED */
    esp_hidh_config_t config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK( esp_hidh_init(&config) );

    ESP_LOGI(TAG_HID, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
#if CONFIG_BT_NIMBLE_ENABLED
    /* XXX Need to have template for store */
    ble_store_config_init();

    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
	/* Starting nimble task after gatts is initialized*/
    ret = esp_nimble_enable(ble_hid_host_task);
    if (ret) {
        ESP_LOGE(TAG_HID, "esp_nimble_enable failed: %d", ret);
    }
#endif
    xTaskCreate(&hid_scan_task, "hid_scan_task", 6 * 1024, NULL, 2, NULL);
    xTaskCreate(&control_task, "control_task", 4 * 1024, NULL, 2, NULL);
}
