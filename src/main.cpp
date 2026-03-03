#include "secrets.h"
#include "dto.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_partition.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include <string.h>

static const char *TAG = "ZIGBEE_RELAY";

#define STATUS_LED_GPIO GPIO_NUM_8
#define BOOT_BUTTON_GPIO GPIO_NUM_9
#define NUM_RELAYS 4

// Relay GPIO pins (active LOW for SSR control)
Relay RELAYS[] = {
    Relay(23),  // Channel 1 - GPIO 23
    Relay(22),  // Channel 2 - GPIO 22
    Relay(21),  // Channel 3 - GPIO 21
    Relay(20),  // Channel 4 - GPIO 20
};

// Status LED
static led_strip_handle_t status_led = NULL;
static bool zigbee_paired = false;
static bool force_pairing_requested = false;

// Message queue for relay commands
QueueHandle_t relayQueue;

// Event group for connection status
const int ZIGBEE_CONNECTED_BIT = BIT1;
EventGroupHandle_t online_event_group;

// Forward declarations
void create_zigbee_endpoints();
esp_err_t zigbee_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message);
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct);
void init_status_led();
void set_status_led(uint8_t r, uint8_t g, uint8_t b);
void TaskZigbeeStack(void *pvParameters);
void TaskRelayHandler(void *pvParameters);
void TaskStatusLED(void *pvParameters);
void TaskBootButton(void *pvParameters);

void init_gpio() {
    // Configure relay output pins
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    for (int i = 0; i < NUM_RELAYS; i++) {
        io_conf.pin_bit_mask = (1ULL << RELAYS[i].getPin());
        gpio_config(&io_conf);
        gpio_set_level((gpio_num_t) RELAYS[i].getPin(), 1); // Start OFF (active LOW)
    }

    // Configure BOOT button as input with pull-up
    gpio_config_t btn_conf = {};
    btn_conf.pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO);
    btn_conf.mode = GPIO_MODE_INPUT;
    btn_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    btn_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    btn_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&btn_conf);
}

void init_status_led() {
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = STATUS_LED_GPIO;
    strip_config.max_leds = 1;
    strip_config.led_pixel_format = LED_PIXEL_FORMAT_GRB;
    strip_config.led_model = LED_MODEL_WS2812;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.resolution_hz = 10 * 1000 * 1000; // 10 MHz

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &status_led));
    led_strip_clear(status_led);
    ESP_LOGI(TAG, "Status LED initialized on GPIO %d", STATUS_LED_GPIO);
}

void set_status_led(uint8_t r, uint8_t g, uint8_t b) {
    if (status_led) {
        led_strip_set_pixel(status_led, 0, r, g, b);
        led_strip_refresh(status_led);
    }
}

extern "C" void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    online_event_group = xEventGroupCreate();
    relayQueue = xQueueCreate(16, sizeof(RelayEvent));

    // Initialize GPIO and status LED
    init_gpio();
    init_status_led();

    ESP_LOGI(TAG, "Starting ZWitcher 4CH relay controller...");

    // Create FreeRTOS tasks
    xTaskCreate(TaskZigbeeStack, "ZigbeeStack", 8192, NULL, 5, NULL);
    xTaskCreate(TaskRelayHandler, "RelayHandler", 4096, NULL, 2, NULL);
    xTaskCreate(TaskStatusLED, "StatusLED", 2048, NULL, 1, NULL);
    xTaskCreate(TaskBootButton, "BootButton", 4096, NULL, 1, NULL);
}

// ZCL strings (length-prefixed)
static char zcl_manufacturer[32];
static char zcl_model[32];

static void init_zcl_string(char *buf, const char *str) {
    size_t len = strlen(str);
    buf[0] = (char) len;
    memcpy(buf + 1, str, len);
}

void create_zigbee_endpoints() {
    init_zcl_string(zcl_manufacturer, DEVICE_MANUFACTURER);
    init_zcl_string(zcl_model, DEVICE_MODEL_ID);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

    for (int i = 0; i < NUM_RELAYS; i++) {
        esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

        // Basic cluster (with manufacturer/model on first endpoint)
        esp_zb_basic_cluster_cfg_t basic_cfg = {
            .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
            .power_source = 0x01, // Mains powered
        };
        esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
        if (i == 0) {
            esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
                                          zcl_manufacturer);
            esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
                                          zcl_model);
        }
        esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster,
                                              ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        // Identify cluster
        esp_zb_identify_cluster_cfg_t identify_cfg = {
            .identify_time = 0,
        };
        esp_zb_cluster_list_add_identify_cluster(cluster_list,
                                                 esp_zb_identify_cluster_create(&identify_cfg),
                                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        // On/Off cluster
        esp_zb_on_off_cluster_cfg_t on_off_cfg = {
            .on_off = false, // Initial state: off
        };
        esp_zb_cluster_list_add_on_off_cluster(cluster_list,
                                               esp_zb_on_off_cluster_create(&on_off_cfg),
                                               ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        // OTA cluster (only on first endpoint)
        if (i == 0) {
            esp_zb_ota_cluster_cfg_t ota_cfg = {
                .ota_upgrade_file_version = 0x00000001,
                .ota_upgrade_manufacturer = 0x131B,
                .ota_upgrade_image_type = 0x0000,
                .ota_min_block_reque = 0,
                .ota_upgrade_file_offset = 0xFFFFFFFF,
                .ota_upgrade_downloaded_file_ver = 0xFFFFFFFF,
                .ota_upgrade_server_id = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                .ota_image_upgrade_status = 0,
            };
            esp_zb_cluster_list_add_ota_cluster(cluster_list,
                                                esp_zb_ota_cluster_create(&ota_cfg),
                                                ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
        }

        // Register endpoint
        esp_zb_endpoint_config_t ep_config = {
            .endpoint = (uint8_t)(i + 1),
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID,
            .app_device_version = 0,
        };
        esp_zb_ep_list_add_ep(ep_list, cluster_list, ep_config);

        ESP_LOGI(TAG, "Created endpoint %d for relay channel %d (GPIO %d)",
                 i + 1, i + 1, RELAYS[i].getPin());
    }

    esp_zb_device_register(ep_list);
    ESP_LOGI(TAG, "Registered %d Zigbee endpoints", NUM_RELAYS);
}

esp_err_t zigbee_action_handler(esp_zb_core_action_callback_id_t callback_id,
                                const void *message) {
    if (callback_id == ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID) {
        const esp_zb_zcl_set_attr_value_message_t *msg =
                (esp_zb_zcl_set_attr_value_message_t *) message;

        if (msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
            uint8_t endpoint = msg->info.dst_endpoint;
            uint8_t on_off = *(uint8_t *) msg->attribute.data.value;

            if (endpoint >= 1 && endpoint <= NUM_RELAYS) {
                RelayEvent event(endpoint, on_off != 0);
                xQueueSend(relayQueue, &event, portMAX_DELAY);
                ESP_LOGI(TAG, "Endpoint %d: %s", endpoint, on_off ? "ON" : "OFF");
            }
        }
    }

    return ESP_OK;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = (esp_zb_app_signal_type_t) *p_sg_p;

    switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            ESP_LOGI(TAG, "Zigbee stack initialized");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) {
                ESP_LOGI(TAG, "Device started up in %s factory-reset mode",
                         sig_type == ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START ? "" : "non");
                if (sig_type == ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START) {
                    ESP_LOGI(TAG, "Start network steering");
                    zigbee_paired = false;
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                } else {
                    ESP_LOGI(TAG, "Device rebooted - already joined network");
                    zigbee_paired = true;
                }
            } else {
                ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %d)", err_status);
                zigbee_paired = false;
            }
            break;
        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                zigbee_paired = true;
                esp_zb_ieee_addr_t extended_pan_id;
                esp_zb_get_extended_pan_id(extended_pan_id);
                ESP_LOGI(TAG,
                    "Joined network (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)",
                    extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                    extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                    esp_zb_get_pan_id(), esp_zb_get_current_channel());
            } else {
                zigbee_paired = false;
                ESP_LOGI(TAG, "Network steering was not successful (status: %d)", err_status);
                esp_zb_scheduler_alarm((esp_zb_callback_t) esp_zb_bdb_start_top_level_commissioning,
                                       ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;
        default:
            ESP_LOGI(TAG, "Zigbee signal: %d, status: %d", sig_type, err_status);
            break;
    }
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskZigbeeStack(void *pvParameters) {
    ESP_LOGI(TAG, "Initializing Zigbee stack...");

    esp_zb_cfg_t zb_nwk_cfg = {};
    zb_nwk_cfg.esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER;
    zb_nwk_cfg.install_code_policy = false;
    zb_nwk_cfg.nwk_cfg.zczr_cfg.max_children = 10;

    esp_zb_init(&zb_nwk_cfg);
    create_zigbee_endpoints();
    esp_zb_core_action_handler_register(zigbee_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));
    xEventGroupSetBits(online_event_group, ZIGBEE_CONNECTED_BIT);

    ESP_LOGI(TAG, "Zigbee stack started, entering main loop...");
    esp_zb_stack_main_loop();
}

void TaskRelayHandler(void *pvParameters) {
    while (true) {
        RelayEvent event;
        if (xQueueReceive(relayQueue, &event, portMAX_DELAY)) {
            uint8_t ep = event.getEndpoint();
            bool state = event.getState();
            int idx = ep - 1; // endpoint 1-4 → index 0-3

            if (idx >= 0 && idx < NUM_RELAYS) {
                RELAYS[idx].setState(state);
                gpio_set_level((gpio_num_t) RELAYS[idx].getPin(), state ? 0 : 1);

                // Report On/Off state back to coordinator
                esp_zb_zcl_report_attr_cmd_t report_cmd = {};
                report_cmd.zcl_basic_cmd.src_endpoint = ep;
                report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
                report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
                report_cmd.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
                esp_zb_zcl_report_attr_cmd_req(&report_cmd);

                ESP_LOGI(TAG, "Relay %d (GPIO %d): %s", idx + 1,
                         RELAYS[idx].getPin(), state ? "ON" : "OFF");
            }
        }
    }
}

void TaskStatusLED(void *pvParameters) {
    bool once = false;
    while (true) {
        if (!zigbee_paired) {
            set_status_led(0, 0, 10); // Blue blink while pairing
            vTaskDelay(pdMS_TO_TICKS(500));
        } else if (!once) {
            set_status_led(0, 10, 0); // Dim green when paired
            vTaskDelay(pdMS_TO_TICKS(1000));
            once = true;
        }
        set_status_led(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void TaskBootButton(void *pvParameters) {
    const int HOLD_TIME_MS = 3000;
    const int POLL_INTERVAL_MS = 50;
    int hold_counter = 0;

    while (true) {
        if (gpio_get_level(BOOT_BUTTON_GPIO) == 0) {
            hold_counter += POLL_INTERVAL_MS;

            if (hold_counter >= HOLD_TIME_MS && !force_pairing_requested) {
                ESP_LOGI(TAG, "BOOT button held for 3 seconds - forcing pairing mode");
                force_pairing_requested = true;
                zigbee_paired = false;

                const esp_partition_t *zb_part = esp_partition_find_first(
                    ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "zb_storage");
                if (zb_part) {
                    ESP_LOGI(TAG, "Erasing zb_storage partition...");
                    esp_partition_erase_range(zb_part, 0, zb_part->size);
                }

                const esp_partition_t *zb_fct = esp_partition_find_first(
                    ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "zb_fct");
                if (zb_fct) {
                    ESP_LOGI(TAG, "Erasing zb_fct partition...");
                    esp_partition_erase_range(zb_fct, 0, zb_fct->size);
                }

                esp_restart();
            }
        } else {
            hold_counter = 0;
            if (force_pairing_requested && zigbee_paired) {
                force_pairing_requested = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}
