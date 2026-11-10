#include "secrets.h"
#include "dto.h"
#include "ota_element_parser.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "driver/gpio.h"
#include "driver/temperature_sensor.h"
#include "led_strip.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

static const char *TAG = "ZIGBEE_RELAY";

// Bump for every OTA release; served image version must be greater
#define OTA_FW_VERSION 0x00000007
#define OTA_MANUFACTURER_CODE 0x131B
// Product discriminator: zwitcher-lamp uses 0x0000 under the same manufacturer.
#define OTA_IMAGE_TYPE 0x0001

// Bump when the endpoint/cluster layout changes: stale zb_storage reporting config crashes ZBOSS after OTA
#define ZB_LAYOUT_VERSION 1

#define STATUS_LED_GPIO GPIO_NUM_8
#define BOOT_BUTTON_GPIO GPIO_NUM_9
#define NUM_RELAYS 4
#define OTA_ENDPOINT 1  // OTA cluster lives on the first relay endpoint
#define TEMP_ENDPOINT 5 // Relays own endpoints 1-4

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
static temperature_sensor_handle_t temp_sensor = NULL;

// A single snapshot coalesces bursts without blocking the Zigbee callback.
struct RelayStates {
    bool states[NUM_RELAYS] = {};
};
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
void TaskTemperature(void *pvParameters);

void init_gpio() {
    // Configure relay output pins
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    for (int i = 0; i < NUM_RELAYS; i++) {
        io_conf.pin_bit_mask = (1ULL << RELAYS[i].getPin());
        // Preload OFF before enabling the active-low output.
        ESP_ERROR_CHECK(gpio_set_level((gpio_num_t) RELAYS[i].getPin(), 1));
        ESP_ERROR_CHECK(gpio_config(&io_conf));
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
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
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

// Erase zb_storage when the compiled endpoint layout differs from the stored one
static void reset_zb_storage_on_layout_change() {
    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("app", NVS_READWRITE, &nvs));
    uint32_t stored = 0;
    if (nvs_get_u32(nvs, "zb_layout", &stored) != ESP_OK || stored != ZB_LAYOUT_VERSION) {
        const esp_partition_t *part = esp_partition_find_first(
            ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "zb_storage");
        if (part) {
            ESP_LOGW(TAG, "Zigbee layout changed (v%u -> v%d): erasing zb_storage, re-pairing needed",
                     (unsigned) stored, ZB_LAYOUT_VERSION);
            ESP_ERROR_CHECK(esp_partition_erase_range(part, 0, part->size));
        }
        ESP_ERROR_CHECK(nvs_set_u32(nvs, "zb_layout", ZB_LAYOUT_VERSION));
        ESP_ERROR_CHECK(nvs_commit(nvs));
    }
    nvs_close(nvs);
}

extern "C" void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    reset_zb_storage_on_layout_change();

    online_event_group = xEventGroupCreate();
    relayQueue = xQueueCreate(1, sizeof(RelayStates));
    configASSERT(relayQueue);

    // Initialize GPIO and status LED
    init_gpio();
    init_status_led();

    ESP_LOGI(TAG, "Starting ZWitcher 4CH relay controller...");

    // Create FreeRTOS tasks
    xTaskCreate(TaskZigbeeStack, "ZigbeeStack", 8192, NULL, 5, NULL);
    xTaskCreate(TaskRelayHandler, "RelayHandler", 4096, NULL, 2, NULL);
    xTaskCreate(TaskStatusLED, "StatusLED", 2048, NULL, 1, NULL);
    xTaskCreate(TaskBootButton, "BootButton", 4096, NULL, 1, NULL);
    xTaskCreate(TaskTemperature, "Temperature", 3072, NULL, 1, NULL);
}

// ZCL strings (length-prefixed)
static char zcl_manufacturer[32];
static char zcl_model[32];
static char zcl_sw_build[16];

static void init_zcl_string(char *buf, size_t buf_size, const char *str) {
    size_t len = strlen(str);
    if (len > buf_size - 1) {
        len = buf_size - 1;
    }
    buf[0] = (char) len;
    memcpy(buf + 1, str, len);
}

void create_zigbee_endpoints() {
    init_zcl_string(zcl_manufacturer, sizeof(zcl_manufacturer), DEVICE_MANUFACTURER);
    init_zcl_string(zcl_model, sizeof(zcl_model), DEVICE_MODEL_ID);
    char sw_ver[12];
    snprintf(sw_ver, sizeof(sw_ver), "%08x", (unsigned) OTA_FW_VERSION);
    init_zcl_string(zcl_sw_build, sizeof(zcl_sw_build), sw_ver);

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
            esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID,
                                          zcl_sw_build);
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
                .ota_upgrade_file_version = OTA_FW_VERSION,
                .ota_upgrade_manufacturer = OTA_MANUFACTURER_CODE,
                .ota_upgrade_image_type = OTA_IMAGE_TYPE,
                .ota_min_block_reque = 0,
                .ota_upgrade_file_offset = 0xFFFFFFFF,
                .ota_upgrade_downloaded_file_ver = 0xFFFFFFFF,
                .ota_upgrade_server_id = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                .ota_image_upgrade_status = 0,
            };
            esp_zb_attribute_list_t *ota_cluster = esp_zb_ota_cluster_create(&ota_cfg);
            // Stack keeps this pointer - must outlive the cluster
            static esp_zb_zcl_ota_upgrade_client_variable_t ota_client_var = {
                .timer_query = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF,
                .hw_version = 0x0101,
                .max_data_size = 223, // Max ZCL payload; fewer round-trips per image
            };
            esp_zb_ota_cluster_add_attr(ota_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID,
                                        &ota_client_var);
            esp_zb_cluster_list_add_ota_cluster(cluster_list, ota_cluster,
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

    // Endpoint 5 - internal chip temperature sensor
    esp_zb_cluster_list_t *temp_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_basic_cluster_cfg_t temp_basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x01,
    };
    esp_zb_cluster_list_add_basic_cluster(temp_cluster_list,
                                          esp_zb_basic_cluster_create(&temp_basic_cfg),
                                          ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_identify_cluster_cfg_t temp_identify_cfg = {
        .identify_time = 0,
    };
    esp_zb_cluster_list_add_identify_cluster(temp_cluster_list,
                                             esp_zb_identify_cluster_create(&temp_identify_cfg),
                                             ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_temperature_meas_cluster_cfg_t temp_meas_cfg = {
        .measured_value = 2500, // 0.01°C units, placeholder until first read
        .min_value = -1000,
        .max_value = 8000,
    };
    esp_zb_cluster_list_add_temperature_meas_cluster(temp_cluster_list,
                                                     esp_zb_temperature_meas_cluster_create(&temp_meas_cfg),
                                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_endpoint_config_t temp_ep_config = {
        .endpoint = TEMP_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, temp_cluster_list, temp_ep_config);

    esp_zb_device_register(ep_list);
    ESP_LOGI(TAG, "Registered %d relay endpoints + temperature endpoint %d", NUM_RELAYS, TEMP_ENDPOINT);
}

static esp_err_t attr_value_handler(const esp_zb_zcl_set_attr_value_message_t *msg) {
    static RelayStates requested;
    if (!msg || msg->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        return ESP_ERR_INVALID_ARG;
    }
    if (msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF &&
        msg->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID &&
        msg->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL &&
        msg->attribute.data.value) {
        uint8_t endpoint = msg->info.dst_endpoint;
        uint8_t on_off = *(uint8_t *) msg->attribute.data.value;

        if (endpoint >= 1 && endpoint <= NUM_RELAYS) {
            requested.states[endpoint - 1] = on_off != 0;
            xQueueOverwrite(relayQueue, &requested);
            ESP_LOGI(TAG, "Endpoint %d: %s", endpoint, on_off ? "ON" : "OFF");
        }
    }

    return ESP_OK;
}

// OTA download state
static const esp_partition_t *ota_partition = NULL;
static esp_ota_handle_t ota_handle = 0;
static uint32_t ota_total_size = 0;
static uint32_t ota_received = 0;
static OtaElementParser ota_parser;

static void reset_ota_download() {
    if (ota_handle) {
        esp_ota_abort(ota_handle);
        ota_handle = 0;
    }
    ota_partition = NULL;
    ota_total_size = 0;
    ota_received = 0;
    ota_parser = OtaElementParser{};
}

static esp_err_t ota_upgrade_status_handler(const esp_zb_zcl_ota_upgrade_value_message_t *msg) {
    esp_err_t ret = ESP_OK;

    if (!msg) {
        return ESP_ERR_INVALID_ARG;
    }
    // Terminal events must clean up even when their common status is a failure.
    if (msg->upgrade_status == ESP_ZB_ZCL_OTA_UPGRADE_STATUS_ABORT ||
        msg->upgrade_status == ESP_ZB_ZCL_OTA_UPGRADE_STATUS_ERROR) {
        ESP_LOGW(TAG, "OTA download stopped (status: %d)", msg->upgrade_status);
        reset_ota_download();
        return ESP_OK;
    }
    if (msg->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        reset_ota_download();
        return ESP_FAIL;
    }

    switch (msg->upgrade_status) {
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
            ESP_LOGI(TAG, "OTA upgrade started");
            reset_ota_download();
            ota_partition = esp_ota_get_next_update_partition(NULL);
            ESP_RETURN_ON_FALSE(ota_partition, ESP_FAIL, TAG, "No OTA update partition");

            ret = esp_ota_begin(ota_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE:
            if (!ota_handle || (msg->payload_size && !msg->payload)) {
                ret = ESP_ERR_INVALID_STATE;
                break;
            }
            ota_total_size = msg->ota_header.image_size;
            if (msg->payload_size) {
                const uint8_t *data = NULL;
                uint16_t len = 0;
                ret = ota_parser.consume(ota_total_size, msg->payload, msg->payload_size, &data, &len)
                        ? ESP_OK : ESP_ERR_INVALID_ARG;
                if (ret == ESP_OK && len > 0) {
                    ret = esp_ota_write(ota_handle, data, len);
                }
                if (ret != ESP_OK) {
                    break;
                }
                uint32_t before = ota_received;
                ota_received += msg->payload_size;
                // Log every ~5%: per-block UART writes stall the Zigbee task
                if (ota_total_size > 0 &&
                    (before * 20 / ota_total_size) != (ota_received * 20 / ota_total_size)) {
                    ESP_LOGI(TAG, "OTA progress: %u/%u bytes",
                             (unsigned) ota_received, (unsigned) ota_total_size);
                }
            }
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_APPLY:
            ESP_LOGI(TAG, "OTA upgrade apply");
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
            ret = (ota_handle && ota_parser.complete()) ? ESP_OK : ESP_FAIL;
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH:
            if (!ota_handle || !ota_parser.complete()) {
                ret = ESP_ERR_INVALID_STATE;
                break;
            }
            ret = esp_ota_end(ota_handle);
            ota_handle = 0; // esp_ota_end frees the session even if validation fails.
            if (ret == ESP_OK) {
                ret = esp_ota_set_boot_partition(ota_partition);
            }
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "OTA upgrade finished, rebooting to %s", ota_partition->label);
                esp_restart();
            }
            ESP_LOGE(TAG, "OTA finalize failed: %s", esp_err_to_name(ret));
            break;

        default:
            break;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OTA download failed: %s", esp_err_to_name(ret));
        reset_ota_download();
    }
    return ret;
}

static esp_err_t ota_query_image_resp_handler(const esp_zb_zcl_ota_upgrade_query_image_resp_message_t *msg) {
    if (!msg || msg->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        return ESP_FAIL;
    }
    // Server reached - relax the query cadence to hourly
    esp_zb_ota_upgrade_client_query_interval_set(OTA_ENDPOINT, 60);
    if (msg->query_status == ESP_ZB_ZCL_STATUS_SUCCESS) {
        if (msg->manufacturer_code != OTA_MANUFACTURER_CODE || msg->image_type != OTA_IMAGE_TYPE) {
            ESP_LOGW(TAG, "Rejecting OTA image for manufacturer 0x%04x, type 0x%04x",
                     msg->manufacturer_code, msg->image_type);
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "OTA image available: version 0x%x, size %u - approving",
                 (unsigned) msg->file_version, (unsigned) msg->image_size);
    }
    return ESP_OK; // ESP_OK approves the download
}

esp_err_t zigbee_action_handler(esp_zb_core_action_callback_id_t callback_id,
                                const void *message) {
    switch (callback_id) {
        case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
            return attr_value_handler((const esp_zb_zcl_set_attr_value_message_t *) message);
        case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID:
            return ota_upgrade_status_handler((const esp_zb_zcl_ota_upgrade_value_message_t *) message);
        case ESP_ZB_CORE_OTA_UPGRADE_QUERY_IMAGE_RESP_CB_ID:
            return ota_query_image_resp_handler((const esp_zb_zcl_ota_upgrade_query_image_resp_message_t *) message);
        default:
            return ESP_OK;
    }
}

static void start_steering_cb(uint8_t mode) {
    esp_zb_bdb_start_top_level_commissioning(mode);
}

// Stack-managed periodic query: first within a minute, relaxed to hourly after server contact
static void schedule_ota_query() {
    esp_zb_ota_upgrade_client_query_interval_set(OTA_ENDPOINT, 1);
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
                    schedule_ota_query();
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
                schedule_ota_query();
            } else {
                zigbee_paired = false;
                ESP_LOGI(TAG, "Network steering was not successful (status: %d)", err_status);
                esp_zb_scheduler_alarm(start_steering_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;
        case ESP_ZB_ZDO_SIGNAL_LEAVE: {
            esp_zb_zdo_signal_leave_params_t *leave_params =
                    (esp_zb_zdo_signal_leave_params_t *) esp_zb_app_signal_get_params(p_sg_p);
            zigbee_paired = false;
            ESP_LOGI(TAG, "Left network (leave type: %d), restarting steering",
                     leave_params ? leave_params->leave_type : -1);
            esp_zb_scheduler_alarm(start_steering_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            break;
        }
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
        RelayStates requested;
        if (xQueueReceive(relayQueue, &requested, portMAX_DELAY)) {
            for (int idx = 0; idx < NUM_RELAYS; idx++) {
                uint8_t ep = idx + 1;
                bool state = requested.states[idx];
                if (RELAYS[idx].getState() == state) {
                    continue;
                }
                RELAYS[idx].setState(state);
                gpio_set_level((gpio_num_t) RELAYS[idx].getPin(), state ? 0 : 1);

                // Report On/Off state back to coordinator
                // Stack lock required: this runs outside the Zigbee task
                esp_zb_zcl_report_attr_cmd_t report_cmd = {};
                report_cmd.zcl_basic_cmd.src_endpoint = ep;
                report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
                report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
                report_cmd.attributeID = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID;
                report_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
                esp_zb_lock_acquire(portMAX_DELAY);
                esp_zb_zcl_report_attr_cmd_req(&report_cmd);
                esp_zb_lock_release();

                ESP_LOGI(TAG, "Relay %d (GPIO %d): %s", idx + 1,
                         RELAYS[idx].getPin(), state ? "ON" : "OFF");
            }
        }
    }
}

void TaskStatusLED(void *pvParameters) {
    uint32_t tick = 0;
    while (true) {
        if (force_pairing_requested) {
            // Rapid red blink: release BOOT to factory reset
            set_status_led((tick % 2) ? 15 : 0, 0, 0);
        } else if (!zigbee_paired) {
            // Blue blink (500ms) while searching for a network
            set_status_led(0, 0, (tick % 8 < 4) ? 30 : 0);
        } else {
            set_status_led(0, (tick % 30 < 3) ? 30 : 0, 0);
        }
        tick++;
        vTaskDelay(pdMS_TO_TICKS(125));
    }
}

void TaskBootButton(void *pvParameters) {
    const int HOLD_TIME_MS = 3000;
    const int POLL_INTERVAL_MS = 50;
    int hold_counter = 0;

    while (true) {
        // BOOT button is active LOW (pressed = 0)
        if (gpio_get_level(BOOT_BUTTON_GPIO) == 0) {
            hold_counter += POLL_INTERVAL_MS;

            if (hold_counter >= HOLD_TIME_MS && !force_pairing_requested) {
                ESP_LOGI(TAG, "BOOT held 3s - factory reset pending, release button");
                force_pairing_requested = true;
                zigbee_paired = false;
            }
        } else {
            if (force_pairing_requested) {
                // Reset only after release: rebooting with GPIO9 low enters ROM download mode
                ESP_LOGI(TAG, "Factory resetting Zigbee network...");
                vTaskDelay(pdMS_TO_TICKS(100)); // Debounce
                esp_zb_lock_acquire(portMAX_DELAY);
                esp_zb_factory_reset(); // Erases zb_storage and reboots
                esp_zb_lock_release();
            }
            hold_counter = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}

void TaskTemperature(void *pvParameters) {
    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    if (temperature_sensor_install(&cfg, &temp_sensor) != ESP_OK ||
        temperature_sensor_enable(temp_sensor) != ESP_OK) {
        ESP_LOGE(TAG, "Temperature sensor init failed");
        vTaskDelete(NULL);
    }

    xEventGroupWaitBits(online_event_group, ZIGBEE_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    while (true) {
        float celsius;
        if (temperature_sensor_get_celsius(temp_sensor, &celsius) == ESP_OK) {
            int16_t value = (int16_t) lroundf(celsius * 100.0f); // ZCL: 0.01°C units
            esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_zcl_set_attribute_val(TEMP_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                                         ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                         ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                                         &value, false);
            esp_zb_lock_release();
        }
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}
