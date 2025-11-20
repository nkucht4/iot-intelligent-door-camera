/*
  - Device name: "LAB31 - KEYBOARD"
  - HID service 0x1812 with:
      HID Information (0x2A4A) READ
      Protocol Mode (0x2A4E) READ, WRITE NO RESPONSE
      Report Map (0x2A4B) READ
      Report (0x2A4D) Report ID 1 (NOTIFY, READ) + CCCD + Report Reference
      Report (0x2A4D) Report ID 2 (NOTIFY, READ) + CCCD + Report Reference
      Report (0x2A4D) Report ID 3 (READ/WRITE/WRITE_NO_RESP) + Report Reference
      Boot Keyboard Input (0x2A22) NOTIFY, READ + CCCD
      Boot Keyboard Output (0x2A32) READ, WRITE, WRITE NO RESPONSE (LED)
  - Sends random key presses once per second (Report ID 1 + Boot Input)
  - Logs writes to Boot Output (LEDs) and Report3
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"

static const char *TAG = "LAB31_HID";

#define HID_SERVICE_UUID           0x1812
#define HID_INFO_CHAR_UUID         0x2A4A
#define PROTOCOL_MODE_UUID         0x2A4E
#define REPORT_MAP_UUID            0x2A4B
#define REPORT_CHAR_UUID           0x2A4D
#define BOOT_KEYBOARD_INPUT_UUID   0x2A22
#define BOOT_KEYBOARD_OUTPUT_UUID  0x2A32
#define REPORT_REF_DESC_UUID       0x2908
#define CLIENT_CHAR_CFG_UUID       0x2902

#define GATTS_NUM_HANDLE 40
#define PROFILE_APP_ID 0

/* HID values */
static uint8_t hid_info_value[4] = { 0x11, 0x01, 0x00, 0x02 }; // bcdHID 1.11, country 0, flags
static uint8_t protocol_mode = 1; // report mode (1)

/* Report Map: includes Report ID 1 (keyboard) and Report ID 2 (simple vendor) */
static uint8_t report_map[] = {
    // Report ID 1 - Keyboard (mod + reserved + 6 keys)
    0x05,0x01, 0x09,0x06, 0xA1,0x01,
      0x85,0x01,
      0x05,0x07, 0x19,0xE0, 0x29,0xE7, 0x15,0x00, 0x25,0x01, 0x75,0x01, 0x95,0x08, 0x81,0x02,
      0x95,0x01, 0x75,0x08, 0x81,0x03,
      0x95,0x05, 0x75,0x01, 0x05,0x08, 0x19,0x01, 0x29,0x05, 0x91,0x02,
      0x95,0x01, 0x75,0x03, 0x91,0x03,
      0x95,0x06, 0x75,0x08, 0x15,0x00, 0x25,0x65, 0x05,0x07, 0x19,0x00, 0x29,0x65, 0x81,0x00,
    0xC0,

    // Report ID 2 - Vendor/extra input (simple)
    0x05,0x01, 0x09,0x06, 0xA1,0x01,
      0x85,0x02,
      0x09,0x00, 0x15,0x00, 0x25,0xFF, 0x75,0x08, 0x95,0x04, 0x81,0x02,
    0xC0
};

/* Boot input (standard boot report format): modifier, reserved, 6 keycodes */
static uint8_t boot_input[8] = {0};

/* advertisement */
static uint8_t adv_service_uuid128[16] = {
    0x12,0x34,0x56,0x78,0x9a,0xbc,0xde,0xf0,
    0x12,0x34,0x56,0x78,0x12,0x12,0x12,0x12
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT)
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* Hid handles structure (stores discovered handles) */
struct hid_handles_t {
    uint16_t service_handle;
    uint16_t hid_info_handle;
    uint16_t protocol_mode_handle;
    uint16_t report_map_handle;

    uint16_t report1_handle;
    uint16_t report1_cccd_handle;
    uint16_t report1_report_ref_handle;

    uint16_t report2_handle;
    uint16_t report2_cccd_handle;
    uint16_t report2_report_ref_handle;

    uint16_t report3_handle;
    uint16_t report3_report_ref_handle;

    uint16_t boot_input_handle;
    uint16_t boot_input_cccd_handle;

    uint16_t boot_output_handle;

    uint16_t gatts_if;
    uint16_t conn_id;

    int added_chars;
    int added_descr;
} hid = {0};

/* helper to create 16-bit uuid obj */
static inline esp_bt_uuid_t mk_uuid16(uint16_t u) {
    esp_bt_uuid_t id;
    id.len = ESP_UUID_LEN_16;
    id.uuid.uuid16 = u;
    return id;
}

/* forward */
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* send random key on Report1 and Boot Input */
static void send_random_key_reports(void)
{
    if (hid.conn_id == 0xFFFF) return;

    uint8_t mod = (uint8_t)(rand() & 0x07);       // demo: some modifier bits
    uint8_t key = 0x04 + (rand() % 26);          // a..z usage codes

    // REPORT ID 1 payload: first byte = Report ID (1)
    uint8_t report1_payload[8];
    memset(report1_payload, 0, sizeof(report1_payload));
    report1_payload[0] = 0x01;   // Report ID 1
    report1_payload[1] = mod;    // treat index 1 as modifier in this mapping
    report1_payload[2] = key;    // first key

    esp_err_t err = esp_ble_gatts_send_indicate(hid.gatts_if, hid.conn_id, hid.report1_handle, sizeof(report1_payload), report1_payload, false);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Sent Report1 ID1 key 0x%02x mod 0x%02x", key, mod);
    } else {
        ESP_LOGW(TAG, "Report1 send failed: %s", esp_err_to_name(err));
    }

    // Boot input (standard 8 bytes)
    memset(boot_input, 0, sizeof(boot_input));
    boot_input[0] = mod;
    boot_input[2] = key;
    err = esp_ble_gatts_send_indicate(hid.gatts_if, hid.conn_id, hid.boot_input_handle, sizeof(boot_input), boot_input, false);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Sent Boot Input key 0x%02x mod 0x%02x", key, mod);
    } else {
        ESP_LOGW(TAG, "Boot Input send failed: %s", esp_err_to_name(err));
    }

    // release shortly after
    vTaskDelay(pdMS_TO_TICKS(50));
    memset(boot_input, 0, sizeof(boot_input));
    esp_ble_gatts_send_indicate(hid.gatts_if, hid.conn_id, hid.boot_input_handle, sizeof(boot_input), boot_input, false);

    memset(report1_payload, 0, sizeof(report1_payload));
    esp_ble_gatts_send_indicate(hid.gatts_if, hid.conn_id, hid.report1_handle, sizeof(report1_payload), report1_payload, false);
}

/* FreeRTOS task sending random keys */
static void random_sender_task(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        send_random_key_reports();
    }
}

/* GAP handler - start advertising after adv data set */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    if (event == ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT) {
        esp_ble_gap_start_advertising(&adv_params);
        ESP_LOGI(TAG, "Advertising started");
    }
}

/* Main GATT profile handler */
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT: {
        ESP_LOGI(TAG, "GATTS_REG_EVT");
        hid.gatts_if = gatts_if;
        hid.conn_id = 0xFFFF;
        hid.added_chars = 0;
        hid.added_descr = 0;

        esp_ble_gap_set_device_name("LAB31 - KEYBOARD");
        esp_ble_gap_config_adv_data(&adv_data);

        esp_ble_gatts_create_service(gatts_if, &(esp_gatt_srvc_id_t){
            .is_primary = true,
            .id = {.inst_id = 0, .uuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = HID_SERVICE_UUID}}}
        }, GATTS_NUM_HANDLE);
        break;
    }

    case ESP_GATTS_CREATE_EVT: {
        ESP_LOGI(TAG, "GATTS_CREATE_EVT svc_handle=%d", param->create.service_handle);
        hid.service_handle = param->create.service_handle;

        // 1) HID Information
        {
            esp_bt_uuid_t u = mk_uuid16(HID_INFO_CHAR_UUID);
            esp_attr_value_t a = {.attr_max_len = sizeof(hid_info_value), .attr_len = sizeof(hid_info_value), .attr_value = hid_info_value};
            esp_ble_gatts_add_char(hid.service_handle, &u, ESP_GATT_PERM_READ, ESP_GATT_CHAR_PROP_BIT_READ, &a, NULL);
        }

        // 2) Protocol Mode (read, write no resp)
        {
            esp_bt_uuid_t u = mk_uuid16(PROTOCOL_MODE_UUID);
            esp_attr_value_t a = {.attr_max_len = 1, .attr_len = 1, .attr_value = &protocol_mode};
            esp_ble_gatts_add_char(hid.service_handle, &u, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE_NR, &a, NULL);
        }

        // 3) Report Map (read)
        {
            esp_bt_uuid_t u = mk_uuid16(REPORT_MAP_UUID);
            esp_attr_value_t a = {.attr_max_len = sizeof(report_map), .attr_len = sizeof(report_map), .attr_value = report_map};
            esp_ble_gatts_add_char(hid.service_handle, &u, ESP_GATT_PERM_READ, ESP_GATT_CHAR_PROP_BIT_READ, &a, NULL);
        }

        // 4) Report 1 (Report ID 1): NOTIFY, READ
        {
            esp_bt_uuid_t u = mk_uuid16(REPORT_CHAR_UUID);
            esp_attr_value_t a = {.attr_max_len = 8, .attr_len = 0, .attr_value = NULL};
            esp_ble_gatts_add_char(hid.service_handle, &u, ESP_GATT_PERM_READ, ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY, &a, NULL);
        }

        // 5) Report 2 (Report ID 2): NOTIFY, READ
        {
            esp_bt_uuid_t u = mk_uuid16(REPORT_CHAR_UUID);
            esp_attr_value_t a = {.attr_max_len = 4, .attr_len = 0, .attr_value = NULL};
            esp_ble_gatts_add_char(hid.service_handle, &u, ESP_GATT_PERM_READ, ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY, &a, NULL);
        }

        // 6) Report 3 (configurable) READ/WRITE/WRITE_NO_RESP
        {
            esp_bt_uuid_t u = mk_uuid16(REPORT_CHAR_UUID);
            esp_attr_value_t a = {.attr_max_len = 16, .attr_len = 0, .attr_value = NULL};
            esp_ble_gatts_add_char(hid.service_handle, &u, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR, &a, NULL);
        }

        // 7) Boot Keyboard Input (notify + read)
        {
            esp_bt_uuid_t u = mk_uuid16(BOOT_KEYBOARD_INPUT_UUID);
            esp_attr_value_t a = {.attr_max_len = sizeof(boot_input), .attr_len = sizeof(boot_input), .attr_value = boot_input};
            esp_ble_gatts_add_char(hid.service_handle, &u, ESP_GATT_PERM_READ, ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY, &a, NULL);
        }

        // 8) Boot Keyboard Output (read/write)
        {
            esp_bt_uuid_t u = mk_uuid16(BOOT_KEYBOARD_OUTPUT_UUID);
            uint8_t init_out[1] = {0};
            esp_attr_value_t a = {.attr_max_len = 1, .attr_len = 1, .attr_value = init_out};
            esp_ble_gatts_add_char(hid.service_handle, &u, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR, &a, NULL);
        }

        break;
    }

    case ESP_GATTS_ADD_CHAR_EVT: {
        uint16_t uuid16 = param->add_char.char_uuid.uuid.uuid16;
        hid.added_chars++;
        ESP_LOGI(TAG, "ADD_CHAR_EVT handle=%d uuid=0x%04x added_count=%d", param->add_char.attr_handle, uuid16, hid.added_chars);

        if (uuid16 == HID_INFO_CHAR_UUID) {
            hid.hid_info_handle = param->add_char.attr_handle;
            // no descriptors
        } else if (uuid16 == PROTOCOL_MODE_UUID) {
            hid.protocol_mode_handle = param->add_char.attr_handle;
            // no descriptors
        } else if (uuid16 == REPORT_MAP_UUID) {
            hid.report_map_handle = param->add_char.attr_handle;
            // no descriptors
        } else if (uuid16 == REPORT_CHAR_UUID) {
            // There are three Report chars added in sequence; we need to disambiguate by order:
            // - first REPORT_CHAR we encounter -> Report1 (ID1) -> add CCCD + Report Reference (ID=1, type=input)
            // - second -> Report2 (ID2) -> add CCCD + Report Reference (ID=2, type=input)
            // - third -> Report3 (configurable) -> add Report Reference (ID=3, type=output or vendor - we use input(1)/output(2) as needed)
            if (!hid.report1_handle) {
                hid.report1_handle = param->add_char.attr_handle;
                // add CCCD
                esp_bt_uuid_t cccd = mk_uuid16(CLIENT_CHAR_CFG_UUID);
                esp_attr_value_t cccd_attr = { .attr_max_len = 2, .attr_len = 0, .attr_value = NULL };
                esp_ble_gatts_add_char_descr(hid.service_handle, &cccd, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, &cccd_attr, NULL);
                // add Report Reference (ID=1, type=1=input)
                uint8_t ref1[2] = { 0x01, 0x01 };
                esp_bt_uuid_t rr = mk_uuid16(REPORT_REF_DESC_UUID);
                esp_attr_value_t rr_attr = { .attr_max_len = 2, .attr_len = 2, .attr_value = ref1 };
                esp_ble_gatts_add_char_descr(hid.service_handle, &rr, ESP_GATT_PERM_READ, &rr_attr, NULL);
            } else if (!hid.report2_handle) {
                hid.report2_handle = param->add_char.attr_handle;
                esp_bt_uuid_t cccd = mk_uuid16(CLIENT_CHAR_CFG_UUID);
                esp_attr_value_t cccd_attr = { .attr_max_len = 2, .attr_len = 0, .attr_value = NULL };
                esp_ble_gatts_add_char_descr(hid.service_handle, &cccd, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, &cccd_attr, NULL);
                uint8_t ref2[2] = { 0x02, 0x01 };
                esp_bt_uuid_t rr = mk_uuid16(REPORT_REF_DESC_UUID);
                esp_attr_value_t rr_attr = { .attr_max_len = 2, .attr_len = 2, .attr_value = ref2 };
                esp_ble_gatts_add_char_descr(hid.service_handle, &rr, ESP_GATT_PERM_READ, &rr_attr, NULL);
            } else {
                hid.report3_handle = param->add_char.attr_handle;
                // report3: add report reference only (e.g. ID=3, type=2 (output) or 1 (input) — pick 0x03, type=0x02 if it's output/config)
                uint8_t ref3[2] = { 0x03, 0x02 }; // choose type 2 (output) to indicate writable report
                esp_bt_uuid_t rr = mk_uuid16(REPORT_REF_DESC_UUID);
                esp_attr_value_t rr_attr = { .attr_max_len = 2, .attr_len = 2, .attr_value = ref3 };
                esp_ble_gatts_add_char_descr(hid.service_handle, &rr, ESP_GATT_PERM_READ, &rr_attr, NULL);
            }
        } else if (uuid16 == BOOT_KEYBOARD_INPUT_UUID) {
            hid.boot_input_handle = param->add_char.attr_handle;
            // add CCCD for notify
            esp_bt_uuid_t cccd = mk_uuid16(CLIENT_CHAR_CFG_UUID);
            esp_attr_value_t cccd_attr = { .attr_max_len = 2, .attr_len = 0, .attr_value = NULL };
            esp_ble_gatts_add_char_descr(hid.service_handle, &cccd, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, &cccd_attr, NULL);
        } else if (uuid16 == BOOT_KEYBOARD_OUTPUT_UUID) {
            hid.boot_output_handle = param->add_char.attr_handle;
            // no descriptor
        } else {
            ESP_LOGW(TAG, "Unknown char UUID added: 0x%04x", uuid16);
        }
        break;
    }

    case ESP_GATTS_ADD_CHAR_DESCR_EVT: {
        hid.added_descr++;
        ESP_LOGI(TAG, "ADD_CHAR_DESCR_EVT handle=%d added_descr=%d", param->add_char_descr.attr_handle, hid.added_descr);

        // Map descriptors in the expected order:
        // 1) report1 CCCD
        // 2) report1 report_ref
        // 3) report2 CCCD
        // 4) report2 report_ref
        // 5) report3 report_ref
        // 6) boot_input CCCD
        if (hid.added_descr == 1) hid.report1_cccd_handle = param->add_char_descr.attr_handle;
        else if (hid.added_descr == 2) hid.report1_report_ref_handle = param->add_char_descr.attr_handle;
        else if (hid.added_descr == 3) hid.report2_cccd_handle = param->add_char_descr.attr_handle;
        else if (hid.added_descr == 4) hid.report2_report_ref_handle = param->add_char_descr.attr_handle;
        else if (hid.added_descr == 5) hid.report3_report_ref_handle = param->add_char_descr.attr_handle;
        else if (hid.added_descr == 6) {
            hid.boot_input_cccd_handle = param->add_char_descr.attr_handle;
            // After we have added all descriptors (6 expected), start the service
            ESP_LOGI(TAG, "All descriptors added -> starting service");
            esp_ble_gatts_start_service(hid.service_handle);
        }
        break;
    }

    case ESP_GATTS_START_EVT:
        ESP_LOGI(TAG, "SERVICE STARTED");
        break;

    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(TAG, "READ_EVT handle=%d", param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(rsp));
        rsp.attr_value.handle = param->read.handle;

        if (param->read.handle == hid.hid_info_handle) {
            memcpy(rsp.attr_value.value, hid_info_value, sizeof(hid_info_value));
            rsp.attr_value.len = sizeof(hid_info_value);
        } else if (param->read.handle == hid.protocol_mode_handle) {
            rsp.attr_value.value[0] = protocol_mode;
            rsp.attr_value.len = 1;
        } else if (param->read.handle == hid.report_map_handle) {
            memcpy(rsp.attr_value.value, report_map, sizeof(report_map));
            rsp.attr_value.len = sizeof(report_map);
        } else if (param->read.handle == hid.boot_input_handle) {
            memcpy(rsp.attr_value.value, boot_input, sizeof(boot_input));
            rsp.attr_value.len = sizeof(boot_input);
        } else {
            rsp.attr_value.len = 0;
        }
        esp_ble_gatts_send_response(hid.gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;
    }

    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(TAG, "WRITE_EVT handle=%d len=%d", param->write.handle, param->write.len);

        // CCCD writes (enable/disable notifications)
        if (param->write.handle == hid.report1_cccd_handle || param->write.handle == hid.report2_cccd_handle || param->write.handle == hid.boot_input_cccd_handle) {
            uint16_t val = param->write.value[0] | (param->write.value[1] << 8);
            ESP_LOGI(TAG, "CCCD written 0x%04x", val);
            if (param->write.need_rsp) esp_ble_gatts_send_response(hid.gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            break;
        }

        // Boot Output (LEDs)
        if (param->write.handle == hid.boot_output_handle) {
            if (param->write.len >= 1) {
                uint8_t led = param->write.value[0];
                ESP_LOGI(TAG, "Boot Output (LEDs) written: 0x%02x (Num=%d Caps=%d Scroll=%d)",
                         led, (led & 0x01) ? 1 : 0, (led & 0x02) ? 1 : 0, (led & 0x04) ? 1 : 0);
            }
            if (param->write.need_rsp) esp_ble_gatts_send_response(hid.gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            break;
        }

        // Report3 writes
        if (param->write.handle == hid.report3_handle) {
            ESP_LOGI(TAG, "Report3 written, len=%d", param->write.len);
            ESP_LOG_BUFFER_HEX(TAG, param->write.value, param->write.len);
            if (param->write.need_rsp) esp_ble_gatts_send_response(hid.gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            break;
        }

        // Protocol Mode write (write without response maybe)
        if (param->write.handle == hid.protocol_mode_handle) {
            if (param->write.len >= 1) {
                protocol_mode = param->write.value[0];
                ESP_LOGI(TAG, "Protocol mode set to %d", protocol_mode);
            }
            if (param->write.need_rsp) esp_ble_gatts_send_response(hid.gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            break;
        }

        if (param->write.need_rsp) esp_ble_gatts_send_response(hid.gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        break;
    }

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "CONNECT_EVT conn_id=%d", param->connect.conn_id);
        hid.conn_id = param->connect.conn_id;
        // start random sender once
        static bool started = false;
        if (!started) {
            started = true;
            xTaskCreate(random_sender_task, "rand_send", 4096, NULL, 5, NULL);
        }
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "DISCONNECT_EVT");
        hid.conn_id = 0xFFFF;
        esp_ble_gap_start_advertising(&adv_params);
        break;

    default:
        break;
    }
}

/* top-level GATTS handler that delegates to profile handler */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "REG failed, status=%d", param->reg.status);
            return;
        }
    }
    gatts_profile_event_handler(event, gatts_if, param);
}

/* app_main: init NVS, BT controller, bluedroid and register callbacks */
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // register callbacks
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(PROFILE_APP_ID));

    srand((unsigned)esp_timer_get_time());

    // init hid struct defaults
    hid.conn_id = 0xFFFF;
    hid.gatts_if = 0;
    hid.added_chars = 0;
    hid.added_descr = 0;
}
