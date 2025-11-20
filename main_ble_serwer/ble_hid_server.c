#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
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

/* UUIDs */
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

/* EventGroup bits */
static EventGroupHandle_t hid_evt_group;

#define EVT_GATTS_READY   (1 << 0)
#define EVT_CONNECTED     (1 << 1)
#define EVT_PAIRED_READY  (1 << 2) 

/* HID values */
static uint8_t hid_info_value[4] = { 0x11, 0x01, 0x00, 0x00 }; // bcdHID 1.11, country 0, flags
static uint8_t protocol_mode = 1; // report mode (1)

/* Report Map */
static uint8_t report_map[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    0x05, 0x07,        //   Usage Page (Key Codes)
    0x19, 0xE0,        //   Usage Minimum (224)
    0x29, 0xE7,        //   Usage Maximum (231)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x08,        //   Report Count (8)
    0x81, 0x02,        //   Input (Data,Var,Abs) ; Modifier byte
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x01,        //   Input (Cnst,Arr,Abs) ; Reserved byte
    0x95, 0x05,        //   Report Count (5)
    0x75, 0x01,        //   Report Size (1)
    0x05, 0x08,        //   Usage Page (LEDs)
    0x19, 0x01,        //   Usage Minimum (1)
    0x29, 0x05,        //   Usage Maximum (5)
    0x91, 0x02,        //   Output (Data,Var,Abs) ; LED report
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x03,        //   Report Size (3)
    0x91, 0x01,        //   Output (Cnst,Arr,Abs) ; LED report padding
    0x95, 0x06,        //   Report Count (6)
    0x75, 0x08,        //   Report Size (8)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x65,        //   Logical Maximum (101)
    0x05, 0x07,        //   Usage Page (Key Codes)
    0x19, 0x00,        //   Usage Minimum (0)
    0x29, 0x65,        //   Usage Maximum (101)
    0x81, 0x00,        //   Input (Data,Arr,Abs) ; Key arrays (6 keys)
    0xC0,              // End Collection

    // Boot Keyboard Report (for compatibility)
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x08,        //   Report Count (8)
    0x05, 0x07,        //   Usage Page (Key Codes)
    0x19, 0xE0,        //   Usage Minimum (224)
    0x29, 0xE7,        //   Usage Maximum (231)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x81, 0x02,        //   Input (Data,Var,Abs) ; Modifier keys
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x01,        //   Input (Cnst,Arr,Abs) ; Reserved
    0x95, 0x05,        //   Report Count (5)
    0x75, 0x01,        //   Report Size (1)
    0x05, 0x08,        //   Usage Page (LEDs)
    0x19, 0x01,        //   Usage Minimum (1)
    0x29, 0x05,        //   Usage Maximum (5)
    0x91, 0x02,        //   Output (Data,Var,Abs) ; LED report
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x03,        //   Report Size (3)
    0x91, 0x01,        //   Output (Cnst,Arr,Abs) ; Padding
    0x95, 0x06,        //   Report Count (6)
    0x75, 0x08,        //   Report Size (8)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x65,        //   Logical Maximum (101)
    0x05, 0x07,        //   Usage Page (Key Codes)
    0x19, 0x00,        //   Usage Minimum (0)
    0x29, 0x65,        //   Usage Maximum (101)
    0x81, 0x00,        //   Input (Data,Arr,Abs) ; Key array
    0xC0               // End Collection
};

/* Boot input (modifier + reserved + 6 keycodes) */
static uint8_t boot_input[8] = {0};

/* advertising data */
static uint8_t adv_service_uuid128[16] = {
    0x12,0x34,0x56,0x78,0x9a,0xbc,0xde,0xf0,
    0x12,0x34,0x56,0x78,0x12,0x12,0x12,0x12
};

//static uint16_t adv_service_uuid16 = 0x1812;


static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .appearance = 0x03C1,
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

/* handles storage */
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

/* helper */
static inline esp_bt_uuid_t mk_uuid16(uint16_t u) {
    esp_bt_uuid_t id;
    id.len = ESP_UUID_LEN_16;
    id.uuid.uuid16 = u;
    return id;
}

/* forward */
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* safe send: check event bits before send */
/* safe send: check event bits before send */
static void safe_send_indicate(uint16_t handle, const void *data, uint16_t len)
{
    EventBits_t bits = xEventGroupGetBits(hid_evt_group);
    // Only check for GATT ready and connected (remove PAIRED_READY)
    if ((bits & (EVT_GATTS_READY | EVT_CONNECTED)) != (EVT_GATTS_READY | EVT_CONNECTED)) {
        ESP_LOGW(TAG, "Cannot send: GATT not ready or not connected. Bits: 0x%x", bits);
        return;
    }
    
    if (hid.gatts_if == ESP_GATT_IF_NONE || hid.conn_id == 0xFFFF) {
        ESP_LOGW(TAG, "Cannot send: Invalid gatts_if=%d or conn_id=%d", hid.gatts_if, hid.conn_id);
        return;
    }

    ESP_LOGI(TAG, "Sending to handle %d, len=%d, gatts_if=%d, conn_id=%d", 
             handle, len, hid.gatts_if, hid.conn_id);
    
    esp_err_t err;
    if (handle == hid.boot_input_handle || handle == hid.report1_handle) {
        err = esp_ble_gatts_send_indicate(hid.gatts_if, hid.conn_id, handle, len, (uint8_t*)data, false);
    } else {
        err = esp_ble_gatts_send_indicate(hid.gatts_if, hid.conn_id, handle, len, (uint8_t*)data, false);
    }
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Send failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Send successful");
    }
}

/* prepare and send random reports */
static void send_random_key_reports(void)
{
    // Only check for GATT ready and connected
    EventBits_t bits = xEventGroupGetBits(hid_evt_group);
    if ((bits & (EVT_GATTS_READY | EVT_CONNECTED)) != (EVT_GATTS_READY | EVT_CONNECTED)) {
        return;
    }

    
    if (hid.gatts_if == ESP_GATT_IF_NONE || hid.conn_id == 0xFFFF) {
        ESP_LOGW(TAG, "Invalid connection state");
        return;
    }

    // Standard Boot Keyboard Input Report (8 bytes)
    // Byte 0: Modifiers
    // Byte 1: Reserved (must be 0)
    // Bytes 2-7: Key codes (up to 6 keys)
    uint8_t boot_report[8] = {0};
    
    // Generate random key (a-z)
    uint8_t keycode = 0x04 + (rand() % 26); // HID keycodes for a-z

    ESP_LOGI(TAG, "=== SENDING KEY: %c (0x%02x) ===", 'a' + (keycode - 0x04), keycode);

    boot_report[0] = 0x00;        // No modifiers
    boot_report[1] = 0x00;        // Reserved byte (IMPORTANT!)
    boot_report[2] = keycode;     // First key code    // First keycode
    
    ESP_LOGI(TAG, "Sending key press - Report: %02x %02x %02x %02x %02x %02x %02x %02x", 
             boot_report[0], boot_report[1], boot_report[2], boot_report[3],
             boot_report[4], boot_report[5], boot_report[6], boot_report[7]);
    safe_send_indicate(hid.boot_input_handle, boot_report, sizeof(boot_report));

    // Wait longer for key press to register (Android needs more time)
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Release keys - send all zeros
    memset(boot_report, 0, sizeof(boot_report));
    ESP_LOGI(TAG, "Sending key release");
    safe_send_indicate(hid.boot_input_handle, boot_report, sizeof(boot_report));

    ESP_LOGI(TAG, "=== KEY PRESS AND RELEASE COMPLETED ===");
}

/* random sender task: waits for both bits (GATT ready + connected) */
static void random_sender_task(void *arg)
{
    (void)arg;
    const EventBits_t bits_to_wait = (EVT_GATTS_READY | EVT_CONNECTED); // REMOVED EVT_PAIRED_READY

    while (1) {
        // Wait indefinitely until both bits set
        ESP_LOGI(TAG, "Waiting for connection...");
        xEventGroupWaitBits(hid_evt_group, bits_to_wait, pdFALSE, pdTRUE, portMAX_DELAY);

        ESP_LOGI(TAG, "🎉 Connected! Starting to send HID data...");
        
        // while connected and ready, send reports every 5s
        while ((xEventGroupGetBits(hid_evt_group) & bits_to_wait) == bits_to_wait) {
            send_random_key_reports();
            vTaskDelay(pdMS_TO_TICKS(1000)); // 
        }

        ESP_LOGI(TAG, "❌ Disconnected - stopping HID data");
    }
}

/* GAP handler */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        case ESP_GAP_BLE_SEC_REQ_EVT:
            ESP_LOGI(TAG, "ESP_GAP_BLE_SEC_REQ_EVT");
            // Always accept security request for Motorola
            esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
            break;
            
        case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
            // Motorola might request passkey notification
            ESP_LOGI(TAG, "ESP_GAP_BLE_PASSKEY_NOTIF_EVT passkey: %06d", param->ble_security.key_notif.passkey);
            break;
            
        case ESP_GAP_BLE_NC_REQ_EVT:
            ESP_LOGI(TAG, "ESP_GAP_BLE_NC_REQ_EVT");
            // For numeric comparison, always confirm
            esp_ble_confirm_reply(param->ble_security.key_notif.bd_addr, true);
            break;
            
        case ESP_GAP_BLE_KEY_EVT:
            ESP_LOGI(TAG, "ESP_GAP_BLE_KEY_EVT key type: %d", param->ble_security.ble_key.key_type);
            break;
            
        case ESP_GAP_BLE_AUTH_CMPL_EVT:
            if (param->ble_security.auth_cmpl.success) {
                ESP_LOGI(TAG, "ESP_GAP_BLE_AUTH_CMPL_EVT success");
                // Now that auth is complete, we can enable bonding
                esp_ble_bond_dev_t bonded_devs[10];
                int dev_num = sizeof(bonded_devs) / sizeof(bonded_devs[0]);
                int count = esp_ble_get_bond_device_list(&dev_num, bonded_devs);
                ESP_LOGI(TAG, "Bonded devices count: %d", count);
            } else {
                ESP_LOGE(TAG, "ESP_GAP_BLE_AUTH_CMPL_EVT fail: reason=%d", 
                        param->ble_security.auth_cmpl.fail_reason);
            }
            break;
            
        default:
            break;
    }
}

/* Main GATT profile handler */
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(TAG, "GATTS_REG_EVT");
        hid.gatts_if = gatts_if;
        hid.conn_id = 0xFFFF;
        hid.added_chars = 0;
        hid.added_descr = 0;

        // mark GATT ready
        xEventGroupSetBits(hid_evt_group, EVT_GATTS_READY);

        esp_ble_gap_set_device_name("ESP32 - KEYBOARD");
        esp_ble_gap_config_adv_data(&adv_data);

        esp_ble_gatts_create_service(gatts_if, &(esp_gatt_srvc_id_t){
            .is_primary = true,
            .id = {.inst_id = 0, .uuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = HID_SERVICE_UUID}}}
        }, GATTS_NUM_HANDLE);
        break;

    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(TAG, "GATTS_CREATE_EVT svc_handle=%d", param->create.service_handle);
        hid.service_handle = param->create.service_handle;

        // Add characteristics (HID info, protocol, report_map, reports, boot input/output)
        {
            esp_bt_uuid_t u;
            esp_attr_value_t a;

            // HID Information
            u = mk_uuid16(HID_INFO_CHAR_UUID);
            a = (esp_attr_value_t){ .attr_max_len = sizeof(hid_info_value), .attr_len = sizeof(hid_info_value), .attr_value = hid_info_value };
            esp_ble_gatts_add_char(hid.service_handle, &u, ESP_GATT_PERM_READ, ESP_GATT_CHAR_PROP_BIT_READ, &a, NULL);

            // Protocol Mode
            u = mk_uuid16(PROTOCOL_MODE_UUID);
            a = (esp_attr_value_t){ .attr_max_len = 1, .attr_len = 1, .attr_value = &protocol_mode };
            esp_ble_gatts_add_char(hid.service_handle, &u, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE_NR, &a, NULL);

            // Report Map
            u = mk_uuid16(REPORT_MAP_UUID);
            a = (esp_attr_value_t){ .attr_max_len = sizeof(report_map), .attr_len = sizeof(report_map), .attr_value = report_map };
            esp_ble_gatts_add_char(hid.service_handle, &u, ESP_GATT_PERM_READ, ESP_GATT_CHAR_PROP_BIT_READ, &a, NULL);

            // Report 1 (notify, read)
            u = mk_uuid16(REPORT_CHAR_UUID);
            a = (esp_attr_value_t){ .attr_max_len = 8, .attr_len = 0, .attr_value = NULL };
            esp_ble_gatts_add_char(hid.service_handle, &u, ESP_GATT_PERM_READ, ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY, &a, NULL);

            // Report 2 (notify, read)
            u = mk_uuid16(REPORT_CHAR_UUID);
            a = (esp_attr_value_t){ .attr_max_len = 4, .attr_len = 0, .attr_value = NULL };
            esp_ble_gatts_add_char(hid.service_handle, &u, ESP_GATT_PERM_READ, ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY, &a, NULL);

            // Report 3 (read/write)
            u = mk_uuid16(REPORT_CHAR_UUID);
            a = (esp_attr_value_t){ .attr_max_len = 16, .attr_len = 0, .attr_value = NULL };
            esp_ble_gatts_add_char(hid.service_handle, &u, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR, &a, NULL);

            // Boot Keyboard Input
            u = mk_uuid16(BOOT_KEYBOARD_INPUT_UUID);
            a = (esp_attr_value_t){ .attr_max_len = sizeof(boot_input), .attr_len = sizeof(boot_input), .attr_value = boot_input };
            esp_ble_gatts_add_char(hid.service_handle, &u, ESP_GATT_PERM_READ, 
                                ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY, &a, NULL);

            // Boot Keyboard Output
            u = mk_uuid16(BOOT_KEYBOARD_OUTPUT_UUID);
            uint8_t init_out[1] = {0};
            a = (esp_attr_value_t){ .attr_max_len = 1, .attr_len = 1, .attr_value = init_out };
            esp_ble_gatts_add_char(hid.service_handle, &u, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR, &a, NULL);
        }
          
        break;

    case ESP_GATTS_ADD_CHAR_EVT: {
        uint16_t uuid16 = param->add_char.char_uuid.uuid.uuid16;
        hid.added_chars++;
        ESP_LOGI(TAG, "ADD_CHAR_EVT handle=%d uuid=0x%04x added=%d", param->add_char.attr_handle, uuid16, hid.added_chars);

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
                // add CCCD + report ref (ID=1, type=input)
                esp_bt_uuid_t cccd = mk_uuid16(CLIENT_CHAR_CFG_UUID);
                esp_attr_value_t cccd_attr = { .attr_max_len = 2, .attr_len = 0, .attr_value = NULL };
                esp_ble_gatts_add_char_descr(hid.service_handle, &cccd, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, &cccd_attr, NULL);

                uint8_t ref1[2] = {0x01, 0x01};
                esp_bt_uuid_t rr = mk_uuid16(REPORT_REF_DESC_UUID);
                esp_attr_value_t rr_attr = { .attr_max_len = 2, .attr_len = 2, .attr_value = ref1 };
                esp_ble_gatts_add_char_descr(hid.service_handle, &rr, ESP_GATT_PERM_READ, &rr_attr, NULL);

            } else if (!hid.report2_handle) {
                hid.report2_handle = param->add_char.attr_handle;
                esp_bt_uuid_t cccd = mk_uuid16(CLIENT_CHAR_CFG_UUID);
                esp_attr_value_t cccd_attr = { .attr_max_len = 2, .attr_len = 0, .attr_value = NULL };
                esp_ble_gatts_add_char_descr(hid.service_handle, &cccd, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, &cccd_attr, NULL);

                uint8_t ref2[2] = {0x02, 0x01};
                esp_bt_uuid_t rr = mk_uuid16(REPORT_REF_DESC_UUID);
                esp_attr_value_t rr_attr = { .attr_max_len = 2, .attr_len = 2, .attr_value = ref2 };
                esp_ble_gatts_add_char_descr(hid.service_handle, &rr, ESP_GATT_PERM_READ, &rr_attr, NULL);

            } else {
                hid.report3_handle = param->add_char.attr_handle;
                uint8_t ref3[2] = {0x03, 0x02}; // ID=3 type=output (writable)
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
        } else {
            ESP_LOGW(TAG, "Unknown char UUID 0x%04x", uuid16);
        }
        break;
    }

    case ESP_GATTS_ADD_CHAR_DESCR_EVT: {
        hid.added_descr++;
        uint16_t descr_handle = param->add_char_descr.attr_handle;
        uint16_t uuid16 = param->add_char_descr.descr_uuid.uuid.uuid16;

        ESP_LOGI(TAG, "ADD_CHAR_DESCR_EVT handle=%d uuid=0x%04x added_descr=%d",
            descr_handle, uuid16, hid.added_descr);

        if (uuid16 == CLIENT_CHAR_CFG_UUID) {
            // This is a CCCD - figure out which one based on order
            if (!hid.report1_cccd_handle) {
                hid.report1_cccd_handle = param->add_char_descr.attr_handle;
            } else if (!hid.report2_cccd_handle) {
                hid.report2_cccd_handle = param->add_char_descr.attr_handle;
            } else if (!hid.boot_input_cccd_handle) {
                hid.boot_input_cccd_handle = param->add_char_descr.attr_handle;
            }
        } else if (uuid16 == REPORT_REF_DESC_UUID) {
            // This is a Report Reference descriptor
            if (!hid.report1_report_ref_handle) {
                hid.report1_report_ref_handle = param->add_char_descr.attr_handle;
            } else if (!hid.report2_report_ref_handle) {
                hid.report2_report_ref_handle = param->add_char_descr.attr_handle;
            } else if (!hid.report3_report_ref_handle) {
                hid.report3_report_ref_handle = param->add_char_descr.attr_handle;
            }
        }

        // Start service when all expected descriptors are added
        if (hid.added_descr >= 6) {  // Adjust based on your actual count
            ESP_LOGI(TAG, "All descriptors added - starting service");
            esp_ble_gatts_start_service(hid.service_handle);
        }
        break;
    }

    case ESP_GATTS_START_EVT:
        //ESP_LOGI(TAG, "SERVICE STARTED");
        break;

    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(TAG, "READ_EVT conn_id=%d, handle=%d, trans_id=%d, offset=%d", 
                param->read.conn_id, param->read.handle, param->read.trans_id, param->read.offset);
        
        // Only send response if needed
        if (param->read.need_rsp) {
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.offset = param->read.offset;
            rsp.attr_value.len = 0;

            if (param->read.handle == hid.hid_info_handle) {
                memcpy(rsp.attr_value.value, hid_info_value, sizeof(hid_info_value));
                rsp.attr_value.len = sizeof(hid_info_value);
            } else if (param->read.handle == hid.protocol_mode_handle) {
                rsp.attr_value.value[0] = protocol_mode;
                rsp.attr_value.len = 1;
            } else if (param->read.handle == hid.report_map_handle) {
                // Handle report map with offset support
                uint16_t total_len = sizeof(report_map);
                uint16_t offset = param->read.offset;
                uint16_t remaining = total_len - offset;
                uint16_t copy_len = (remaining > ESP_GATT_MAX_ATTR_LEN) ? ESP_GATT_MAX_ATTR_LEN : remaining;
                
                if (offset < total_len) {
                    memcpy(rsp.attr_value.value, report_map + offset, copy_len);
                    rsp.attr_value.len = copy_len;
                } else {
                    rsp.attr_value.len = 0;
                }
                ESP_LOGI(TAG, "Report map read: offset=%d, len=%d, total=%d", offset, copy_len, total_len);
            } else if (param->read.handle == hid.boot_input_handle) {
                memcpy(rsp.attr_value.value, boot_input, sizeof(boot_input));
                rsp.attr_value.len = sizeof(boot_input);
            } else if (param->read.handle == hid.report1_handle) {
                // Handle report1 read - return empty report
                rsp.attr_value.value[0] = 0x01; // Report ID
                rsp.attr_value.len = 8;
            } else if (param->read.handle == hid.report2_handle) {
                // Handle report2 read - return empty report  
                rsp.attr_value.value[0] = 0x02; // Report ID
                rsp.attr_value.len = 4;
            } else if (param->read.handle == hid.boot_output_handle) {
                rsp.attr_value.value[0] = 0x00; // LED status
                rsp.attr_value.len = 1;
            } else {
                ESP_LOGW(TAG, "Read from unknown handle: %d", param->read.handle);
                // Send empty response for unknown handles
                rsp.attr_value.len = 0;
            }

            esp_err_t ret = esp_ble_gatts_send_response(gatts_if, param->read.conn_id, 
                                                    param->read.trans_id, ESP_GATT_OK, &rsp);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send read response: %s", esp_err_to_name(ret));
            }
        }
        break;
    }

    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(TAG, "WRITE_EVT handle=%d len=%d need_rsp=%d", 
            param->write.handle, param->write.len, param->write.need_rsp);

        // CCCD writes (enable/disable notifications)
        if (param->write.handle == hid.report1_cccd_handle || 
            param->write.handle == hid.report2_cccd_handle || 
            param->write.handle == hid.boot_input_cccd_handle) {
            
            if (param->write.len >= 2) {
                uint16_t val = param->write.value[0] | (param->write.value[1] << 8);
                ESP_LOGI(TAG, "CCCD written 0x%04x to handle %d", val, param->write.handle);
                
                // Just log, don't set EVT_PAIRED_READY
                if (val == 0x0001) {
                    ESP_LOGI(TAG, "NOTIFICATIONS ENABLED by Android");
                } else if (val == 0x0000) {
                    ESP_LOGI(TAG, "NOTIFICATIONS DISABLED by Android");
                }
            }
            
            if (param->write.need_rsp) {
                esp_err_t ret = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, 
                                                        param->write.trans_id, ESP_GATT_OK, NULL);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to send CCCD write response: %s", esp_err_to_name(ret));
                }
            }
            break;
        }

        // Boot Output (LEDs)
        if (param->write.handle == hid.boot_output_handle) {
            if (param->write.len >= 1) {
                uint8_t led = param->write.value[0];
                ESP_LOGI(TAG, "Boot Output (LEDs) written: 0x%02x", led);
            }
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, 
                                        param->write.trans_id, ESP_GATT_OK, NULL);
            }
            break;
        }

        // Protocol Mode write
        if (param->write.handle == hid.protocol_mode_handle) {
            if (param->write.len >= 1) {
                protocol_mode = param->write.value[0];
                ESP_LOGI(TAG, "Protocol mode set to %d", protocol_mode);
            }
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, 
                                        param->write.trans_id, ESP_GATT_OK, NULL);
            }
            break;
        }

        // Report3 writes
        if (param->write.handle == hid.report3_handle) {
            ESP_LOGI(TAG, "Report3 written, len=%d", param->write.len);
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, 
                                        param->write.trans_id, ESP_GATT_OK, NULL);
            }
            break;
        }

        // Default response for any other writes
        if (param->write.need_rsp) {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, 
                                    param->write.trans_id, ESP_GATT_OK, NULL);
        }
        break;
    }

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "CONNECT_EVT conn_id=%d, gatts_if=%d", param->connect.conn_id, gatts_if);
        hid.conn_id = param->connect.conn_id;
        hid.gatts_if = gatts_if;  // Make sure this is set
        xEventGroupSetBits(hid_evt_group, EVT_CONNECTED);
        
        // Start security encryption
        esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "DISCONNECT_EVT, reason=0x%02x", param->disconnect.reason);
        hid.conn_id = 0xFFFF;
        // Clear both connected AND paired ready bits
        xEventGroupClearBits(hid_evt_group, EVT_CONNECTED | EVT_PAIRED_READY);
        
        // Restart advertising
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

    xEventGroupSetBits(hid_evt_group, EVT_GATTS_READY);
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

    hid_evt_group = xEventGroupCreate();
    configASSERT(hid_evt_group != NULL);


    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    //security
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM;  // Basic bonding without Secure Connections
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t oob_support = ESP_BLE_OOB_DISABLE;

    uint8_t init_key = ESP_BLE_ENC_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK;

    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(iocap));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(key_size));
    esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(oob_support));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(init_key));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(rsp_key));

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

    // create random sender task - it will wait on event bits
    xTaskCreate(random_sender_task, "rand_send", 4096, NULL, 5, NULL);
}
