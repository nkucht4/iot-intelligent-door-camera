// dodanie mapowania na klawisze przy wyjsciu, dodanie zeby sie ponownie polaczyl
// po tym jak bedzie disconected W (24427) KBD_CLIENT: Disconnected w event handler

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/util/util.h"

static const char *TAG = "KBD_CLIENT";

#define TARGET_NAME_SUBSTR "Keyboard"
#define HID_SERVICE_UUID 0x1812
#define HID_BOOT_UUID 0x2A22

static uint16_t g_conn = BLE_HS_CONN_HANDLE_NONE;
static uint16_t g_kbd_val_handle = 0;

/* HEX dump */
static void dump_hex(uint8_t *d, int len)
{
    char buf[128];
    int p = 0;
    for (int i = 0; i < len; i++)
        p += snprintf(&buf[p], sizeof(buf) - p, "%02X ", d[i]);
    ESP_LOGI(TAG, "[REPORT %d] %s", len, buf);
}

/* ----------- CHARACTERISTICS DISCOVERY ----------- */
static int chr_disc_cb(
    uint16_t conn_handle,
    const struct ble_gatt_error *error,
    const struct ble_gatt_chr *chr,
    void *arg)
{
    if (error->status != 0)
    {
        ESP_LOGI(TAG, "Characteristics discovery finished");

        if (g_kbd_val_handle)
        {
            uint16_t cccd = g_kbd_val_handle + 1;
            uint8_t cfg[2] = {1, 0};
            ble_gattc_write_flat(conn_handle, cccd, cfg, 2, NULL, NULL);
        }
        return 0;
    }

    uint16_t uuid16 = ble_uuid_u16(&chr->uuid.u);
    if (uuid16 == HID_BOOT_UUID)
    {
        g_kbd_val_handle = chr->val_handle;
        ESP_LOGI(TAG, "Found Boot Input Report: val=0x%04X", g_kbd_val_handle);
    }

    return 0;
}

/* ----------- SERVICE DISCOVERY ----------- */
static int svc_disc_cb(
    uint16_t conn_handle,
    const struct ble_gatt_error *error,
    const struct ble_gatt_svc *svc,
    void *arg)
{
    if (error->status != 0)
        return 0;

    uint16_t uuid16 = ble_uuid_u16(&svc->uuid.u);
    if (uuid16 == HID_SERVICE_UUID)
    {

        ESP_LOGI(TAG, "HID Service: start=0x%04X end=0x%04X",
                 svc->start_handle, svc->end_handle);

        ble_gattc_disc_all_chrs(
            conn_handle,
            svc->start_handle,
            svc->end_handle,
            chr_disc_cb,
            NULL);
    }

    return 0;
}

/* ----------- GAP EVENTS ----------- */
static int gap_cb(struct ble_gap_event *ev, void *arg)
{
    switch (ev->type)
    {

    case BLE_GAP_EVENT_DISC:
    {
        struct ble_hs_adv_fields f;
        if (ble_hs_adv_parse_fields(&f, ev->disc.data, ev->disc.length_data))
            return 0;

        if (f.name != NULL)
        {
            char name[32] = {0};
            memcpy(name, f.name, f.name_len);

            if (strstr(name, TARGET_NAME_SUBSTR))
            {

                ESP_LOGI(TAG, "FOUND %s, connecting...", name);
                ble_gap_disc_cancel();

                struct ble_gap_conn_params p = {
                    .scan_itvl = 0x10,
                    .scan_window = 0x10,
                    .itvl_min = 0x10,
                    .itvl_max = 0x10,
                    .latency = 0,
                    .supervision_timeout = 500};

                ble_gap_connect(
                    BLE_OWN_ADDR_PUBLIC,
                    &ev->disc.addr,
                    BLE_HS_FOREVER,
                    &p,
                    gap_cb,
                    NULL);
            }
        }
        break;
    }

    case BLE_GAP_EVENT_CONNECT:
        if (ev->connect.status == 0)
        {
            g_conn = ev->connect.conn_handle;
            ESP_LOGI(TAG, "Connected.");

            ble_uuid16_t svc = BLE_UUID16_INIT(HID_SERVICE_UUID);
            ble_gattc_disc_svc_by_uuid(g_conn, &svc.u, svc_disc_cb, NULL);
        }
        else
        {
            ESP_LOGE(TAG, "Connect failed");
        }
        break;

    case BLE_GAP_EVENT_NOTIFY_RX:
    {
        uint8_t buf[32];
        int len = OS_MBUF_PKTLEN(ev->notify_rx.om);
        if (len > 32)
            len = 32;

        os_mbuf_copydata(ev->notify_rx.om, 0, len, buf);
        dump_hex(buf, len);
        break;
    }

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "Disconnected");
        g_conn = BLE_HS_CONN_HANDLE_NONE;
        break;
    }
    return 0;
}

/* ----------- SCANNING ----------- */
static void start_scan(void)
{
    ESP_LOGI(TAG, "Scanning...");
    struct ble_gap_disc_params p = {0};
    p.itvl = 0x10;
    p.window = 0x10;
    p.passive = 0;

    ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &p, gap_cb, NULL);
}

/* ----------- SYNC ----------- */
static void on_sync(void)
{
    uint8_t addr[6];
    ble_hs_id_infer_auto(0, &addr[0]);
    ble_hs_id_set_rnd(addr);

    start_scan();
}

void host_task(void *param)
{
    nimble_port_run();
}

/* ----------- MAIN ----------- */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    nimble_port_init();
    ble_hs_cfg.sync_cb = on_sync;

    nimble_port_freertos_init(host_task);
}
