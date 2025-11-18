#include "mqqt_client.h"
#include "nvs_flash.h"
#include "esp_log.h"

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_ERROR_CHECK(ret);

    wifi_init();

    mqtt_init();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}