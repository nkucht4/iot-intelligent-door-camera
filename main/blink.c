#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/gpio.h" 
#include <sys/socket.h>
#include <netdb.h>
#include "esp_wifi.h"

#define WIFI_SSID      "SSID_IOT"
#define WIFI_PASS      "PASSWORD_IOT"
static const char *TAG = "wifi_station";
#define WEB_SERVER "example.com"
#define WEB_PORT "80"
#define WEB_PATH "/"

#define LED_GPIO 33

EventGroupHandle_t wifi_eventgroup;
const EventBits_t WF1_BIT = BIT0;

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void blink_task(void* arg);


void app_main(void)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    esp_err_t ret = nvs_flash_init();           
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_eventgroup = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = { //konfiguracja polaczenia
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    xTaskCreate(blink_task, "blink_task", 2048, NULL, 1, NULL);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

static void blink_task(void* arg)
{
    EventBits_t wifi;
    bool led_on = false;
    while (1){
        wifi = xEventGroupWaitBits(wifi_eventgroup, WF1_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

        while (xEventGroupGetBits(wifi_eventgroup) & WF1_BIT) {
            led_on = !led_on;
            gpio_set_level(LED_GPIO, led_on);
            vTaskDelay(300);
        }

        gpio_set_level(LED_GPIO, 0);
    }
}

static void htttp_request(){ 
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM }; 
    struct addrinfo *res;

    int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res); 

    if (err != 0) {
        ESP_LOGI(TAG, "getaddrinfo failed /n");
        return; 
    }

    int sock = socket(res->ai_family, res->ai_socktype, 0); 
    if (sock < 0) {
        ESP_LOGI(TAG, "Failed to create socket: errno %d", errno);
        freeaddrinfo(res);
        return;
    }

    if (connect(sock, res->ai_addr, res->ai_addrlen) != 0) {
        ESP_LOGI(TAG, "Socket connect failed: errno %d", errno);
        close(sock);
        freeaddrinfo(res);
        return;
    }

    char request[128]; // wysylanie zapytania http
    sprintf(request, "GET %s HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n", WEB_PATH, WEB_SERVER);
    ssize_t sent = send(sock, request, strlen(request), 0);

    if (sent < 0) {
        ESP_LOGI(TAG, "Failed to send request: errno %d", errno);
        close(sock);
        freeaddrinfo(res);
        return;
    }

    char recv_buf[512]; // odbior odpowiedzi i wypisanie 
    int len;
    while ((len = recv(sock, recv_buf, sizeof(recv_buf)-1, 0)) > 0) {
        recv_buf[len] = 0;
        printf("%s", recv_buf);
    }

    close(sock); // cleaning 
    freeaddrinfo(res);
    ESP_LOGI(TAG, "Done!");
}

static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        gpio_set_level(LED_GPIO, 0); 
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI("wifi", "Disconnected...");
        xEventGroupSetBits(wifi_eventgroup, WF1_BIT);
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("wifi", "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupClearBits(wifi_eventgroup, WF1_BIT);
        htttp_request();
    }
}