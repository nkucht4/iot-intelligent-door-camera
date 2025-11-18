#include <stdint.h>    
#include <sys/types.h>  
#include <sys/select.h> 
#include "mqtt_client.h"
#include "wifi.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

static const char *TAG = "MQTT";

static const char *user_id   = "user123";
static const char *device_id = "device01";

/* TOPICS

    home/user<id>/device<id>/data/temperature
    home/user<id>/device<id>/data/battery

    home/user<id>/device<id>/doorbell

    home/user<id>/device<id>/cam/image
    home/user<id>/device<id>/cam/img_metadata

    home/user<id>/device<id>/cmd/capture
    home/user<id>/device<id>/cmd/reboot

    home/user<id>/device<id>/cmd/lcd/text
    home/user<id>/device<id>/cmd/lcd/clear

*/

static esp_mqtt_client_handle_t client;

static void make_topic(char *buf, size_t buf_len, const char *suffix)
{
    snprintf(buf, buf_len,
             "home/user%s/device%s/%s",
             user_id, device_id, suffix);
    ESP_LOGI(TAG, "Topic %s created.\n", buf);
}

#define TOPIC_LEN 128

static char topic_temperature[TOPIC_LEN];
static char topic_doorbell[TOPIC_LEN];
static char topic_battery[TOPIC_LEN];

static char topic_cam_image[TOPIC_LEN];
static char topic_cam_meta[TOPIC_LEN];

static char topic_lcd_cmd_text[TOPIC_LEN];
static char topic_lcd_cmd_clear[TOPIC_LEN];

static char topic_cmd_capture[TOPIC_LEN];
static char topic_cmd_reboot[TOPIC_LEN];

static void init_topics(void)
{
    make_topic(topic_temperature,     TOPIC_LEN, "data/temperature");
    make_topic(topic_battery,         TOPIC_LEN, "data/battery");

    make_topic(topic_doorbell,        TOPIC_LEN, "doorbell");

    make_topic(topic_cam_image,       TOPIC_LEN, "cam/image");
    make_topic(topic_cam_meta,        TOPIC_LEN, "cam/img_metadata");

    make_topic(topic_cmd_capture,     TOPIC_LEN, "cmd/capture");
    make_topic(topic_cmd_reboot,      TOPIC_LEN, "cmd/reboot");

    make_topic(topic_lcd_cmd_text,    TOPIC_LEN, "cmd/lcd/text");
    make_topic(topic_lcd_cmd_clear,   TOPIC_LEN, "cmd/lcd/clear");

    ESP_LOGI(TAG, "Finished initializing topics.");
}

static void subscribe_to_commands(void)
{
    int msg_id;

    msg_id = esp_mqtt_client_subscribe(client, topic_lcd_cmd_text, 1);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to subscribe to %s", topic_lcd_cmd_text);
    } else {
        ESP_LOGI(TAG, "Subscribed to %s, msg_id=%d", topic_lcd_cmd_text, msg_id);
    }

    msg_id = esp_mqtt_client_subscribe(client, topic_lcd_cmd_clear, 1);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to subscribe to %s", topic_lcd_cmd_clear);
    } else {
        ESP_LOGI(TAG, "Subscribed to %s, msg_id=%d", topic_lcd_cmd_clear, msg_id);
    }

    msg_id = esp_mqtt_client_subscribe(client, topic_cmd_capture, 1);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to subscribe to %s", topic_cmd_capture);
    } else {
        ESP_LOGI(TAG, "Subscribed to %s, msg_id=%d", topic_cmd_capture, msg_id);
    }

    msg_id = esp_mqtt_client_subscribe(client, topic_cmd_reboot, 1);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to subscribe to %s", topic_cmd_reboot);
    } else {
        ESP_LOGI(TAG, "Subscribed to %s, msg_id=%d", topic_cmd_reboot, msg_id);
    }

    ESP_LOGI(TAG, "Subscription attempts finished.");
}

static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQQT Connected.");
            subscribe_to_commands();
            break;

        case MQTT_EVENT_DATA:
            if (strncmp(event->topic, topic_cmd_capture, event->topic_len) == 0) {
                ESP_LOGI(TAG, "Command received: Capture");
            }
            else if (strncmp(event->topic, topic_cmd_reboot, event->topic_len) == 0) {
                ESP_LOGI(TAG, "Command received: Reboot");
                esp_restart();
            }
            else if (strncmp(event->topic, topic_lcd_cmd_text, event->topic_len) == 0) {
                ESP_LOGI(TAG, "Command received: Display text");
            }
            else if (strncmp(event->topic, topic_lcd_cmd_clear, event->topic_len) == 0) {
                ESP_LOGI(TAG, "Command received: Clear text on LCD");
            }
            break;

        default:
            break;
    }
}

void publish_temperature(float temp)
{
    char msg[32];
    snprintf(msg, sizeof(msg), "%.2f", temp);

    int msg_id = esp_mqtt_client_publish(client, topic_temperature, msg, 0, 1, false);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish temperature");
    } else {
        ESP_LOGI(TAG, "Temperature published, msg_id=%d", msg_id);
    }
    
    ESP_LOGI(TAG, "Send temperature %.2f", temp);
}

void publish_doorbell_event(void)
{
    esp_mqtt_client_publish(client, topic_doorbell, "pressed", 0, 1, false);
}

void publish_battery(int percent)
{
    char msg[16];
    snprintf(msg, sizeof(msg), "%d", percent);
    esp_mqtt_client_publish(client, topic_battery, msg, 0, 1, false);
}

void publish_image(/*camera_fb_t *fb*/)
{
    uint8_t placeholder_image[75] = {
        0,0,0,   255,255,255, 0,0,0,   255,255,255, 0,0,0,
        255,255,255, 0,0,0,   255,255,255, 0,0,0,   255,255,255,
        0,0,0,   255,255,255, 0,0,0,   255,255,255, 0,0,0,
        255,255,255, 0,0,0,   255,255,255, 0,0,0,   255,255,255,
        0,0,0,   255,255,255, 0,0,0,   255,255,255, 0,0,0
    };
    esp_mqtt_client_publish(client,
                        topic_cam_image,
                        (const char *)placeholder_image,
                        sizeof(placeholder_image),
                        0,
                        false);
                        
    publish_image_meta(5, 5, sizeof(tiny_image));
}

void publish_image_meta(int width, int height, int size_bytes)
{
    char json[128];
    snprintf(json, sizeof(json),
             "{\"width\":%d,\"height\":%d,\"size\":%d}",
             width, height, size_bytes);

    esp_mqtt_client_publish(client, topic_cam_meta, json, 0, 1, false);
}

static void temperature_task(void* arg)
{
    while (1){
        publish_temperature(10.2);
        vTaskDelay(pdMS_TO_TICKS(10000)); 
    }
}

void mqtt_init(void)
{
    xEventGroupWaitBits(wifi_eventgroup, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
    init_topics();

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = "mqtt://10.237.191.186",
        .credentials.username = "esp1",
        .credentials.authentication.password = "password",
    };

    client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    xTaskCreate(temperature_task, "temperature_task", 2048, NULL, 0, NULL);

}