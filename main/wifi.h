#ifndef WIFI_H
#define WIFI_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

extern EventGroupHandle_t wifi_eventgroup;

#define WIFI_CONNECTED_BIT BIT1

void wifi_init(void);

#endif