#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"

#include "app_wifi.h"

static wifi_state_t wifi_state = WIFI_STATE_DISABLED;
static void (*wifi_state_callback)(wifi_state_t state, wifi_state_t state_prev) = NULL;

void wifi_SetConnectionState(wifi_state_t state)
{
	if (wifi_state != state) {
		wifi_state_t state_prev = wifi_state;
		wifi_state = state;
		if (wifi_state_callback) {
			wifi_state_callback(wifi_state, state_prev);
		}
	}
}

wifi_state_t wifi_GetConnectionState(void)
{
	return wifi_state;
}

void wifi_SetStateCallback(void (*callback)(wifi_state_t state, wifi_state_t state_prev))
{
	wifi_state_callback = callback;
}

static wifi_config_t wifi_config = { 0 };
esp_err_t wifi_StationJoin(char* ssid, char* password)
{
	esp_err_t err;

	if (!ssid || !password) {
		return ESP_FAIL;
	}

	memset(&wifi_config, 0 , sizeof(wifi_config));
	strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid)-1);
	strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password)-1); 
	err = esp_wifi_set_mode(WIFI_MODE_STA);
	if (err != ESP_OK) {
		return err;
	}
	err = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
	if (err != ESP_OK) {
		return err;
	}
	err = esp_wifi_disconnect();
	if (err != ESP_OK) {
		return err;
	}
	err = esp_wifi_connect();
	return err;
}

