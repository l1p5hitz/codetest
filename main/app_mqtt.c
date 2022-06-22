#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"

#include "app_mqtt.h"

static bool mqtt_bConnect = false;
static void (*mqtt_connect_callback)(bool bConnect) = NULL;

void mqtt_SetConnected(bool bConnect)
{
	if (mqtt_bConnect != bConnect) {
		mqtt_bConnect = bConnect;
		if (mqtt_connect_callback) {
			mqtt_connect_callback(mqtt_bConnect);
		}
	}
}

bool mqtt_IsConnected(void)
{
	return mqtt_bConnect;
}

void mqtt_SetConnectionCallback(void (*callback)(bool bConnect))
{
	mqtt_connect_callback = callback;
}

static bool mqtt_bReconnectAction = false;
void mqtt_SetReconnectAction(bool bReconnectAction)
{
	mqtt_bReconnectAction = bReconnectAction;
}

bool mqtt_IsReconnectAction(void)
{
	return mqtt_bReconnectAction;
}

