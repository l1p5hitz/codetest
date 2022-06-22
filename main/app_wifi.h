#ifndef _APP_WIFI_H
#define _APP_WIFI_H

typedef enum {
	WIFI_STATE_DISABLED = 0,
	WIFI_STATE_INIT,
	WIFI_STATE_CONNECTING, //connected, but not yet got IP address
	WIFI_STATE_CONNECTED,
	WIFI_STATE_DISCONNECTED,
	NUM_OF_WIFI_STATE,
} wifi_state_t;


void wifi_SetConnectionState(wifi_state_t state);
wifi_state_t wifi_GetConnectionState(void);
void wifi_SetStateCallback(void (*callback)(wifi_state_t state, wifi_state_t state_prev));

esp_err_t wifi_StationJoin(char* ssid, char* password);

#endif /* _APP_WIFI_H */
