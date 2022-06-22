#ifndef _APP_MQTT_H
#define _APP_MQTT_H


void mqtt_SetConnected(bool bConnect);
bool mqtt_IsConnected(void);
void mqtt_SetConnectionCallback(void (*callback)(bool bConnect));

void mqtt_SetReconnectAction(bool bReconnectAction);
bool mqtt_IsReconnectAction(void);

#endif /* _APP_WIFI_H */
