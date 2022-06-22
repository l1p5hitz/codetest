#ifndef _APP_MAIN_H
#define _APP_MAIN_H


//MQTT function
#define MQTT_BROKER_URL_LEN		80
extern char MQTT_BROKER_URL[MQTT_BROKER_URL_LEN];
extern char MQTT_BROKER_URL_TestMode[MQTT_BROKER_URL_LEN];
void mqtt_app_start(void);
void mqtt_app_stop(void);


//DeviceConfig settings
enum {
	DEVICECONFIG_OUTPUT_IGNORE = 0,
	DEVICECONFIG_OUTPUT_OFF,
	DEVICECONFIG_OUTPUT_ON,
	DEVICECONFIG_OUTPUT_DISABLE,
	NUM_OF_DEVICECONFIG_OUTPUT
};
bool device_isValidDeviceConfig_031(uint8_t _DeviceConfig);
uint8_t device_calcFanRealState_031(uint8_t _DeviceConfig, uint8_t _Bttn1RealState, uint8_t _Bttn2RealState);
uint8_t device_calcMOverride_031(uint8_t _DeviceConfig, uint8_t _FanAlgoState, uint8_t _FanRealState);
bool device_isAlgoActive_031(void);
uint8_t device_getOutput1Config_031(void);
uint8_t device_getOutput2Config_031(void);
void device_updateResult_031(void);

#endif /* _APP_MAIN_H */