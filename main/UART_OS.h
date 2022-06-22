#ifndef __UART__
#define __UART__


#define MAX_SENSOR	9
extern float sensorVal[MAX_SENSOR];
//extern float sensorDiff[MAX_SENSOR];
extern uint8_t isNewData;
extern uint8_t isNewDataFalseCnt;
extern uint8_t FanRealState;//2019.12.11
extern uint8_t Bttn1RealState;//2019.12.11
extern uint8_t Bttn2RealState;//2019.12.11
extern uint8_t MOverride;//2019.12.11

extern unsigned char requestSensorData;
extern uint32_t switchAppCmd;
extern uint32_t switchCommand;
extern uint32_t switchCommand_Repeat;
#define SW_CMD_REPEAT_CNT	(2)
extern unsigned char firmwareVersion;

//011
extern uint8_t IAQindex;
extern uint8_t IAQcolor;
extern uint8_t LEDanimation;
extern uint8_t Bttn1AppState;
unsigned int FRCSetvalue;
//021
extern uint8_t FanAlgoState;
extern uint8_t Bttn2AppState;
extern uint8_t FanAppState;
extern uint8_t SwitchConfig;

extern uint8_t IAQLEDIntensity;

extern uint8_t STM_bootloader_test;

extern uint8_t Call_BT_enable;
extern uint8_t Call_BT_disable;
uint32_t checkUART_comm_cnt;


//in bit-wise
enum {
	SW_CMD_IAQ_INDEX		= 0, //0x01
	SW_CMD_IAQ_COLOR		= 1, //0x02
	SW_CMD_LED_ANIMATION	= 2, //0x04
	SW_CMD_FAN_ALGO_STATE	= 3, //0x08
	SW_CMD_BTTN1_APP_STATE	= 4, //0x10
	SW_CMD_BTTN2_APP_STATE	= 5, //0x20
	SW_CMD_FAN_APP_STATE	= 6, //0x40
	SW_CMD_SW_COFIG			= 7, //0x80
	SW_CMD_IAQLED_INTENSITY	= 8, //0x0100	
	SW_CMD_IAQ_BRIGHT_PWM	= 9, //0x0200 reserved for internal use
	SW_CMD_YELLOW_CONT_RED	= 10,//0x0400 reserved for internal use
	SW_CMD_YELLOW_CONT_GREEN= 11,//0x0800 reserved for internal use //patrick - for IAQ led yellow c
	
};

//in bit-wise
enum {
	SW_APPCMD_CO2_FRC_SET	= 0,
	SW_APPCMD_REQ_STM_VER	= 1,
	SW_APPCMD_REQ_STM_MODEL	= 2,
	SW_APPCMD_BT_STATUS		= 3,
	SW_APPCMD_REQ_STM_SIGN	= 4,		//patrick - STM security boot
	SW_APPCMD_HW_VER		= 5,
	SW_APPCMD_STM_BOOTLOADER_ONOFF = 6, //for internal use of RDP testing
};
#define SW_APPCMD_MASK	((1UL << SW_APPCMD_CO2_FRC_SET) \
						| (1UL << SW_APPCMD_REQ_STM_VER) \
						| (1UL << SW_APPCMD_REQ_STM_MODEL) \
						| (1UL << SW_APPCMD_BT_STATUS) \
						| (1UL << SW_APPCMD_REQ_STM_SIGN)\
						| (1UL << SW_APPCMD_HW_VER)\
						| (1UL << SW_APPCMD_STM_BOOTLOADER_ONOFF))

typedef enum {
	SW_LEDCMD_BT_PAIRING = 6,
	//SW_LEDCMD_BT_PAIR_OK = 7,
	SW_LEDCMD_WIFI_CONNECT_OK = 8,
	//SW_LEDCMD_WIFI_CONNECTED_NOTACTIVE = 9,
	//SW_LEDCMD_WIFI_CONNECTED_ACTIVE = 10,
	SW_LEDCMD_WIFI_DISCONNECTED = 11,
	//SW_LEDCMD_NO_WIFI_BT = 13,
} uart_SW_ledCmd_t;


//Cmd Type of STM to ESP (Special Type)
enum {
	SW_STM2ESP_VER_REQ = 1,
	SW_STM2ESP_VER_RESPONSE = 2,
	SW_STM2ESP_MODEL_REQ = 3,
	SW_STM2ESP_MODEL_RESPONSE = 4,
	SW_STM2ESP_MOVERRIDE = 5,
	SW_STM2ESP_REQ_ESP_WIFI_STATUS = 6,
	SW_STM2ESP_STM_FW_SIGN = 7,
	SW_STM2ESP_STM_SYSTEM_START = 8,
};

void UART_Init(void);

void uart_setSwitchDummyCmd(void);

void uart_setSwitchLedCmd(uart_SW_ledCmd_t cmd);
uart_SW_ledCmd_t uart_getSwitchLedCmd(void);

void uart_setSenorDataFlag(bool bEnable);
bool uart_getSenorDataFlag(void);
void uart_flushData(void);

void uart_statPrint(void);

void uart_skipToSendSensorDataCmd(uint8_t count);
enum {
	UART_SEND_TYPE_GENERAL = 0,
	UART_SEND_TYPE_SENSOR_DATA = 1,
	UART_SEND_TYPE_HEALTH_DATA = 2,
	NUM_OF_UART_SEND_TYPE,
};
uint8_t uart_sendCmd(uint8_t send_type);
bool uart_isSendCmdReady(void);
void uart_sensorCmdTimeoutCheck(void);
void uart_TxRxTask(void *arg);

void Usart2_Write(uint8_t *data, uint16_t size);
int Usart2_Read(uint8_t *data, uint16_t size);

#endif

