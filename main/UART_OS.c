#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "time.h"
#include "sys/time.h"

#include "CommonUse.h"
#include "Usart_BootMode.h"
#include "UART_OS.h"
#include "BLE_gatts_demo.h"
#include "app_wifi.h"


#define SENSOR_DEBUG_PRINT
#ifdef SENSOR_DEBUG_PRINT
	#define sen_dbg_printf(format, ... )	printf(format, ##__VA_ARGS__)
#else
	#define sen_dbg_printf(...)				do{}while(0)
#endif


extern char strftime_buf[64];

float sensorVal[MAX_SENSOR] = {0};
uint8_t isNewData = 0;
uint8_t isNewDataFalseCnt = 0;

uint8_t FanRealState = 0;//2019.12.11
uint8_t Bttn1RealState = 0;//2019.12.11
uint8_t Bttn2RealState = 0;//2019.12.11
uint8_t MOverride = 0;//2019.12.11
uint8_t Call_BT_enable;
uint8_t Call_BT_disable;

unsigned char requestSensorData = 0;
uint32_t switchAppCmd = 0; //it is in bitwise
uint32_t switchCommand = 0; //it is in bitwise
uint32_t switchCommand_Repeat = 0; //2019.12.17
unsigned char getUartData = 1;
unsigned char getUartData_failed = 0;




#define UART_TAG "UART_DRV"

#define BUFF_SIZE			512

#define EX_UART_NUM 		UART_NUM_1
#define UART_PORT_TX		17
#define UART_PORT_RX		16

// default pin for UART
//UART1 TX - GPIO 10
//UART1 RX - GPIO 9
//UART2 TX - GPIO 17
//UART2 RX - GPIO 16


enum {
	RX_CMD_TYPE_SENSOR_DATA = 1,
	RX_CMD_TYPE_KEY_CONTROL = 2,
	RX_CMD_TYPE_POWER_CONTROL = 3,
	RX_CMD_TYPE_SPECIAL = 4,
};

#define UART_START_BYTE		(0x7C)
#define UART_HEADER_STARTBYTE_POS	(0)
#define UART_HEADER_LENGTH_POS		(1)
#define UART_DATA_START_POS			(2)

#pragma pack(1)
typedef struct {
	uint8_t cmd_type;
	uint8_t cmd_id;
} cmd_header_t;

//cmd_type = 1 (RX_CMD_TYPE_SENSOR_DATA), Sensor Data Response Type
typedef struct {
	cmd_header_t hdr;
	union {
		//cmd_id = 1, CO2 Sensor (SCD30)
		struct {
			float CO2;
			float Temp;
			float Humidity;
		} CO2_sensor;
		//cmd_id = 2, PM2.5 Sensor (SPS30)
		struct {
			float PM_2_5;
		} PM_2_5_sensor;
		//cmd_id = 3, Temperature Sensor (Si7021)
		struct {
			float Temp;
			float Humidity;
		} Temp_sensor;
		//cmd_id = 4, TVOC Sensor (ZMOD4410)
		struct {
			float TVOC;
			float R_MOX_raw_data;
			float IAQ;
			float eCO2;
		} TVOC_sensor;
		//cmd_id = 5, Temperature Sensor Raw Data (Si7021)
		struct {
			float RawTemp;
			float RawHumidity;
			float RawNTC1;
			float RawNTC2;
			float RawNTC3;
		} Temp_sensor_RAW;
	} data;
} cmd_sensorData_t;
#pragma pack()/* reverting back to non-packing of structures*/

typedef struct {
	uint32_t rx_dummy_data;
	uint32_t rx_no_start_byte;
	uint32_t rx_packet_incomplete;
	uint32_t rx_checksum_incorrect;
} uart_stat_t;
static uart_stat_t uart_stat = {0};


#define UART_TX_BUFFER_SIZE		(128)
#define UART_RX_BUFFER_SIZE		(BUFF_SIZE)

typedef struct {
	//Tx
	xSemaphoreHandle TxSemaphore;
	uint8_t TxBuffer[UART_TX_BUFFER_SIZE];
	uint8_t TxBuffer_len;
	//Rx
	uint8_t RxBuffer[UART_RX_BUFFER_SIZE];
	uint8_t RxBuffer_len;
	uint8_t RxPartialData[64];
	uint8_t RxPartialData_len;
} uart_ctx_t;
uart_ctx_t uart_ctx = {0};




static QueueHandle_t uart0_queue;
void UART_Init(void)
{
	/* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
		// .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN, //UART_PARITY_EVEN,UART_PARITY_DISABLE
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

	uart_param_config(EX_UART_NUM, &uart_config);
    // Set UART pins using UART0 default pins i.e. no changes
    //uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_pin(EX_UART_NUM, UART_PORT_TX, UART_PORT_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(EX_UART_NUM, BUFF_SIZE * 2, BUFF_SIZE * 2, 10, &uart0_queue, 0);

    // Set uart pattern detection function
    //uart_enable_pattern_det_intr(EX_UART_NUM, '+', 3, 10000, 10, 10);

    {
        //workaround: STM32 will miss 1st command, send dummy command first
        uint8_t dummy_data[8];
        memset(dummy_data, 0xff, sizeof(dummy_data));
        Usart2_Write(dummy_data, sizeof(dummy_data));

        vTaskDelay(100/portTICK_RATE_MS);
        uart_flushData();
    }
}

#define CRC8_INIT		0xff
#define CRC8_POLYNOMIAL	0x31

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t generate_crc_8bit(uint8_t *data, uint16_t count) {
    uint16_t current_byte;
    uint8_t crc = CRC8_INIT;
    uint8_t crc_bit;

    // calculates 8-Bit checksum with given polynomial
    for (current_byte = 0; current_byte < count; ++current_byte) {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit) {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}


static xSemaphoreHandle switchLedCmd_semaphore = NULL;
void switchLedCmd_mutex_lock(void)
{
	if (!switchLedCmd_semaphore) {
		switchLedCmd_semaphore = xSemaphoreCreateMutex();
	}
	if (switchLedCmd_semaphore) {
		xSemaphoreTake(switchLedCmd_semaphore, portMAX_DELAY);
	}
}

void switchLedCmd_mutex_unlock(void)
{
	if (switchLedCmd_semaphore) {
		xSemaphoreGive(switchLedCmd_semaphore);
	}
}


static uint8_t switchDummyCmd = 0;
void uart_setSwitchDummyCmd(void)
{
	switchDummyCmd = 1;
}

static uart_SW_ledCmd_t switchLedCmd = 0;
static uart_SW_ledCmd_t switchLedCmd_runnning = 0;
void uart_setSwitchLedCmd(uart_SW_ledCmd_t cmd)
{
	switchLedCmd_mutex_lock();
	switchLedCmd = cmd;
	switchLedCmd_runnning = cmd;
	switchLedCmd_mutex_unlock();
}

uart_SW_ledCmd_t uart_getSwitchLedCmd(void)
{
	return switchLedCmd_runnning;
}

void uart_setSenorDataFlag(bool bEnable)
{
	requestSensorData = (bEnable)?5:0;
}

bool uart_getSenorDataFlag(void)
{
	if (requestSensorData) {
		return true;
	} else {
		return false;
	}
}

void uart_flushData(void)
{
	uart_flush_input(EX_UART_NUM);
}


void uart_statPrint(void)
{
	printf("rx_dummy_data: %d\n", uart_stat.rx_dummy_data);
	printf("rx_no_start_byte: %d\n", uart_stat.rx_no_start_byte);
	printf("rx_packet_incomplete: %d\n", uart_stat.rx_packet_incomplete);
	printf("rx_checksum_incorrect: %d\n", uart_stat.rx_checksum_incorrect);
}

static void TxBuffer_MutexLock(void)
{
	if (!uart_ctx.TxSemaphore) {
		uart_ctx.TxSemaphore = xSemaphoreCreateMutex();
	}
	if (uart_ctx.TxSemaphore) {
		xSemaphoreTake(uart_ctx.TxSemaphore, portMAX_DELAY);
	}
}

static void TxBuffer_MutexUnlock(void)
{
	if (uart_ctx.TxSemaphore) {
		xSemaphoreGive(uart_ctx.TxSemaphore);
	}
}

static int writeTxBuffer(uint8_t *data, uint8_t data_len)
{
	if (data == NULL || data_len == 0) {
		return -1;
	}
	if (data_len > UART_TX_BUFFER_SIZE) {
		return -1;
	}
	memcpy(uart_ctx.TxBuffer, data, data_len);
	uart_ctx.TxBuffer_len = data_len;
	return data_len;
}

static uint8_t generatePacket(uint8_t *buffer, uint8_t *data, uint8_t data_len)
{
	uint8_t packet_len = 0;
	if (buffer == NULL) {
		return 0;
	}
	if ((data_len > 0) && (data == NULL)) {
		return 0;
	}
	packet_len = 3 + data_len;
	buffer[0] = UART_START_BYTE;
	buffer[1] = packet_len - 1;
	if (data_len) {
		memcpy(&buffer[2], data, data_len);
	}
	buffer[packet_len - 1] = generate_crc_8bit(buffer, packet_len - 1);
	return packet_len;
}

static uint8_t generateSimplePacket(uint8_t *buffer, uint8_t cmd_type, uint8_t cmd_id, uint8_t cmd_single_data)
{
	uint8_t packet_len = 0;
	if (buffer == NULL) {
		return 0;
	}
	packet_len = 3 + 2 + 1;
	buffer[0] = UART_START_BYTE;
	buffer[1] = packet_len - 1;
	buffer[2] = cmd_type;
	buffer[3] = cmd_id;
	buffer[4] = cmd_single_data;
	buffer[packet_len - 1] = generate_crc_8bit(buffer, packet_len - 1);
	return packet_len;
}

static uint8_t uSkipSensorDataCmdCount = 0;
void uart_skipToSendSensorDataCmd(uint8_t count)
{
	uSkipSensorDataCmdCount = count;
}

uint8_t uart_sendCmd(uint8_t send_type)
{
	#define SENSOR_DATA_LEN		64
	uint8_t data[SENSOR_DATA_LEN] = {0};
	uint8_t data_len = 0;
	uint8_t ComLength;
	struct {
		cmd_header_t hdr;
		uint8_t cmd_data[8];
	} simpleCmd;

	if(send_type > NUM_OF_UART_SEND_TYPE)
	{
		return 0;
	}
	//check if Tx buffer is not empty
	if (uart_ctx.TxBuffer_len > 0)
	{
		return 0;
	}

	//check if uSkipSensorDataCmdCount > 0
	if (send_type == UART_SEND_TYPE_SENSOR_DATA)
	{
		if(uSkipSensorDataCmdCount)
		{
			printf("uSkipSensorDataCmdCount = %u\n", uSkipSensorDataCmdCount);
			uSkipSensorDataCmdCount--;
			return 0;
		}
	}

	TxBuffer_MutexLock();
	if (send_type == UART_SEND_TYPE_GENERAL)
	{
		if(switchDummyCmd)
		{
			data_len = generateSimplePacket(data, 255, 1, 0);
			switchDummyCmd = 0;
		}
		else if(switchLedCmd)
		{
			switchLedCmd_mutex_lock();
			if (switchLedCmd == SW_LEDCMD_BT_PAIRING) {
				data_len = generateSimplePacket(data, 2, 6, 1);
			} else if (switchLedCmd == SW_LEDCMD_WIFI_CONNECT_OK) {
				data_len = generateSimplePacket(data, 2, 8, 1);
			} else if (switchLedCmd == SW_LEDCMD_WIFI_DISCONNECTED) {
				data_len = generateSimplePacket(data, 2, 11, 1);
			} else {
			}
			switchLedCmd = 0;
			switchLedCmd_mutex_unlock();
		}
		else if(switchAppCmd & SW_APPCMD_MASK)
		{
			if(switchAppCmd & (1UL << SW_APPCMD_CO2_FRC_SET))
			{
				simpleCmd.hdr.cmd_type = 4;
				simpleCmd.hdr.cmd_id = 6;
				simpleCmd.cmd_data[0] = (FRCSetvalue>>8)&0xFF;
				simpleCmd.cmd_data[1] = FRCSetvalue&0xFF;
				data_len = generatePacket(data, (uint8_t *)&simpleCmd, sizeof(simpleCmd.hdr) + 2);
				printf("FRCSetvalue = %u, data[%X][%X]\r\n", FRCSetvalue, simpleCmd.cmd_data[0], simpleCmd.cmd_data[1]);
				switchAppCmd &= ~(1UL << SW_APPCMD_CO2_FRC_SET);
			}
			else if(switchAppCmd & (1UL << SW_APPCMD_REQ_STM_VER))
			{
				data_len = generateSimplePacket(data, 4, 1, 0);
				switchAppCmd &= ~(1UL << SW_APPCMD_REQ_STM_VER);
			}
			else if(switchAppCmd & (1UL << SW_APPCMD_REQ_STM_MODEL))
			{
				data_len = generateSimplePacket(data, 4, 3, 0);
				switchAppCmd &= ~(1UL << SW_APPCMD_REQ_STM_MODEL);
			}
			else if(switchAppCmd & (1UL << SW_APPCMD_BT_STATUS))
			{
				data_len = generateSimplePacket(data, 4, 7, BLE_IsEnabled());
				switchAppCmd &= ~(1UL << SW_APPCMD_BT_STATUS);
			}
			else if(switchAppCmd & (1UL << SW_APPCMD_REQ_STM_SIGN))
			{
				data_len = generateSimplePacket(data, 4, 12, 0);
				switchAppCmd &= ~(1UL << SW_APPCMD_REQ_STM_SIGN);
			}
			else if(switchAppCmd & (1UL << SW_APPCMD_HW_VER))
			{
				data_len = generateSimplePacket(data, 4, 13, appCommon_GetHwVersion());
				switchAppCmd &= ~(1UL << SW_APPCMD_HW_VER);
			}
			else if(switchAppCmd & (1UL << SW_APPCMD_STM_BOOTLOADER_ONOFF))
			{
				data_len = generateSimplePacket(data, 4, 15, STM_bootloader_test);
				switchAppCmd &= ~(1UL << SW_APPCMD_STM_BOOTLOADER_ONOFF);
			}
		}
		else if(switchCommand)
		{
			/*
				{"DeviceId":"RS19XXXXXXX",
				"IAQindex":"32",
				"IAQcolor":"1", 
				"LEDanimation":"0",
				"Bttn1AppState":"0",
				"TimeStamp":"119|9|29|00|00|00"}
			*/
			if (firmwareVersion == 0x11) //Room Sesor
			{
				data[0] = 0x7C;
				ComLength = 0;
				if(switchCommand & (1UL << SW_CMD_IAQ_INDEX))//check 1st command
				{
					data[2 + ComLength * 3] = 0x02; data[3 + ComLength * 3] = 0x04; data[4 + ComLength * 3] = IAQindex;
					ComLength++;
				}
				if(switchCommand & (1UL << SW_CMD_IAQ_COLOR))//check 2nd command
				{
					data[2 + ComLength * 3] = 0x02; data[3 + ComLength * 3] = 0x03; data[4 + ComLength * 3] = IAQcolor;
					ComLength++;
				}
				if(switchCommand & (1UL << SW_CMD_LED_ANIMATION))//check 3rd command
				{
					data[2 + ComLength * 3] = 0x02; data[3 + ComLength * 3] = 0x05; data[4 + ComLength * 3] = LEDanimation;
					ComLength++;
				}
				if(switchCommand & (1UL << SW_CMD_BTTN1_APP_STATE))//check 4th command
				{
					//Bttn1RealState = Bttn1AppState;//2019.12.17
					data[2 + ComLength * 3] = 0x05; data[3 + ComLength * 3] = 0x01; data[4 + ComLength * 3] = Bttn1AppState;
					ComLength++;
				}
				if(switchCommand & (1UL << SW_CMD_FAN_APP_STATE))//check 5th command
				{
					data[2 + ComLength * 3] = 0x05; data[3 + ComLength * 3] = 0x03; data[4 + ComLength * 3] = FanAppState;
					ComLength++;
				}
				if(switchCommand & (1UL << SW_CMD_SW_COFIG))//check 6th command
				{
					data[2 + ComLength * 3] = 0x04; data[3 + ComLength * 3] = 0x05; data[4 + ComLength * 3] = SwitchConfig;
					ComLength++;
				}
				if(switchCommand & (1UL << SW_CMD_IAQLED_INTENSITY))//check 7th command
				{
					data[2 + ComLength * 3] = 0x02; data[3 + ComLength * 3] = 0x0E; data[4 + ComLength * 3] = IAQLEDIntensity;
					ComLength++;
				}
				data_len = (ComLength * 3) + 2 + 1;
				data[1] = data_len - 1;
				data[data_len - 1] = generate_crc_8bit(data, data_len - 1);
			}
			/*
				{"DeviceId":"WC19XXXXXXX",
				"IAQindex":"32",
				"IAQcolor":"1",
				"LEDanimation":"0",
				"FanAlgoState":"1", 
				"Bttn1AppState":"0", 
				"Bttn2AppState":"0", 
				"SwitchConfig":"0", 
				"TimeStamp":"119|9|29|00|00|00""}
			*/
			else if (firmwareVersion == 0x21) //Wall Control
			{
				data[0] = 0x7C;
				ComLength = 0;
				if(switchCommand & (1UL << SW_CMD_IAQ_INDEX))//check 1st command
				{
					data[2 + ComLength * 3] = 0x02; data[3 + ComLength * 3] = 0x04; data[4 + ComLength * 3] = IAQindex;
					ComLength++;
				}
				if(switchCommand & (1UL << SW_CMD_IAQ_COLOR))//check 2nd command
				{
					data[2 + ComLength * 3] = 0x02; data[3 + ComLength * 3] = 0x03; data[4 + ComLength * 3] = IAQcolor;
					ComLength++;
				}
				if(switchCommand & (1UL << SW_CMD_LED_ANIMATION))//check 3rd command
				{
					data[2 + ComLength * 3] = 0x02; data[3 + ComLength * 3] = 0x05; data[4 + ComLength * 3] = LEDanimation;
					ComLength++;
				}
				if(switchCommand & (1UL << SW_CMD_BTTN1_APP_STATE))//check 4th command
				{
					data[2 + ComLength * 3] = 0x05; data[3 + ComLength * 3] = 0x01; data[4 + ComLength * 3] = Bttn1AppState;
					ComLength++;
				}
				if(switchCommand & (1UL << SW_CMD_BTTN2_APP_STATE))//check 5th command
				{
					data[2 + ComLength * 3] = 0x05; data[3 + ComLength * 3] = 0x02; data[4 + ComLength * 3] = Bttn2AppState;
					ComLength++;
				}
				if(switchCommand & (1UL << SW_CMD_FAN_APP_STATE))//check 6th command
				{
					data[2 + ComLength * 3] = 0x05; data[3 + ComLength * 3] = 0x03; data[4 + ComLength * 3] = FanAppState;
					ComLength++;
				}
				if(switchCommand & (1UL << SW_CMD_FAN_ALGO_STATE))//check 7th command
				{
					data[2 + ComLength * 3] = 0x03; data[3 + ComLength * 3] = 0x01; data[4 + ComLength * 3] = FanAlgoState;
					ComLength++;
				}
				if(switchCommand & (1UL << SW_CMD_SW_COFIG))//check 8th command
				{
					data[2 + ComLength * 3] = 0x04; data[3 + ComLength * 3] = 0x05; data[4 + ComLength * 3] = SwitchConfig;
					ComLength++;
				}
				if(switchCommand & (1UL << SW_CMD_IAQLED_INTENSITY))//check 9th command
				{
					data[2 + ComLength * 3] = 0x02; data[3 + ComLength * 3] = 0x0E; data[4 + ComLength * 3] = IAQLEDIntensity;
					ComLength++;
				}
				data_len = (ComLength * 3) + 2 + 1; //max data number : (9*3)+2+1=30
				data[1] = data_len - 1;
				data[data_len - 1] = generate_crc_8bit(data, data_len - 1);
			}

			if(switchCommand_Repeat)
			{
				switchCommand_Repeat--;
			}
			if (switchCommand_Repeat == 0) {
				//clear switchCommand
				switchCommand = 0;
			}
		}
	}
	else if (send_type == UART_SEND_TYPE_SENSOR_DATA)
	{
		switch(requestSensorData)
		{
			case 1://Sensor Data Request Type -> CO2 Sensor (SCD30)
			{
				data_len = generateSimplePacket(data, 1, 1, 0);
				break;
			}
			case 2://Sensor Data Request Type -> PM2.5 Sensor (SPS30)
			{
				data_len = generateSimplePacket(data, 1, 2, 0);
				break;
			}
			case 3://Sensor Data Request Type -> Tempperature Sensor (Si7021)
			{
				data_len = generateSimplePacket(data, 1, 3, 0);
				break;
			}
			case 4://Sensor Data Request Type -> TVOC Sensor (ZMOD4410)
			{
				data_len = generateSimplePacket(data, 1, 4, 0);
				break;
			}
			case 5://Sensor Data Request Type -> TVOC Sensor (ZMOD4410)
			{
				uint8_t tmpData[12] = {0};
				tmpData[0] = 0x01; tmpData[1] = 0x01; tmpData[2] = 0x00;
				tmpData[3] = 0x01; tmpData[4] = 0x02; tmpData[5] = 0x00;
				tmpData[6] = 0x01; tmpData[7] = 0x03; tmpData[8] = 0x00;
				tmpData[9] = 0x01; tmpData[10] = 0x04; tmpData[11] = 0x00;
				data_len = generatePacket(data, tmpData, 12);
				break;
			}
			case 0:
			{
				break;
			}
			default:
			{
				requestSensorData = 5;
				break;
			}

			if (requestSensorData > 0)
			{
				if((getUartData == 0) || (getUartData_failed > 5))
				{
					requestSensorData = 5;
					getUartData = 1;
					getUartData_failed = 0;
				}
				else
				{
					getUartData_failed++;
				}
			}
		}
	}
	else if (send_type == UART_SEND_TYPE_HEALTH_DATA)
	{
		data_len = generateSimplePacket(data, 4, 17, 0);
	}

	if (data_len > 0) {
		writeTxBuffer(data, data_len);
	}
	TxBuffer_MutexUnlock();
	return data_len;
}

//this function should be called after uart_sendCmd(UART_SEND_TYPE_SENSOR_DATA), every 2 sceonds
void uart_sensorCmdTimeoutCheck(void)
{
	if (requestSensorData == 0) {
		checkUART_comm_cnt = 0;
		return;
	}

	checkUART_comm_cnt++;
	if (checkUART_comm_cnt == 10 || checkUART_comm_cnt == 20) {
		ESP_LOGW(UART_TAG, "STM32 no response to sensor command (%ds)", checkUART_comm_cnt*2);
	}

	//UART no response timeout is ~60s
	if (checkUART_comm_cnt == (60/2)) {
		if (!BLE_IsEnabled()) {
			BLE_SetDebugMode(true);
		}
		Call_BT_enable = 1;
		ESP_LOGE(UART_TAG, "STM32 no response to sensor command, enable BT....");
	}
	//reset STM32 if not receiving response for 10 minutes
	else if (checkUART_comm_cnt == ((10*60)/2)) {
		checkUART_comm_cnt = 0;
		ESP_LOGE(UART_TAG, "fail to receive STM command for a long time, reset STM32!!!");
#ifdef USE_STM32_CT_BOOTLOADER
		Suspend_SensorTask = 1;
		Suspend_Uart = 1;
		vTaskDelay(400/portTICK_RATE_MS);
		uart_flushData();
#endif
		gpio_set_level(STM_RST_PIN, 0);
		vTaskDelay(200/portTICK_RATE_MS);
		gpio_set_level(STM_RST_PIN, 1);
#ifdef USE_STM32_CT_BOOTLOADER
		vTaskDelay(200/portTICK_RATE_MS);
		BootLoader_GoCmd();
		vTaskDelay(200/portTICK_RATE_MS);
		Suspend_SensorTask = 0;
		Suspend_Uart = 0;
#endif

		//don't request sensor data for 8*2=16 seconds
		uart_skipToSendSensorDataCmd(8);

		//Update: avid WiFi disconnection
		////disconnect WiFi to make sure that all necessary commands send to STM32 again
		//if(wifi_GetConnectionState() == WIFI_STATE_CONNECTED) {
		//	extern esp_err_t esp_wifi_disconnect(void);
		//	esp_wifi_disconnect();
		//}
	}
}

//only call this function from external side
bool uart_isSendCmdReady(void)
{
	bool bReady = false;
	TxBuffer_MutexLock();
	if (uart_ctx.TxBuffer_len > 0) {
		bReady = false;
	} else {
		bReady = true;
	}
	TxBuffer_MutexUnlock();
	return bReady;
}

static void uart_RxParseCmd(uint8_t *cmd, uint8_t cmd_len)
{
	cmd_header_t *pHeader = NULL;
	uint8_t *pData = NULL;
	uint8_t data_len = 0;
	if (cmd_len < sizeof(cmd_header_t)) {
		return;
	}

	//reset uart checking count
	checkUART_comm_cnt = 0;

	pHeader = (cmd_header_t *)cmd;
	pData = cmd + sizeof(cmd_header_t);
	data_len = cmd_len - sizeof(cmd_header_t);
	//cmd_type = 1 (RX_CMD_TYPE_SENSOR_DATA), Sensor Data Response Type
	if (pHeader->cmd_type == RX_CMD_TYPE_SENSOR_DATA)
	{
		bool bExitLoop = false;
		uint16_t offset = 0;

		//new sensor data checking
		isNewData = 1;
		getUartData = 0;

		//print time for debugging
		if (1) {
			time_t now = 0;
			struct tm timeinfo = {0};
			time(&now);
			localtime_r(&now, &timeinfo);
			strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
			printf("[%s]\n", strftime_buf);
		}
		
		while (offset < cmd_len)
		{
			cmd_sensorData_t *pSensorCmd = (cmd_sensorData_t *)(cmd + offset);
			if (pSensorCmd->hdr.cmd_type != RX_CMD_TYPE_SENSOR_DATA)
			{
				//not sensor data
				break;
			}
			offset += sizeof(pSensorCmd->hdr);
			switch (pSensorCmd->hdr.cmd_id)
			{
				case 1://CO2 Sensor (SCD30)
				{
					offset += sizeof(pSensorCmd->data.CO2_sensor);
					if (firmwareVersion == 0x11)
					{
						sensorVal[3] = pSensorCmd->data.CO2_sensor.CO2;
						BLE_send_CO2 = pSensorCmd->data.CO2_sensor.CO2;
					#ifdef TH_TESTING
						sensorVal[5] = pSensorCmd->data.CO2_sensor.Temp;
						sensorVal[6] = pSensorCmd->data.CO2_sensor.Humidity;
					#endif
						sen_dbg_printf("Printf_CO2: %0.2f\n", pSensorCmd->data.CO2_sensor.CO2);
						sen_dbg_printf("Printf_CO2_temp: %0.2f\n", pSensorCmd->data.CO2_sensor.Temp);
						sen_dbg_printf("Printf_CO2_humidity: %0.2f\n", pSensorCmd->data.CO2_sensor.Humidity);
					}
					break;
				}
				case 2://PM2.5 Sensor (SPS30)
				{
					offset += sizeof(pSensorCmd->data.PM_2_5_sensor);
					if (firmwareVersion == 0x11)
					{
						sensorVal[4] = pSensorCmd->data.PM_2_5_sensor.PM_2_5;
						BLE_send_PM2p5 = pSensorCmd->data.PM_2_5_sensor.PM_2_5;
						sen_dbg_printf("Printf_PM_2_5: %0.6f\n", pSensorCmd->data.PM_2_5_sensor.PM_2_5);
					}
					break;
				}
				case 3://Temperature Sensor (Si7021)
				{
					offset += sizeof(pSensorCmd->data.Temp_sensor);
					sensorVal[0] = pSensorCmd->data.Temp_sensor.Temp;
					sensorVal[1] = pSensorCmd->data.Temp_sensor.Humidity;
					BLE_send_Temperature = pSensorCmd->data.Temp_sensor.Temp;
					BLE_send_Humidity = pSensorCmd->data.Temp_sensor.Humidity;
					sen_dbg_printf("Printf_temp: %0.2f\n", pSensorCmd->data.Temp_sensor.Temp);
					sen_dbg_printf("Printf_humidity: %0.2f\n", pSensorCmd->data.Temp_sensor.Humidity);
					break;
				}
				case 4://TVOC Sensor (ZMOD4410)
				{
					offset += sizeof(pSensorCmd->data.TVOC_sensor);
					sensorVal[2] = pSensorCmd->data.TVOC_sensor.TVOC;
					BLE_send_TVOC = pSensorCmd->data.TVOC_sensor.TVOC;
				#ifdef TH_TESTING
					sensorVal[7] = pSensorCmd->data.TVOC_sensor.R_MOX_raw_data;
					sensorVal[8] = pSensorCmd->data.TVOC_sensor.IAQ;
				#endif
					if(firmwareVersion == 0x21)
					{
						sensorVal[3] = pSensorCmd->data.TVOC_sensor.eCO2;
						BLE_send_eCO2 = pSensorCmd->data.TVOC_sensor.eCO2;
					}
					sen_dbg_printf("Printf_TVOC: %0.6f\n", pSensorCmd->data.TVOC_sensor.TVOC);
					sen_dbg_printf("Printf_R_MOX_raw_data: %0.2f\n", pSensorCmd->data.TVOC_sensor.R_MOX_raw_data);
					sen_dbg_printf("Printf_IAQ: %0.2f\n", pSensorCmd->data.TVOC_sensor.IAQ);
					sen_dbg_printf("Printf_eCO2: %0.2f\n", pSensorCmd->data.TVOC_sensor.eCO2);
					break;
				}
				case 5://Temperature Sensor Raw Data (Si7021)
				{
					offset += sizeof(pSensorCmd->data.Temp_sensor_RAW);
					TempSensorRawData_t data = {0};
					appCommon_GetTempSensorRawData(&data);
					data.RawTemp = pSensorCmd->data.Temp_sensor_RAW.RawTemp;
					data.RawHumidity = pSensorCmd->data.Temp_sensor_RAW.RawHumidity;
					data.RawNTC1 = pSensorCmd->data.Temp_sensor_RAW.RawNTC1;
					data.RawNTC2 = pSensorCmd->data.Temp_sensor_RAW.RawNTC2;
					data.RawNTC3 = pSensorCmd->data.Temp_sensor_RAW.RawNTC3;
					appCommon_SetTempSensorRawData(&data);
					sen_dbg_printf("Printf_RawTemp: %0.2f\n", pSensorCmd->data.Temp_sensor_RAW.RawTemp);
					sen_dbg_printf("Printf_RawHumidity: %0.2f\n", pSensorCmd->data.Temp_sensor_RAW.RawHumidity);
					sen_dbg_printf("Printf_RawNTC1: %0.2f\n", pSensorCmd->data.Temp_sensor_RAW.RawNTC1);
					sen_dbg_printf("Printf_RawNTC2: %0.2f\n", pSensorCmd->data.Temp_sensor_RAW.RawNTC2);
					sen_dbg_printf("Printf_RawNTC3: %0.2f\n", pSensorCmd->data.Temp_sensor_RAW.RawNTC3);
					break;
				}
				default:
				{
					//If it is unknown command, don't decode remaining data
					ESP_LOGE(UART_TAG, "unknown sensor cmd");
					bExitLoop = true;
					break;
				}
			}
			if (bExitLoop)
			{
				break;
			}
		}
	}
	//cmd_type = 2 (RX_CMD_TYPE_KEY_CONTROL), Key Control Type
	else if (pHeader->cmd_type == RX_CMD_TYPE_KEY_CONTROL)
	{
		switch (pHeader->cmd_id)
		{
			case 1://Device On/Off
			{
				printf("Device On/Off: %d\n", pData[0]);
				break;
			}
			case 2://CIAQ Override
			{
				printf("CIAQ Override: %d\n", pData[0]);
				break;
			}
			case 3://BLE Pairing
			{
				if (data_len >= 1)
				{
					if (pData[0])
					{
						if(!BLE_IsEnabled()) {
							Call_BT_enable = 1;
						} else {
							BLE_ResetEnableTimeout();
							//send BT status
							switchAppCmd |= (1UL << SW_APPCMD_BT_STATUS);
						}
					}
					else
					{
						if(BLE_IsEnabled()) {
							Call_BT_disable = 1;
						} else {
							//send BT status
							switchAppCmd |= (1UL << SW_APPCMD_BT_STATUS);
						}
					}
					printf("BLE Pairing: %d\n", pData[0]);
				}
				break;
			}
			case 4://Button 1 Status
			{
				if (data_len >= 1)
				{
					Bttn1RealState = pData[0];
					printf("Bttn1RealState = %u\n", Bttn1RealState);
				}
				break;
			}
			case 5://Button 2 Status
			{
				if (data_len >= 1)
				{
					Bttn2RealState = pData[0];
					printf("Bttn2RealState = %u\n", Bttn2RealState);
				}
				break;
			}
			case 6://System Reset
			{
				system_event_run(SYS_EVENT_WIFI_RESET);
				break;
			}
			default:
			{
				ESP_LOGE(UART_TAG, "unknown cmd (type=%d id=%d)", pHeader->cmd_type, pHeader->cmd_id);
				break;
			}
		}
	}
	//cmd_type = 3 (RX_CMD_TYPE_POWER_CONTROL), Power Control Type
	else if (pHeader->cmd_type == RX_CMD_TYPE_POWER_CONTROL)
	{
		switch (pHeader->cmd_id)
		{
			case 1://Relay Status
			{
				if (data_len >= 1)
				{
					FanRealState = pData[0];
					printf("FanRealState = %u\n", FanRealState);
				}
				break;
			}
			case 2://Triac 1 status
			{
				printf("Triac 1 status: %d\n", pData[0]);
				break;
			}
			case 3://Triac 2 status
			{
				printf("Triac 2 status: %d\n", pData[0]);
				break;
			}
			default:
			{
				ESP_LOGE(UART_TAG, "unknown cmd (type=%d id=%d)", pHeader->cmd_type, pHeader->cmd_id);
				break;
			}
		}
	}
	//cmd_type = 4 (RX_CMD_TYPE_SPECIAL), Special Type
	else if (pHeader->cmd_type == RX_CMD_TYPE_SPECIAL)
	{
		switch (pHeader->cmd_id)
		{
			case SW_STM2ESP_VER_RESPONSE: //Version Response to ESP32
			{
				if (data_len >= 3)
				{
					if (data_len >= 4) {
						//4th byte data, 0x00=start address 0x0000, 0x01=start address 0x8000
						printf("STM32 Version Response: %d . %d . %d (0x%02x)\n", pData[0], pData[1], pData[2], pData[3]);
					} else {
						printf("STM32 Version Response: %d . %d . %d\n", pData[0], pData[1], pData[2]);
					}
					STM_FW_VER_Major = pData[0];
					STM_FW_VER_Minor = pData[1];
					STM_FW_VER_Sub =  pData[2];
					STM_FW_VER_bValid = true;
				}
				break;
			}
			case SW_STM2ESP_MODEL_RESPONSE: //Model Response to ESP32
			{
				if (data_len >= 2)
				{
					printf("STM32 Model Response: 0x%02x 0x%02x\n", pData[0], pData[1]);
					if (pData[0] == 0x00) {
						//old STM32 FW uses old format
						if (pData[1] == 011) {
							STM_FW_MODEL = 0x11;
						} else if (pData[1] == 021) {
							STM_FW_MODEL = 0x21;
						} else {
							STM_FW_MODEL = 0x00;
						}
					} else {
						STM_FW_MODEL = pData[0];
					}
					STM_FW_MODEL_bValid = true;
				}
				break;
			}
			case SW_STM2ESP_MOVERRIDE: //MOverride
			{
				if (data_len >= 1)
				{
					MOverride = pData[0];
					printf("MOverride = %u\n", MOverride);
				}
				break;
			}
			case SW_STM2ESP_STM_FW_SIGN:
			{
				ESP_LOGI(UART_TAG, "Got signature");
			#ifdef BROAN_SECURE_RELEASE
				if (data_len >= sizeof(STM_FW_SIGN_ARR))
				{
					if (memcmp(STM_FW_SIGN_ARR, pData, sizeof(STM_FW_SIGN_ARR)) == 0)
					{
						ESP_LOGI("UART", "Signature is valid!");
						STM_FW_SIGN_bValid = true;
					}
				}
			#endif
				break;
			}
			case SW_STM2ESP_STM_SYSTEM_START:
			{
				ESP_LOGI(UART_TAG, "STM system start");
				system_event_run(SYS_EVENT_STM32_START);
				break;
			}
			default:
			{
				ESP_LOGE(UART_TAG, "unknown cmd (type=%d id=%d)", pHeader->cmd_type, pHeader->cmd_id);
				break;
			}
		}
	}
}

static void uart_RxHandler(uint8_t *buffer, uint16_t buffer_len)
{
	uint16_t rx_index_prev = 0;
	uint16_t rx_index = 0;
	uint8_t *pPacket = NULL;
	uint8_t packet_len = 0;
	uint8_t checksum;
	if (buffer_len <= 0) {
		return;
	}

	//print debug message
	ESP_LOGW(UART_TAG, "RX <<");
	ESP_LOG_BUFFER_HEX_LEVEL(UART_TAG, buffer, buffer_len, ESP_LOG_WARN);

	while (rx_index < buffer_len) {
		rx_index_prev = rx_index;
		//find the start byte position
		while (rx_index < buffer_len) {
			if (buffer[rx_index] == UART_START_BYTE) {
				break;
			}
			rx_index++;
		}
		if (rx_index >= buffer_len) {
			//no start byte is found
			ESP_LOGE(UART_TAG, "No start byte");
			uart_stat.rx_no_start_byte++;
			return;
		}
		if (rx_index != rx_index_prev) {
			//some dummy data before start byte
			ESP_LOGE(UART_TAG, "dummy data (%d)", rx_index - rx_index_prev);
			uart_stat.rx_dummy_data++;
			if (rx_index_prev == 0)
			{
				//try to recovery if there are some partial data
				if(rx_index <= (sizeof(uart_ctx.RxPartialData) - uart_ctx.RxPartialData_len))
				{
					memcpy(&uart_ctx.RxPartialData[uart_ctx.RxPartialData_len], buffer, rx_index);
					uart_ctx.RxPartialData_len += rx_index;

					pPacket = uart_ctx.RxPartialData;
					packet_len = pPacket[UART_HEADER_LENGTH_POS] + 1; // total packet length
					if (packet_len <= uart_ctx.RxPartialData_len) {
						checksum = generate_crc_8bit(pPacket, packet_len-1);
						if (checksum != pPacket[packet_len-1]) {
							//checksum incorrect
							ESP_LOGI(UART_TAG, "checksum incorrect");
							uart_stat.rx_checksum_incorrect++;
						} else {
							
							//print debug message
							ESP_LOGI(UART_TAG, "RX len=%d", packet_len);
							//ESP_LOG_BUFFER_HEX_LEVEL(UART_TAG, pPacket, packet_len, ESP_LOG_INFO);

							uart_RxParseCmd(&pPacket[UART_DATA_START_POS], packet_len-UART_DATA_START_POS-1);
						}
					}
				}
			}
		}

		//clear partial data
		uart_ctx.RxPartialData_len = 0;

		//check if packet is incomplete
		bool bIncompletePacket = false;
		if ((rx_index + UART_HEADER_LENGTH_POS) > buffer_len) {
			bIncompletePacket = true;
		}
		pPacket = &buffer[rx_index];
		packet_len = pPacket[UART_HEADER_LENGTH_POS] + 1; // total packet length
		if ((rx_index + packet_len) > buffer_len) {
			bIncompletePacket = true;
		}

		if (bIncompletePacket) {
			//packet incomplete
			ESP_LOGE(UART_TAG, "packet incomplete (%d)", buffer_len - rx_index);
			uart_stat.rx_packet_incomplete++;
			if ((buffer_len - rx_index) <= sizeof(uart_ctx.RxPartialData)) {
				uart_ctx.RxPartialData_len = buffer_len - rx_index;
				memcpy(uart_ctx.RxPartialData, &buffer[rx_index], uart_ctx.RxPartialData_len);
			}
			return;
		} else {
			checksum = generate_crc_8bit(pPacket, packet_len-1);
			if (checksum != pPacket[packet_len-1]) {
				//checksum incorrect
				ESP_LOGE(UART_TAG, "checksum incorrect");
				uart_stat.rx_checksum_incorrect++;
			} else {
				//print debug message
				if (1) {
					//time_t now = 0;
					//struct tm timeinfo = { 0 };
					//time(&now);
					//localtime_r(&now, &timeinfo);
					//strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
					//printf("[%s]\n", strftime_buf);
					//printf("(%d) RX << ", esp_log_timestamp());
					//for (uint8_t i = 0; i < packet_len; i++) {
					//	printf("%x, ", pPacket[i]);
					//}
					//printf("\n");
					ESP_LOGI(UART_TAG, "RX offset=%d len=%d", rx_index, packet_len);
					//ESP_LOG_BUFFER_HEX_LEVEL(UART_TAG, pPacket, packet_len, ESP_LOG_INFO);
				}

				uart_RxParseCmd(&pPacket[UART_DATA_START_POS], packet_len-UART_DATA_START_POS-1);
			}
		}

		rx_index += packet_len;
	}
}

static void uart_TxHandler(void)
{
	TxBuffer_MutexLock();
	if (uart_ctx.TxBuffer_len > 0) {
		//print debug message
		if (1) {
			ESP_LOGW(UART_TAG, "TX >>");
			ESP_LOG_BUFFER_HEX_LEVEL(UART_TAG, uart_ctx.TxBuffer, uart_ctx.TxBuffer_len, ESP_LOG_WARN);
		}

		uart_write_bytes(EX_UART_NUM, (const char *)uart_ctx.TxBuffer, uart_ctx.TxBuffer_len);
		uart_ctx.TxBuffer_len = 0;
	}
	TxBuffer_MutexUnlock();
}

void uart_TxRxTask(void *arg)
{
	uint16_t rx_wait_ms;
	int rx_read_len;

	//reset uart checking count
	checkUART_comm_cnt = 0;

	gpio_set_direction(18, GPIO_MODE_OUTPUT);
	gpio_set_level(18, 1);
	vTaskDelay(50/portTICK_RATE_MS);

	#define UART_RX_INTERVAL	(250) //in ms
	if (UART_RX_INTERVAL > 100) {
		rx_wait_ms = UART_RX_INTERVAL - 100;
	} else {
		rx_wait_ms = 0;
	}
	while(1)
	{
		uart_TxHandler();
		vTaskDelay(100/portTICK_RATE_MS);

		//optimize by reading uart 2 times
		rx_read_len = uart_read_bytes(EX_UART_NUM, uart_ctx.RxBuffer, sizeof(uart_ctx.RxBuffer), (rx_wait_ms/2)/portTICK_RATE_MS);
		if (rx_read_len > 0) {
			uart_RxHandler(uart_ctx.RxBuffer, rx_read_len);
		}
		rx_read_len = uart_read_bytes(EX_UART_NUM, uart_ctx.RxBuffer, sizeof(uart_ctx.RxBuffer), (rx_wait_ms/2)/portTICK_RATE_MS);
		if (rx_read_len > 0) {
			uart_RxHandler(uart_ctx.RxBuffer, rx_read_len);
		}

		if (Suspend_Uart) {
			while (Suspend_Uart) {
				vTaskDelay(100 / portTICK_PERIOD_MS);
			}
			ESP_LOGI(UART_TAG, "resume UART");
			checkUART_comm_cnt = 0;
		}
	}

	vTaskDelete(NULL);
}




void Usart2_Write(uint8_t *data, uint16_t size)
{
	uint8_t data_[136];
	uart_write_bytes(EX_UART_NUM, (const char*)data, size);
	printf("Usart2_Write >> ");
	if (size > 32) {
		printf(".....");
	} else {
		for (uint8_t i = 0; i < size; i++){
			data_[i] = data[i];
			printf(" %X", data_[i]);
		}
	}
	printf("\r\n");
	
}

int Usart2_Read(uint8_t *data, uint16_t size)
{
	int rx = uart_read_bytes(EX_UART_NUM, data, size, 1000/portTICK_RATE_MS);
	if (rx > 0) {
		printf("Usart2_Read << ");
		if (rx > 32) {
			printf(".....");
		} else {
			for (uint16_t i = 0; i < rx; i++) {
				printf(" %X", data[i]);
			}
		}
		printf("\r\n");
	}
	return rx;
}

