//#include "ESP32_STM_Comm.h"
#include "main.h"
#include "Si7021_cal.h"


extern UART_HandleTypeDef huart1;
extern LED_Control_data_ LED_Control_data;
extern SensorsOutRawData_ SensorsOutRawData;
extern uint8_t BL_LED1_status;
extern uint8_t BL_LED2_status;
extern uint8_t FAN1_status;
extern uint8_t RGB_LED_status;
extern uint8_t Relay_status;
extern uint8_t Switch_config_setting;
extern uint8_t Switch_config_setting_B4;
extern uint8_t ESP_Cmd_Send_En;
extern uint8_t ESP_Cmd_Rev_Proc;
extern uint16_t SCD30_FRC_Value;
extern uint8_t BLE_Enable_status;
RX_REQ_decode_t RX_REQ_decode;
extern uint8_t HW_VER;
extern uint8_t just_StartUP_tick;
extern bool just_StartUP_canSend;
extern uint8_t OnOff_BL1_func;
extern uint32_t APP_Vect_offset;
extern algorithm_version iaq_2nd_gen_ver;

extern void GetSTM_SignData(uint8_t *Data);

void Reinit_UART1(void);

extern uint8_t UART1_reinit_Cnt;

//extern algorithm_version iaq_2nd_gen_ver;

void STM_ESP_Comm(void){
  uint16_t i;
  uint8_t aTxBuffer[ESP_STM_UART_BUF_SIZE], index;
  uint8_t localRxBuffer[ESP_STM_UART_BUF_SIZE] = {0};
//  uint16_t NTC1_RawADCvalue_, NTC2_RawADCvalue_, NTC3_RawADCvalue_;
//  UartRevStruct_t ESP32toSTMRevData;
//  UartSendStruct_t STMtoESP32SendData;
            /*
                memset(aRxBuffer, 0, sizeof(aRxBuffer));
                UART_Transfering = 1;
                ret = HAL_UART_Receive_IT(&huart1, aRxBuffer, sizeof(ESP32toSTMRevData));
                UART_Transfering = 0;
                HAL_Delay(10);
                Uart1ReadyCnt = 0;
                while (Uart1Ready != SET)
                {
                  if (Uart1ReadyCnt < Uart1ReadyCntMax){
                      Uart1ReadyCnt++;
                  }else{
                      HAL_UART_Abort_IT(&huart1);
                      Uart1Ready = SET;
                      return;
                  }
                }
                Uart1Ready = RESET;
                */
                  
              if (UART1_reinit_Cnt < 55){
//                DEBUG_UART("UART1_reinit_Cnt = %d, Uart1Ready = %d\r\n", UART1_reinit_Cnt, Uart1Ready);
              }else{
                memset(localRxBuffer, 0, sizeof(localRxBuffer));
                Reinit_UART1();
                UART1_reinit_Cnt = 0;
              }
                  
  memset(&RX_REQ_decode, 0, sizeof(RX_REQ_decode));
    if (Uart1Ready == SET){
        for (i = 0; i < sizeof(aRxBuffer); i++){
          if (aRxBuffer[i] == START_BYTE && i < sizeof(aRxBuffer) - RX_BUF_REDUCE){
            if ((i + aRxBuffer[i+1] + 1) < ESP_STM_UART_BUF_SIZE){
              memcpy(localRxBuffer, &aRxBuffer[i], aRxBuffer[i+1]+1);
              break;
            }else{
              __HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);
              memset(aRxBuffer, 0, sizeof(aRxBuffer));
              return;
            }
            
          }
          //DEBUG_UART("0x%2X ", aRxBuffer[i]);
        }
#ifdef DEBUG_UART_
        DEBUG_UART("RX << ");
        for (i = 0; i <= localRxBuffer[1]; i++){
          DEBUG_UART("0x%2X ", localRxBuffer[i]);
        }
        DEBUG_UART("\r\n");
#endif

      msg = aRxBuffer;
      (&huart1)->pRxBuffPtr = msg;
      Uart1Ready = RESET;
      int8_t ChkSum;
      ChkSum = sensirion_common_check_crc(localRxBuffer, localRxBuffer[1], localRxBuffer[localRxBuffer[1]]);
      
      if (ChkSum != STATUS_OK){
//          DEBUG_UART("Checksum error = %2X\r\n", ChkSum);
          __HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);
          memset(aRxBuffer, 0, sizeof(aRxBuffer));
          return;
      }
    }
    
    if (localRxBuffer[3] == BLE_ENABLE_STATUS){
      if (localRxBuffer[2] == SPEC_TYPE_REQ && localRxBuffer[4] == 0x00 && localRxBuffer[5] == 0xE5){
        ESP_Cmd_Send_En = 0;
      }        
    }
    
    if (localRxBuffer[0] != 0x00 && ESP_Cmd_Send_En == 0){
        //Decode localRxBuffer data
      ESP_Cmd_Rev_Proc = 1;
        i = 2;
        while (i <= localRxBuffer[1]){
          if (localRxBuffer[i] == SENSOR_REQ){
            i++;
            switch (localRxBuffer[i]){
            case SCD30_RES:
              RX_REQ_decode.SCD30_data_requested = true;
              break;
            case SPS30_RES:
              RX_REQ_decode.SPS30_data_requested = true;
              break;
            case SI7021_REQ:
              RX_REQ_decode.Si7021_data_requested = true;
              break;
            case ZMOD4410_REQ:
              RX_REQ_decode.ZMOD4410_data_requested = true;       
//              UART1_reinit_Cnt = 0;
              break;
            };
            i+=2; //index + ID + 1 data
          }else if (localRxBuffer[i] == LED_CONTROL){
            i++;
            switch (localRxBuffer[i]){
            case LED_1:
              RX_REQ_decode.LED1_control_requested = true;
              RX_REQ_decode.LED1_isON = localRxBuffer[i + 1];
              break;
            case LED_2:
              RX_REQ_decode.LED2_control_requested = true;
              RX_REQ_decode.LED2_isON = localRxBuffer[i + 1];
              break;
            case IAQ_COLOR:
              RX_REQ_decode.IAQcolor_control_requested = true;
              RX_REQ_decode.IAQ_color_value= localRxBuffer[i + 1];
              break;
            case IAQ_INDEX:
//              RX_REQ_decode.IAQindex_control_requested = true;
//              RX_REQ_decode.IAQ_index_value= localRxBuffer[i + 1];
              break;
            case LED_ANIMATION:
//              RX_REQ_decode.LEDAnimation_control_requested = true;
//              RX_REQ_decode.LEDAnimation= localRxBuffer[i + 1];
              break;
            case BT_PAIRING:
//              RX_REQ_decode.BT_Pairing = true;
              break;
            case BT_PAIR_OK:
              //RX_REQ_decode.BT_Pair_OK = true;
              break;
            case WIFI_CONNECT_OK:
              RX_REQ_decode.Wifi_Connect_OK = true;
              //DEBUG_UART("Wifi_Connect_OK................................\r\n");
              break;
            case WIFI_CONNECTED_NOT_ACTIVE:
              //RX_REQ_decode.Wifi_Connected_Not_Active = true;
              break;
            case WIFI_CONNECTED_ACTIVE:
              //RX_REQ_decode.Wifi_Connected_Active = true;
              break;
            case WIFI_DISCONNECTED:
              RX_REQ_decode.Wifi_Disconnected = true;
              //DEBUG_UART("WIFI_DISCONNECTED................................\r\n");
              break;
            case MANUAL_ON:
              //RX_REQ_decode.Manual_on = true;
              break;
            case NO_WIFI_BT:
              //RX_REQ_decode.No_WiFi_BT = true;
              break;
            case IAQ_LED_INTENSITY:
              RX_REQ_decode.IAQ_Intensity_requested = true;
              RX_REQ_decode.IAQ_Intensity_value= localRxBuffer[i + 1];
              break;
                     
            };
            i+=2; //index + ID + 1 data
          }else if (localRxBuffer[i] == POWER_CONTROL){
            i++;
            switch (localRxBuffer[i]){
            case TRIAC_1:
//              RX_REQ_decode.Triac1_control_requested = true;
//              RX_REQ_decode.Triac1_value = localRxBuffer[i + 1];
              break;
            case TRIAC_2:
//              RX_REQ_decode.Triac2_control_requested = true;
//              RX_REQ_decode.Triac1_value = localRxBuffer[i + 1];
              break;
            case FAN_CLOUD_STATE:
              RX_REQ_decode.FanCloudState_control_requested = true;
              RX_REQ_decode.FanCloudState_Command = localRxBuffer[i + 1];
              DEBUG_UART("FAN_CLOUD_STATE RX_REQ_decode.FanCloudState_Command =%d \r\n", RX_REQ_decode.FanCloudState_Command );
              break;
              
            };
            i+=2; //index + ID + 1 data
          }else if (localRxBuffer[i] == SPEC_TYPE_REQ){
            i++;
            switch (localRxBuffer[i]){
            case VER_REQ_ESP_TO_STM:
              RX_REQ_decode.Version_requested = true;
              break;
            case VER_RES_ESP_TO_STM:
              RX_REQ_decode.Ver_response_from_ESP = true;
              RX_REQ_decode.ESP_MainVer = localRxBuffer[i+1];
              RX_REQ_decode.ESP_MinorVer = localRxBuffer[i+2];
              RX_REQ_decode.ESP_SubVer = localRxBuffer[i+3];
              i+=2;     //Ver have more than 2 data
              break;
            case MODEL_REQ_ESP_TO_STM:
              RX_REQ_decode.Model_requested = true;
              break;
            case MODEL_RES_ESP_TO_STM:
              RX_REQ_decode.Model_response_from_ESP = true;
              RX_REQ_decode.ESP_ModelNoMsb = localRxBuffer[i+1];
              RX_REQ_decode.ESP_ModelNoLsb = localRxBuffer[i+2];
              i+=1;     //Ver have more than 1 data
              break;
            case SWITCH_CONFIG:
              RX_REQ_decode.Switch_config_change_req = true;
              RX_REQ_decode.Switch_config = localRxBuffer[i+1];

              break;
            case ENG_CO2_FRC_TURN:
              RX_REQ_decode.EngCo2FrcTurn_change_req = true;
              RX_REQ_decode.Eng_Co2FrcValue = localRxBuffer[i+1] << 8 | localRxBuffer[i+2];
              i += 1;
              break;
            case BLE_ENABLE_STATUS:
              RX_REQ_decode.BleEnableStatus_change_req = true;
              RX_REQ_decode.BleEnableStatus_Value = localRxBuffer[i+1];      
              break;
            case SIGN_REQ_ESP_TO_STM:
              RX_REQ_decode.Send_Sign_to_ESP_req = true;
              break;
            case HW_VER_FROM_ESP:
              HW_VER = localRxBuffer[i+1];
              if (HW_VER < 1 || HW_VER > 2){
                HW_VER = 1;
              }
              DEBUG_UART("______________ Got HW_VER = %d \r\n", HW_VER);
              break;
            case ONOFF_BL1_FUNC:
              OnOff_BL1_func = localRxBuffer[i+1];
              if (OnOff_BL1_func == 1){
                FLASH_OBProgramInitTypeDef OptionsBytesStruct;
                HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);
                if (OptionsBytesStruct.USERConfig != 0x24F6000){
                      HAL_FLASH_Unlock();
                      HAL_FLASH_OB_Unlock();
                      OptionsBytesStruct.USERConfig = 0x24F6000;
                      if(HAL_FLASHEx_OBProgram(&OptionsBytesStruct) != HAL_OK)
                       {
                         printf("Fail to USERConfig Settings! \r\n");
                       }else{
                         printf("Read-Out USERConfig OK! \r\n");
                       }
                      HAL_FLASH_OB_Lock();
                      printf("HAL_FLASH_OB_Lock OK! \r\n");
                      HAL_FLASH_Lock();
                      HAL_PWR_EnterSTANDBYMode();
                      printf("HAL_FLASH_Lock OK! \r\n");
                      HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);
                      printf("OptionsBytesStruct.USERConfig = %X\r\n", OptionsBytesStruct.USERConfig);
                }
              }
              break;
            case HEALTH_CHECK:
              DEBUG_UART("Got Health Packet\r\n");
              UART1_reinit_Cnt = 0;
              break;
            };
            i+=2; //index + ID + 1 data
          }else if (localRxBuffer[i] == BUTTON_STATUS_CONTROL){
            i++;
            switch (localRxBuffer[i]){
            case BTN_1_CONT_STATUS:
              RX_REQ_decode.Btn1_status_cont_req = true;
              RX_REQ_decode.Btn1_isON = localRxBuffer[i+1];
              break;
            case BTN_2_CONT_STATUS:
              RX_REQ_decode.Btn2_status_cont_req = true;
              RX_REQ_decode.Btn2_isON = localRxBuffer[i+1];
              DEBUG_UART("BTN_2_CONT_STATUS RX_REQ_decode.Btn2_isON = %d\r\n", RX_REQ_decode.Btn2_isON);
              break;
            case FAN_APP_STATUS:
              RX_REQ_decode.Fan_App_Status_Req = true;
              RX_REQ_decode.Fan_App_Status_Value = localRxBuffer[i+1];
              DEBUG_UART("FAN_APP_STATUS RX_REQ_decode.Fan_App_Status_Value = %d\r\n", RX_REQ_decode.Fan_App_Status_Value);
              break;
            };
            i+=2; //index + ID + 1 data
          }else{
            i++;
          }
        }
      
      index = 0;
      memset(aTxBuffer, 0, sizeof(aTxBuffer));
      aTxBuffer[index] = START_BYTE; index++;
      aTxBuffer[index] = SCD30_RES_DATA_LEN; index++;
      
      if (RX_REQ_decode.SCD30_data_requested){
            aTxBuffer[index] = SENSOR_RES; index++; //type
            aTxBuffer[index] = SCD30_RES; index++;      //ID
            memcpy(&aTxBuffer[index], &SensorsOutRawData.SCD30outRawData, sizeof(SensorsOutRawData.SCD30outRawData));
            index += sizeof(SensorsOutRawData.SCD30outRawData);
      }
      if (RX_REQ_decode.SPS30_data_requested){
        aTxBuffer[index] = SENSOR_RES; index++; //type
            aTxBuffer[index] = SPS30_RES; index++;
            memcpy(&aTxBuffer[index], &SensorsOutRawData.PM2p5_32bit, sizeof(SensorsOutRawData.PM2p5_32bit));
            index += sizeof(SensorsOutRawData.PM2p5_32bit);
      }
      if (RX_REQ_decode.Si7021_data_requested){
            aTxBuffer[index] = SENSOR_RES; index++; //type
            aTxBuffer[index] = SI7021_RES; index++;

            memcpy(&aTxBuffer[index], &SensorsOutRawData.Si7021outRawData, 8);//sizeof(SensorsOutRawData.Si7021outRawData));
            index += 8;//sizeof(SensorsOutRawData.Si7021outRawData);   
            aTxBuffer[index] = SENSOR_RES; index++; //type
            aTxBuffer[index] = SI7021_RAW_RES; index++;
            memcpy(&aTxBuffer[index], &SensorsOutRawData.Si7021outRawData.RawTemp, 4); index += 4;
            memcpy(&aTxBuffer[index], &SensorsOutRawData.Si7021outRawData.RawHumidity, 4); index += 4;
            memcpy(&aTxBuffer[index], &SensorsOutRawData.Si7021outRawData.RawNTC1, 4); index += 4;
            memcpy(&aTxBuffer[index], &SensorsOutRawData.Si7021outRawData.RawNTC2, 4); index += 4;
            memcpy(&aTxBuffer[index], &SensorsOutRawData.Si7021outRawData.RawNTC3, 4); index += 4;
#ifdef IS_TH_TEST_FW      
            aTxBuffer[index] = SENSOR_RES; index++; //type
            aTxBuffer[index] = STM_OTHER_STATUS; index++;
            memcpy(&aTxBuffer[index], &LED_Control_data.IAQvalue, 1); index += 1;
            memcpy(&aTxBuffer[index], &LED_Control_data.IAQ_Intensity_value, 1); index += 1;
            NTC1_RawADCvalue_ = GetNTC1_rawADCvalue();
            NTC2_RawADCvalue_ = GetNTC2_rawADCvalue();
            NTC3_RawADCvalue_ = GetNTC3_rawADCvalue();
            memcpy(&aTxBuffer[index], &NTC1_RawADCvalue_, 2); index += 2;
            memcpy(&aTxBuffer[index], &NTC2_RawADCvalue_, 2); index += 2;
            memcpy(&aTxBuffer[index], &NTC3_RawADCvalue_, 2); index += 2;
            
#endif
      }
      if (RX_REQ_decode.ZMOD4410_data_requested){
        aTxBuffer[index] = SENSOR_RES; index++; //type
            aTxBuffer[index] = ZMOD4410_RES; index++;
            memcpy(&aTxBuffer[index], &SensorsOutRawData.ZMOD4410outRawData.TVOC, 4); index += 4;
            memcpy(&aTxBuffer[index], &SensorsOutRawData.ZMOD4410outRawData.r_mox, 4); index += 4;
            memcpy(&aTxBuffer[index], &SensorsOutRawData.ZMOD4410outRawData.IAQ, 4); index += 4;
            memcpy(&aTxBuffer[index], &SensorsOutRawData.ZMOD4410outRawData.eCO2, 4); index += 4;
      }
      if (RX_REQ_decode.LED1_control_requested){
        LED_Control_data.LED1_IsChange = 1;
        if (RX_REQ_decode.LED1_isON){
          LED_Control_data.LED1 = true;
        }else{
          LED_Control_data.LED1 = false;
        }
      }
      if (RX_REQ_decode.LED2_control_requested){
        LED_Control_data.LED2_IsChange = 1;
        if (RX_REQ_decode.LED2_isON){
          LED_Control_data.LED2 = true;
        }else{
          LED_Control_data.LED2 = false;
        }
      }
      if (RX_REQ_decode.IAQcolor_control_requested){
        LED_Control_data.IAQcolor_IsChange = 1;
        LED_Control_data.IAQvalue = RX_REQ_decode.IAQ_color_value;
        //LED_Control_data.B4IAQvalue = LED_Control_data.IAQvalue;
        DEBUG_UART("IAQcolor_IsChange %d\r\n", LED_Control_data.IAQvalue);
        //IAQ value -> RGB value;
      }
      if (RX_REQ_decode.IAQindex_control_requested){
        LED_Control_data.IAQindex_IsChange = 1;
        LED_Control_data.IAQindex = RX_REQ_decode.IAQ_index_value;
      }
      if (RX_REQ_decode.LEDAnimation_control_requested){
        LED_Control_data.LED_Animation_IsChange = 1;
        LED_Control_data.LED_Animation = RX_REQ_decode.LEDAnimation;
      }
      if (RX_REQ_decode.BT_Pairing){
        LED_Control_data.BT_Pairing_IsChange = true;

      }
      if (RX_REQ_decode.BT_Pair_OK){
//        LED_Control_data.BT_Pair_OK_IsChange = true;
      }
      if (RX_REQ_decode.Wifi_Connect_OK){
        LED_Control_data.Wifi_Connect_OK_IsChange = true;
      }
      if (RX_REQ_decode.Wifi_Connected_Not_Active){
        LED_Control_data.Wifi_Connected_Not_Active_IsChange = true;
      }
      if (RX_REQ_decode.Wifi_Connected_Active){
        LED_Control_data.Wifi_Connected_Active_IsChange = true;
      }
      if (RX_REQ_decode.Wifi_Disconnected){
        LED_Control_data.Wifi_Disconnected_IsChange = true;
      }
      if (RX_REQ_decode.Manual_on){
//        LED_Control_data.Manual_on_IsChange = true;
      }
      if (RX_REQ_decode.No_WiFi_BT){
        LED_Control_data.No_Wifi_BT_IsChange = true;
      }
      if (RX_REQ_decode.IAQ_Intensity_requested){
        LED_Control_data.IAQ_Intensity_IsChange = true;
        LED_Control_data.IAQ_Intensity_value = RX_REQ_decode.IAQ_Intensity_value;
      }
      
      if (RX_REQ_decode.Btn1_status_cont_req){
        LED_Control_data.Btn1_OnOff = (GPIO_PinState)RX_REQ_decode.Btn1_isON;
        if (LED_Control_data.Btn1_OnOff_B4 != LED_Control_data.Btn1_OnOff){
          LED_Control_data.Btn1_OnOff_B4 = LED_Control_data.Btn1_OnOff;
          aTxBuffer[index] = KEY_EVENT; index++; //type
          aTxBuffer[index] = BTN_1_STATUS_RES; index++;
          LED_Control_data.Btn1_status_IsChange = true;
          LED_Control_data.Btn1_OnOff = (GPIO_PinState)RX_REQ_decode.Btn1_isON;
          aTxBuffer[index] = LED_Control_data.Btn1_OnOff; index++;
        }
      }
      
      
      if (RX_REQ_decode.FanCloudState_control_requested ){
        
        if (LED_Control_data.B4_Fan_AlgoApp_State_value != RX_REQ_decode.FanCloudState_Command){
          LED_Control_data.Fan_AlgoApp_State_IsChange = true;
          LED_Control_data.Fan_AlgoApp_State_value = RX_REQ_decode.FanCloudState_Command;
          LED_Control_data.B4_Fan_AlgoApp_State_value = RX_REQ_decode.FanCloudState_Command;
        }else{
          LED_Control_data.Fan_AlgoApp_State_IsChange = false;
        }
        
        
        
        
        
//        aTxBuffer[index] = KEY_EVENT; index++; //type
//        aTxBuffer[index] = BTN_1_STATUS_RES; index++;
//        LED_Control_data.Btn1_status_IsChange = 1;
//        if (RX_REQ_decode.Btn1_isON || RX_REQ_decode.FanCloudState_Command == 1){
//          LED_Control_data.Btn1_OnOff = GPIO_PIN_SET;
//          aTxBuffer[index] = 1; index++;
//        }else{
//          LED_Control_data.Btn1_OnOff = GPIO_PIN_RESET;
//          aTxBuffer[index] = 0; index++;
//        }
      }
      if (RX_REQ_decode.Btn2_status_cont_req){
        LED_Control_data.Btn2_OnOff = (GPIO_PinState)RX_REQ_decode.Btn2_isON;
        if (LED_Control_data.Btn2_OnOff_B4 != LED_Control_data.Btn2_OnOff){
          LED_Control_data.Btn2_OnOff_B4 = LED_Control_data.Btn2_OnOff;
          aTxBuffer[index] = KEY_EVENT; index++; //type
          aTxBuffer[index] = BTN_2_STATUS_RES; index++;
          LED_Control_data.Btn2_status_IsChange = true;
          LED_Control_data.Btn2_OnOff = (GPIO_PinState)RX_REQ_decode.Btn2_isON;
          aTxBuffer[index] = LED_Control_data.Btn2_OnOff; index++;
        }
//        if (RX_REQ_decode.Btn2_isON || RX_REQ_decode.FanCloudState_Command == 2){
//          LED_Control_data.Btn2_OnOff = GPIO_PIN_SET;
//          aTxBuffer[index] = 1; index++;
//        }else{
//          LED_Control_data.Btn2_OnOff = GPIO_PIN_RESET;
//          aTxBuffer[index] = 0; index++;
//        }
      }
      if (RX_REQ_decode.Fan_App_Status_Req){

        if (LED_Control_data.Fan_App_Status_Value_B4 != RX_REQ_decode.Fan_App_Status_Value){
           LED_Control_data.Fan_App_Status_IsChange = true;
           LED_Control_data.Fan_App_Status_Value = RX_REQ_decode.Fan_App_Status_Value;
              LED_Control_data.Fan_App_Status_Value_B4 = LED_Control_data.Fan_App_Status_Value;
        }
              
      }
//      if (RX_REQ_decode.Triac1_control_requested){
//        aTxBuffer[index] = POWER_CONTROL_RES; index++; //type
//        aTxBuffer[index] = TRIAC_1_STATUS_RES; index++;
//        LED_Control_data.Triac1_IsChange = 1;
//        LED_Control_data.Triac1_value = RX_REQ_decode.Triac1_value;
//        aTxBuffer[index] = RX_REQ_decode.Triac1_value;
//      }
//      if (RX_REQ_decode.Triac2_control_requested){
//        aTxBuffer[index] = POWER_CONTROL_RES; index++; //type
//        aTxBuffer[index] = TRIAC_2_STATUS_RES; index++;
//        LED_Control_data.Triac2_IsChange = 1;
//        LED_Control_data.Triac2_value = RX_REQ_decode.Triac2_value;
//        aTxBuffer[index] = RX_REQ_decode.Triac2_value;
//      }
      if (RX_REQ_decode.Version_requested){
        aTxBuffer[index] = SPEC_TYPE_RES; index++; //type
        aTxBuffer[index] = VER_RES_STM_TO_ESP; index++;
        aTxBuffer[index] = STM_VER_MAIN; index++;
        aTxBuffer[index] = STM_VER_MINOR; index++;
        aTxBuffer[index] = STM_VER_SUB; index++;
        if (APP_Vect_offset == 0U){
          aTxBuffer[index] = STM_START_ADD_0X0; index++;
        }else if (APP_Vect_offset == 0x8000U) {
          aTxBuffer[index] = STM_START_ADD_0X8000; index++;
        }
//        aTxBuffer[index] = iaq_2nd_gen_ver.major; index++;
//        aTxBuffer[index] = iaq_2nd_gen_ver.minor; index++;
//        aTxBuffer[index] = iaq_2nd_gen_ver.patch; index++;
      }
      if (RX_REQ_decode.Model_requested){
        aTxBuffer[index] = SPEC_TYPE_RES; index++; //type
        aTxBuffer[index] = MODEL_RES_STM_TO_ESP; index++;
        aTxBuffer[index] = STM_MODEL_NO; index++;                 //model MSB
        aTxBuffer[index] = 00; index++;       //model LSB
      }

      if (RX_REQ_decode.Switch_config_change_req){
          
          Switch_config_setting = RX_REQ_decode.Switch_config;
          if (Switch_config_setting_B4 != Switch_config_setting){
            
            LED_Control_data.SwtichConfig_IsChange = true;
            Switch_config_setting_B4 = Switch_config_setting;
            DEBUG_UART("UART get Switch_config_setting = %d\r\n", Switch_config_setting);
          }else{
            DEBUG_UART("UART get same Switch_config_setting = %d\r\n", Switch_config_setting);
          }
      }
      if (RX_REQ_decode.EngCo2FrcTurn_change_req){
        SCD30_FRC_Value = RX_REQ_decode.Eng_Co2FrcValue;
        DEBUG_UART("Get SCD30_FRC_Value (ppm value) = %d\r\n", SCD30_FRC_Value);
      }
      if (RX_REQ_decode.BleEnableStatus_change_req){
        //BLE_Enable_status = RX_REQ_decode.BleEnableStatus_Value;
        LED_Control_data.Get_BLE_Enable_status_Value =  RX_REQ_decode.BleEnableStatus_Value;
        LED_Control_data.BLE_Enable_status_ = true;
        DEBUG_UART("____________Get BLE_Enable_status change = %d\r\n", LED_Control_data.Get_BLE_Enable_status_Value);
        //HAL_Delay(50);
      }
      if (RX_REQ_decode.Send_Sign_to_ESP_req){
        //LED_Control_data.Send_Sign_to_ESP = true;
        aTxBuffer[index] = SPEC_TYPE_RES; index++; //type
        aTxBuffer[index] = SIGN_REQ_STM_TO_ESP; index++;
        
        GetSTM_SignData(&aTxBuffer[index]); index += 64;
      }
      
        aTxBuffer[UART_DATA_LEN_POS] = index;
        aTxBuffer[index] = sensirion_common_generate_crc(aTxBuffer, index); index++;     //CRC
        
        if (aTxBuffer[UART_DATA_LEN_POS] > 2){
          UART_Transfering = 1;
          if (HAL_UART_Transmit_IT(&huart1, (uint8_t *)aTxBuffer, index) != HAL_OK)
          {
            DEBUG_UART("UART1 Send error \r\n");
          }else{
            DEBUG_UART("1. TX>> ");
            for (i = 0; i < index; i++){
              DEBUG_UART("%X ", aTxBuffer[i]);
            }
            DEBUG_UART("\r\n");
            
            //DEBUG_UART("UART1 Send OK \r\n");
          }
          UART_Transfering = 0;
          while (Uart1Ready != SET)
          {

          }

          Uart1Ready = RESET;     
      }

      memset(aRxBuffer, 0, sizeof(aRxBuffer));
      memset(localRxBuffer, 0, sizeof(localRxBuffer));
    }
    
    

    
    
    if (ESP_Cmd_Send_En == 1 || LED_Control_data.FanRealState_IsChange || LED_Control_data.Manual_on_IsChange || LED_Control_data.Btn1_RealState_isChange || LED_Control_data.Btn2_RealState_isChange || LED_Control_data.BT_Pairing_IsChange || LED_Control_data.SysReset_IsChange || just_StartUP_canSend == true){
      
      index = 0;
      memset(aTxBuffer, 0, sizeof(aTxBuffer));
      aTxBuffer[index] = START_BYTE; index++;
      aTxBuffer[index] = SCD30_RES_DATA_LEN; index++;
      
      if (LED_Control_data.BT_Pairing_IsChange){
        aTxBuffer[index] = KEY_EVENT; index++; //type
        aTxBuffer[index] = KEY_BLE_PAIR; index++;
        aTxBuffer[index] = LED_Control_data.BT_Pairing_Value; index++;
        LED_Control_data.BT_Pairing_IsChange = false;
//        DEBUG_UART("________BT_Pairing_IsChange ....\r\n");
        //UartTransRepeat = 2;
      }
      
      else if (LED_Control_data.Btn1_RealState_isChange){
        aTxBuffer[index] = KEY_EVENT; index++; //type
        aTxBuffer[index] = BTN_1_STATUS_RES; index++;
        if (LED_Control_data.Btn1_OnOff){
          aTxBuffer[index] = 1; index++;
        }else{
          aTxBuffer[index] = 0; index++;
        }
        LED_Control_data.Btn1_RealState_isChange = 0;
//        DEBUG_UART("__________Btn1_RealState_isChange = true\r\n");
      }
      else if (LED_Control_data.Btn2_RealState_isChange){
//        DEBUG_UART("__________Btn2_RealState_isChange = true\r\n");
        aTxBuffer[index] = KEY_EVENT; index++; //type
        aTxBuffer[index] = BTN_2_STATUS_RES; index++;
        if (LED_Control_data.Btn2_OnOff){
          aTxBuffer[index] = 1; index++;
        }else{
          aTxBuffer[index] = 0; index++;
        }
        LED_Control_data.Btn2_RealState_isChange = 0;
      }
      else if (LED_Control_data.Manual_on_IsChange){
        aTxBuffer[index] = SPEC_TYPE_RES; index++; //type
        aTxBuffer[index] = M_OVERRIDE; index++;
        if (LED_Control_data.Manual_on_OnOff){
          aTxBuffer[index] = 1; index++;
        }else{
          aTxBuffer[index] = 0; index++;
        }
        LED_Control_data.Manual_on_IsChange = 0;
//        DEBUG_UART("__________ Manual_on_IsChange = true\r\n");
      }
      else if (LED_Control_data.FanRealState_IsChange){
        aTxBuffer[index] = POWER_CONTROL; index++; 
        aTxBuffer[index] = FAN_REAL_STATUS_RES; index++; 
        aTxBuffer[index] = LED_Control_data.Fan_App_Status_Value; index++;
        LED_Control_data.FanRealState_IsChange = 0;
//        DEBUG_UART("________ FanRealState_IsChange....\r\n");
      }
       
      else if (LED_Control_data.SysReset_IsChange){
        aTxBuffer[index] = KEY_EVENT; index++; //type
        aTxBuffer[index] = KEY_SYS_RESET; index++;
        aTxBuffer[index] = 1; index++;
        LED_Control_data.SysReset_IsChange = 0;
        
//        DEBUG_UART("________SysReset_IsChange....\r\n");
      }else if (just_StartUP_tick >= 10 && just_StartUP_canSend == true){
        aTxBuffer[index] = SPEC_TYPE_RES; index++; //type
        aTxBuffer[index] = STM32_JUST_START; index++;
        aTxBuffer[index] = 1; index++;                 //model MSB
        just_StartUP_canSend = false;
        just_StartUP_tick = 0;
        DEBUG_UART("Sent STM32_JUST_START CMD\r\n");
      }
      
        aTxBuffer[UART_DATA_LEN_POS] = index;
        aTxBuffer[index] = sensirion_common_generate_crc(aTxBuffer, index); index++;     //CRC
        
        if (aTxBuffer[UART_DATA_LEN_POS] > 2){
          UART_Transfering = 1;
          if (HAL_UART_Transmit_IT(&huart1, (uint8_t *)aTxBuffer, index) != HAL_OK)
          {
            DEBUG_UART("UART1 Send error \r\n");
          }else{
            DEBUG_UART("2.TX>> ");
            for (i = 0; i < index; i++){
              DEBUG_UART("%X ", aTxBuffer[i]);
            }
            DEBUG_UART("\r\n");
            
  //          DEBUG_UART("UART1 Send OK \r\n");
          }
          UART_Transfering = 0;
          while (Uart1Ready != SET)
          {

          }
          Uart1Ready = RESET;
        }

       ESP_Cmd_Send_En = 0;

    }
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
    uint8_t ret = HAL_OK;
    msg++;
    if( msg == aRxBuffer + ESP_STM_UART_BUF_SIZE)
    {
        msg = aRxBuffer;
    }
    do  
    {  
        ret = HAL_UART_Receive_IT(UartHandle,(uint8_t *)msg,1);            
    }while(ret != HAL_OK);
    
    if (UART_RX_Cnt > 0){
      if (UART_RX_Cnt == 1){
        UART_RX_Len = *(msg - 1);
      }
      UART_RX_Cnt++;
      if (UART_RX_Cnt == UART_RX_Len + 1){
        UART_RX_Cnt = 0;
        Uart1Ready = SET;
      }
    }
    if(*(msg-1) == START_BYTE){
      UART_RX_Cnt++;
    }
    
  
  
  
  //Uart1Ready = SET;

}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  Uart1Ready = SET;
  
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  
    DEBUG_DRIVER("UART Comm error: error code = %d\r\n", UartHandle->ErrorCode);
    HAL_UART_Abort_IT(&huart1);
      memset(aRxBuffer, 0, sizeof(aRxBuffer));
      UART_RX_Cnt = 0;
    HAL_Delay(100);
    HAL_UART_Receive_IT(&huart1, (uint8_t *)msg, 1);
    Uart1Ready = SET;
}

void Reinit_UART1(void){
  
  HAL_UART_Abort(&huart1);
  HAL_UART_DeInit(&huart1);
      memset(aRxBuffer, 0, sizeof(aRxBuffer));
      UART_RX_Cnt = 0;
  HAL_Delay(5);
  HAL_UART_Init(&huart1);
  __HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);
  HAL_Delay(5);
  HAL_UART_Receive_IT(&huart1, (uint8_t *)msg, 1);
  Uart1Ready = SET;
  DEBUG_UART("UART1 Reinit!\r\n");
}
