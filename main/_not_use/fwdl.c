/*
******************************************************************************
* File Name          : fwdl.c
* Description        : This file provides code for FW update for BRNC011/021
* author  YC Ng, Computime
* date    2019-11-19
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2019 Computime
* All rights reserved.</center></h2>
*
******************************************************************************
*/

//#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "uart_os.h"

#define STM_FWDL_CMD_START                 0x79
#define STM_FWDL_CMD_GET                   0x00
#define STM_FWDL_CMD_GET_VER               0x01
#define STM_FWDL_CMD_GET_ID                0x02
#define STM_FWDL_CMD_READ                  0x11
#define STM_FWDL_CMD_GO                    0x21
#define STM_FWDL_CMD_WRITE                 0x31
#define STM_FWDL_CMD_ERASE                 0x43
#define STM_FWDL_CMD_EXTENDED_ERASE        0x44
#define STM_FWDL_CMD_WRITE_PROTECT         0x63
#define STM_FWDL_CMD_WRITE_UNPROTECT       0x73
#define STM_FWDL_CMD_READOUT_PROTECT       0x82
#define STM_FWDL_CMD_READOUT_UNPROTECT     0x92
#define STM_FWDL_ACK      0x7F
#define STM_FWDL_NACK     0x1F
#define DEV_ID       0x0460

#define block_size 256
#define page_size 2048
#define write_size block_size
#define adr_size 4

#define stm_adr 0x08000000              //STM32 
#define fw_adr 0x210000  //!!!! please define fw storage location in ESP32 // #define fw_adr 0x08010000               //fw storage location in ESP32
#define fw_size 0xA2C4                  //should get it in header of fw file in real application 
#define stm_adr_last stm_adr+fw_size-1    //0x08019900 //0x08017c00


static uint16_t max_page = fw_size/page_size;
static uint32_t stm_write_address = 0;
static uint8_t Tx_buf[1+block_size+1];
static uint8_t Rx_buf[block_size*2];
uint8_t fw_pt = 0;


uint8_t STM_FWDL_FW_Write(void);
uint8_t	STM_FWDL_UART_Tx(uint8_t* p, uint16_t size);
void STM_FWDL_memcpy_rev(uint8_t *restrict dst, uint8_t *restrict src, uint8_t n);
uint8_t STM_FWDL_send_cmd(uint8_t cmd);

//extern uint16_t *p16_t1;  //test

uint8_t STM_FWDL_FW_Write(void)
{
//Start  

  Tx_buf[0]=STM_FWDL_CMD_START;
  if(STM_FWDL_NACK==STM_FWDL_UART_Tx(&Tx_buf[0],1))				
          return 1;
//read Device ID
    if(1==STM_FWDL_send_cmd(STM_FWDL_CMD_GET_ID))
      return 1;
    //wait until 2 byte received
    //.....
    //.....
    static uint16_t tmp16;
	static uint16_t tmp16R;
    tmp16 =  DEV_ID; 
    tmp16R =  DEV_ID+0; 

    memcpy((uint8_t*)&Rx_buf[1], (uint8_t*)&tmp16, 2);
    memcpy((uint8_t*)&Rx_buf[3], (uint8_t*)&tmp16R, 2);
    STM_FWDL_memcpy_rev((uint8_t*)&Rx_buf[5], (uint8_t*)&tmp16, 2);
    STM_FWDL_memcpy_rev((uint8_t*)&Rx_buf[7], (uint8_t*)&tmp16R, 2);

    
    STM_FWDL_memcpy_rev((uint8_t*)&tmp16, (uint8_t*)&Rx_buf[1], 2);
//    if(tmp16!=DEV_ID)
//      return 1;
    
//1. erase page
    if(1==STM_FWDL_send_cmd(STM_FWDL_CMD_EXTENDED_ERASE))
      return 1;
    STM_FWDL_memcpy_rev((uint8_t*)&Tx_buf, (uint8_t*)&max_page, 2);
    uint16_t i;

    for(i=0;i<=max_page;i++){
      STM_FWDL_memcpy_rev((uint8_t*)&Tx_buf[i*sizeof(i)+2], (uint8_t*)&i, 2);
    }

    if(STM_FWDL_NACK==STM_FWDL_UART_Tx(&Tx_buf[0],2+(i*2))){				
      return 1;
    }    

//2. start write    
    stm_write_address=stm_adr;
	printf("2. start write\r\n");
	
	
    const esp_partition_t *partition = esp_partition_find_first(64, ESP_PARTITION_SUBTYPE_ANY, "stm_fw_bin");
    if (!partition) 
	{
        //ESP_LOGE(UDATA_TAG, "No stm_fw_bin partition");
        return -1;
    }
	
    const void* mapP;
    spi_flash_mmap_handle_t map_handle;

    /* Map the partition to data memory */
    if (esp_partition_mmap(partition, 0, partition->size, SPI_FLASH_MMAP_DATA, &mapP, &map_handle) != ESP_OK) 
	{
        //ESP_LOGE(UDATA_TAG, "Read_StmFwBin_Data partition mmap failed");
        return -1;
    }
	
    const uint8_t * p = (const uint8_t *) mapP;
	
	
	
	
	
	
	//fw_pt = fw_adr;
    while(1){
//send write address
		
        STM_FWDL_memcpy_rev( (uint8_t*)&Tx_buf, (uint8_t*)&stm_write_address, adr_size);
        int tmp=0;
        for(int i=0;i<adr_size;i++){
                tmp=tmp^Tx_buf[i];
        }
        Tx_buf[adr_size]=tmp;
        if(STM_FWDL_NACK==STM_FWDL_UART_Tx(&Tx_buf[0],adr_size+1)){				
                return 1;
        }
//send block
        memset(Tx_buf,0xFF,sizeof(Tx_buf));
        Tx_buf[0]=block_size-1;
        memcpy( &Tx_buf[1], p, block_size);
		//printf("flagflag3\r\n");
        int chksum = 0;
        for(int i=0;i<block_size;i++){
                chksum=chksum^Tx_buf[i];
        }
        int chksum_idx=block_size+1;
        Tx_buf[chksum_idx]=chksum;
        if(STM_FWDL_NACK==STM_FWDL_UART_Tx(&Tx_buf[0],sizeof(Tx_buf))){				
                return 1;
        }
        stm_write_address=stm_write_address+write_size;
        if(stm_write_address>stm_adr_last){
                return 0;
        }
        //fw_pt=fw_pt+write_size;
		p += write_size;
    }
}		
		
uint8_t	STM_FWDL_UART_Tx(uint8_t* p, uint16_t size){
   //add ESP32 UartTx here
   //and get get ACK/NACK
  Usart2_Write(p, size);
	printf("\n\rTx[");
	for(int i=0;i<size;i++){
		printf("%02x ",*p);
                p++;
	}
	printf("]\n\r");
	return STM_FWDL_ACK;
}

void STM_FWDL_memcpy_rev(uint8_t *restrict dst, uint8_t *restrict src, uint8_t n)
{
    uint8_t i;

    for (i=0; i < n; ++i)
        dst[n-1-i] = src[i];

}

uint8_t STM_FWDL_send_cmd(uint8_t cmd){
  Tx_buf[0]=cmd;
  Tx_buf[1]=Tx_buf[0]^0xFF;
  if(STM_FWDL_NACK==STM_FWDL_UART_Tx(&Tx_buf[0],2)){				
          return 1;
  }
  return 0;
}
