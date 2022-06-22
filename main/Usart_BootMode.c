#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "Usart_BootMode.h"
#include "driver/gpio.h"
#include "CommonUse.h"
#include "app_storage.h"

#ifdef ALLOW_UPDATE_BOOTLOADER
  #define STM_BOOTLOADER_BIN_INIT
  #include "STM_bootloader_bin.h"
#endif

// extern void Reset_High(void);
// extern void Reset_Low(void);
// extern void Boot0_High(void);
// extern void Boot0_Low(void);
// extern void vTaskDelay(__IO uint32_t nTime);
// extern char binfile[12*1024];
// extern uint64_t binfile_length;
// uint64_t binfile[12*1024];
uint32_t binfile_offset = 0;
uint32_t binfile_length = 0xA2C4;

uint32_t flash_address = 0;
uint64_t total_data_length = 0;
uint16_t trnsfer_length = 0;
uint64_t total_data_transfer_counter = 0;
int FWDL_errcode_info;
uint64_t load_start_address = 0;
uint64_t load_end_address = 0;

uint32_t load_index = 0;

// void EnterBootLoaderMode(void)
// {
  // //boot0 high
  // //reset high->low->high
  // Boot0_High();   	//Boot0: High
  // vTaskDelay(5);  
  // Reset_High();   	//Reset: High
  // vTaskDelay(5);  
  // Reset_Low(); 	        //Reset: Low
  // vTaskDelay(10);  
  // Reset_High();   	//Reset: High
  // vTaskDelay(5);  
  // Reset_High();   	//Reset: High
  // vTaskDelay(5);  
  // Reset_Low(); 	        //Reset: Low
  // vTaskDelay(10);  
  // Reset_High();   	//Reset: High
// }

// void LeaveBootLoaderMode(void)
// {
  // //boot0 low
  // //reset high->low->high
  // Boot0_Low();          //Boot0: Low
  // vTaskDelay(5);  
  // Reset_High();   	//Reset: High
  // vTaskDelay(5);  
  // Reset_Low(); 	        //Reset: Low
  // vTaskDelay(10);  
  // Reset_High();   	//Reset: High
  // vTaskDelay(5);  
  // Reset_High();   	//Reset: High
  // vTaskDelay(5);  
  // Reset_Low(); 	        //Reset: Low
  // vTaskDelay(10);  
  // Reset_High();   	//Reset: High
// }


//------------------------------------------------------------------------------//
int Check_StartBit(void)
{
  uint8_t wbuffer[2] = {0};
  uint8_t rbuffer[2] = {0};  
  FWDL_errcode_info = NACK_ERR;
  
  //write command
  wbuffer[0] = 0x7f;
  Usart2_Write(wbuffer, 1); 
  
  //check ack
  Usart2_Read(rbuffer, 1);
  if(rbuffer[0] != ACK)
    return ACK_ERR;
  
  return 0;
}
//------------------------------------------------------------------------------//
int Get_Version_Command(uint8_t *version, uint8_t *OptionByte)
{
  uint8_t wbuffer[2] = {0};
  uint8_t rbuffer[5] = {0};
    
  FWDL_errcode_info = GetVersion_ERR;
  
  //write command
  wbuffer[0] = GetVersion;
  wbuffer[1] = 0xfe;
  Usart2_Write(wbuffer, 2); 
    
  //check ack
  Usart2_Read(rbuffer, 5);
  if(rbuffer[0] != ACK || rbuffer[4] != ACK)
    return ACK_ERR;
  
  version[0] = rbuffer[1];
  OptionByte[0] = rbuffer[2];
  OptionByte[1] = rbuffer[3];
  
  return 0;
}
//------------------------------------------------------------------------------//
int Get_Command(uint8_t *command)
{
  uint8_t wbuffer[2] = {0};
  uint8_t rbuffer[15] = {0};  
  FWDL_errcode_info = Get_ERR;
 
  //write command
  wbuffer[0] = Get;
  wbuffer[1] = 0xff;
  Usart2_Write(wbuffer, 0x02);
  
  //check ack
  Usart2_Read(rbuffer, 15);
  if(rbuffer[0] != ACK || rbuffer[14] != ACK)
    return ACK_ERR;
  
  for(int i=0; i<15; i++)
    command[i] = rbuffer[i];
  
  return 0;
}  
//------------------------------------------------------------------------------//
int Get_ID_Command(uint8_t *id)
{
  uint8_t wbuffer[2] = {0};
  uint8_t rbuffer[5] = {0};
  int ret = 0;
  
  FWDL_errcode_info = GetID_ERR;
  
  //write command
  wbuffer[0] = GetID;
  wbuffer[1] = 0xfd;
  Usart2_Write(wbuffer, 0x02);
  
  //check ack
  Usart2_Read(rbuffer, 5);
  if(rbuffer[0] != ACK || rbuffer[4] != ACK)
    return ACK_ERR;
  
  id[0] = rbuffer[1];
  id[1] = rbuffer[2];
  id[2] = rbuffer[3];
  
  return ret;
}

//------------------------------------------------------------------------------//
int Go_Command(uint32_t go_address)
{
  uint8_t wbuffer[5] = {0};
  uint8_t rbuffer[3] = {0};
  int ret = 0;
  
  FWDL_errcode_info = Go_ERR;
  
  wbuffer[0] = Go;
  wbuffer[1] = 0xde;
  Usart2_Write(wbuffer, 0x02);

  //check 1st ack
  ret = Usart2_Read(rbuffer, 1);
  if(ret <= 0) {
    return ACK_ERR;
  }
  if(rbuffer[0] != ACK) {
    return ACK_ERR;
  }

  uint32_t addr = 0x08000000;
  if(go_address) {
  	addr = go_address;
  }
  wbuffer[0] = (uint8_t) (addr >> 24);
  wbuffer[1] = (uint8_t) (addr >> 16);
  wbuffer[2] = (uint8_t) (addr >> 8);
  wbuffer[3] = (uint8_t) addr;  
  wbuffer[4] = wbuffer[0] ^wbuffer[1] ^wbuffer[2] ^wbuffer[3];
  Usart2_Write(wbuffer, 0x05);

  //check 2nd ack
  ret = Usart2_Read(rbuffer, 1);
  if(ret <= 0) {
    return ACK_ERR;
  }
  if(rbuffer[0] != ACK) {
    return ACK_ERR;
  }
  return ret;
}

//------------------------------------------------------------------------------//
int Enable_Read_Out_Protect(void)
{
  uint8_t wbuffer[2] = {0};
  uint8_t rbuffer[3] = {0};
  int ret = 0;
  
  FWDL_errcode_info = ReadoutProtect_ERR;
  
  wbuffer[0] = ReadoutProtect;
  wbuffer[1] = 0x7d;
  Usart2_Write(wbuffer, 0x02);

  //check 1st ack
  ret = Usart2_Read(rbuffer, 1);
  if(ret <= 0) {
    return ACK_ERR;
  }
  if(rbuffer[0] != ACK) {
    return ACK_ERR;
  }
  //check 2nd ack, poll 10s
  for(int i=0; i<10; i++) {
    ret = Usart2_Read(rbuffer, 1);
    if(ret > 0) {
      if(rbuffer[0] == ACK)
        return 0;
      else
        return ACK_ERR;
    }
  }
  return ACK_ERR;
}

//------------------------------------------------------------------------------//
int Disable_Read_Out_Protect(void)
{
  uint8_t wbuffer[2] = {0};
  uint8_t rbuffer[3] = {0};
  int ret = 0;
  
  FWDL_errcode_info = ReadoutUnprotect_ERR;
  
  wbuffer[0] = ReadoutUnprotect;
  wbuffer[1] = 0x6d;
  Usart2_Write(wbuffer, 0x02);

  //check 1st ack
  ret = Usart2_Read(rbuffer, 1);
  if(ret <= 0) {
    return ACK_ERR;
  }
  if(rbuffer[0] != ACK) {
    return ACK_ERR;
  }
  //check 2nd ack, poll 10s
  for(int i=0; i<10; i++) {
    ret = Usart2_Read(rbuffer, 1);
    if(ret > 0) {
      if(rbuffer[0] == ACK)
        return 0;
      else
        return ACK_ERR;
    }
  }
  return ACK_ERR;
}

//------------------------------------------------------------------------------//
int Erase_All_Flash(void)
{
  uint8_t wbuffer[3] = {0};
  uint8_t rbuffer[3] = {0};
      
  FWDL_errcode_info = Erase_ERR;
  
  //write command
  wbuffer[0] = Erase;
  wbuffer[1] = 0xbb;
  Usart2_Write(wbuffer, 0x02);
  
  //check ack
  Usart2_Read(rbuffer, 1);
  if(rbuffer[0] != ACK)
    return ACK_ERR;
  
  //write command
  wbuffer[0] = 0xff;
  wbuffer[1] = 0xff;
  wbuffer[2] = wbuffer[0] ^ wbuffer[1];
  Usart2_Write(wbuffer, 0x03);
  
#if 1
  //poll 10s
  for(int i=0; i<10; i++) {
    int ret = Usart2_Read(rbuffer, 1);
    if(ret > 0) {
      if(rbuffer[0] == ACK)
        return 0;
      else
        return ACK_ERR;
    }
  }
  return ACK_ERR;
#else
  vTaskDelay(8000);
  
  //check ack
  Usart2_Read(rbuffer, 1);
  if(rbuffer[0] != ACK)
    return ACK_ERR;
  
  return 0;
#endif
}
//------------------------------------------------------------------------------//
//stm32f401 flash module organization
//block         name            blcok base addresses            size
//main memory   sector0         0x0800 0000 - 0x0800 3fff       16kbytes
//              sector1         0x0800 4000 - 0x0800 7fff       16kbytes
//              sector2         0x0800 8000 - 0x0800 bfff       16kbytes
//              sector3         0x0800 c000 - 0x0800 ffff       16kbytes
//              sector4         0x0801 0000 - 0x0801 ffff       64kbytes
//              sector5         0x0802 0000 - 0x0803 ffff       128kbytes
//		-----------------------------------------	stm32f401xx(flash:256KB)
//              sector6         0x0804 0000 - 0x0805 ffff       128kbytes
//              sector7         0x0806 0000 - 0x0807 ffff       128kbytes
//		-----------------------------------------	stm32f411xx(flash:512KB)

int FWDL_Erase_Flash(uint16_t mcu_name, uint8_t total_erase_sector, uint8_t *erase_sector_buffer)
{
  uint8_t wbuffer[20] = {0};
  uint8_t rbuffer[3] = {0};
  uint8_t XOR = 0;
      
  FWDL_errcode_info = Erase_ERR;
    
  if(mcu_name == 401){
    if(total_erase_sector > 6)
      return ACK_ERR;
  }else if(mcu_name == 411){
    if(total_erase_sector > 8)
      return ACK_ERR;
  }else
      return ACK_ERR;
    
  //write command
  wbuffer[0] = Erase;
  wbuffer[1] = 0xbb;
  Usart2_Write(wbuffer, 0x02);
  
  //check ack
  Usart2_Read(rbuffer, 1);
  if(rbuffer[0] != ACK)
    return ACK_ERR;
     
  wbuffer[0] = 0x00;
  XOR = XOR ^ wbuffer[0];
  wbuffer[1] = total_erase_sector-1;
  XOR = XOR ^ wbuffer[1];
  
  for(int i=0; i<total_erase_sector; i++)
  {
    wbuffer[(2*i)+2] = 0x00;
    XOR = XOR ^ wbuffer[(2*i)+2];
    wbuffer[(2*i)+3] = erase_sector_buffer[i];
    XOR = XOR ^ wbuffer[(2*i)+3];  
  }
  
  wbuffer[(2*total_erase_sector)+2+1-1] = XOR;  
  Usart2_Write(wbuffer, (2*total_erase_sector)+2+1);
  
  vTaskDelay(8000);
  
  //check ack
  Usart2_Read(rbuffer, 1);
  if(rbuffer[0] != ACK)
    return ACK_ERR;
  
  return 0;
}

//------------------------------------------------------------------------------//
int Read_Flash(uint32_t addr,uint8_t *buf, uint16_t len)
{
  uint8_t wbuffer[3] = {0};
  uint8_t rbuffer[3] = {0};
  uint8_t data[5] = {0};
  //uint8_t XOR = 0;
  
  FWDL_errcode_info = ReadMemory_ERR;
  
  //Read command
  wbuffer[0] = ReadMemory;
  wbuffer[1] = 0xee;
  Usart2_Write(wbuffer, 0x02);
  
  Usart2_Read(rbuffer, 1);
  if(rbuffer[0] != ACK)
    return ACK_ERR;

  //write start_address & checksum
  data[0] = (uint8_t) (addr >> 24);
  data[1] = (uint8_t) (addr >> 16);
  data[2] = (uint8_t) (addr >> 8);
  data[3] = (uint8_t) addr;  
  data[4] = data[0] ^data[1] ^data[2] ^data[3];
  Usart2_Write(data, 0x05);  
  
  Usart2_Read(rbuffer, 1);
  if(rbuffer[0] != ACK)
    return ACK_ERR;
  
  wbuffer[0] = len-1;
  wbuffer[1] = wbuffer[0] ^ 0xFF;
  Usart2_Write(wbuffer, 0x02);
  
  Usart2_Read(rbuffer, 1);
  if(rbuffer[0] != ACK)
    return ACK_ERR;

  uint8_t *tmp_buf = malloc(len);
  if (tmp_buf) {
    Usart2_Read(tmp_buf, len);
    memcpy(buf, tmp_buf, len);
  }
  free(tmp_buf);

  return 0;
}
//------------------------------------------------------------------------------//

//------------------------------------------------------------------------------//
int Write_Flash(uint32_t addr,uint8_t *buf, uint16_t len)
{
  uint8_t wbuffer[3] = {0};
  uint8_t rbuffer[3] = {0};
  uint8_t data[260] = {0};
  uint8_t XOR = 0;
  
  FWDL_errcode_info = WriteMemory_ERR;
    
  //write command
  wbuffer[0] = WriteMemory;
  wbuffer[1] = 0xce;
  Usart2_Write(wbuffer, 0x02);
  
//  vTaskDelay(5);
  
  //check ack
  Usart2_Read(rbuffer, 1);
  if(rbuffer[0] != ACK)
    return ACK_ERR;
  
//  vTaskDelay(5);
  
  //write start_address & checksum
  data[0] = (uint8_t) (addr >> 24);
  data[1] = (uint8_t) (addr >> 16);
  data[2] = (uint8_t) (addr >> 8);
  data[3] = (uint8_t) addr;  
  data[4] = data[0] ^data[1] ^data[2] ^data[3];
  Usart2_Write(data, 0x05);  
  
//  vTaskDelay(5);
  
  //check ack
  Usart2_Read(rbuffer, 1);
  if(rbuffer[0] != ACK)
    return ACK_ERR;
  
//  vTaskDelay(5);
  
  //prepare data
  data[0] = len-1;                         
  XOR = len-1;
  for (int i = 0; i < len; i++) {
	data[i+1] = buf[i]; 
	XOR ^= buf[i];   
  }
  data[len+1] = XOR;
  //write data
  Usart2_Write(data, (len+2));  
  
//  vTaskDelay(5);
    
  //check ack
  Usart2_Read(rbuffer, 1);
  if(rbuffer[0] != ACK)
    return ACK_ERR;  
  
  return 0;
}

bool BootLoader_GoCmd(void)
{
  int ret = 0;
  uint8_t version[1] = {0};
  uint8_t OptionByte[2] = {0};

  for(uint8_t i=0; i<3; i++)
  {
    ret = Check_StartBit();
    if(ret < 0)
    {
      printf("Not in bootloader\r\n");
      continue;
    }

    ret = Get_Version_Command(version, OptionByte);
    if(ret < 0)
    {
      continue;
    }
    printf("bootloader version: 0x%02x (0x%02x 0x%02x)\r\n", version[0], OptionByte[0], OptionByte[1]);
    if(OptionByte[0] == CT_BOOTLOADER_SPEC_CHAR_1 && OptionByte[1] == CT_BOOTLOADER_SPEC_CHAR_2)
    {
      STM_CT_BOOTLOADER_VER_bValid = true;
      STM_CT_BOOTLOADER_VER = version[0];
      ret = Go_Command(STM_FW_GO_ADDR);
      if(ret >= 0)
      {
        printf("CT bootloader GO\r\n");
        return true;
      }
    }
    else
    {
      //STM32 org bootloader, try GO command anyway
      ret = Go_Command(0x08000000);
      if(ret >= 0)
      {
        printf("org bootloader GO\r\n");
        return true;
      }
    }
    vTaskDelay(50/portTICK_RATE_MS);
  }
  printf("bootloader fails to go\r\n");
  return false;
}

//------------------------------------------------------------------------------//
int BootLoader_Test_Flow(void)
{
  int ret = 0; 
  uint8_t version[1] = {0};
  uint8_t command[15] = {0};
  uint8_t id[3] = {0};
  uint8_t OptionByte[2] = {0};  
  FWDL_errcode_info = 0;
  uint8_t flash_temp[256] = {0};
  uint8_t Read_flash_temp[256] = {0};
  bool bUpdateBootloader = STM_bPrepareUpdateBootloader;
  // EnterBootLoaderMode();  
  vTaskDelay(100);
  
  ret = Check_StartBit();
  if(ret < 0)
    return FWDL_errcode_info;
  printf("Check_StartBit OK\r\n");
  vTaskDelay(10);
  
  ret = Get_Version_Command(version, OptionByte);
  if(ret < 0)
    return FWDL_errcode_info;
  printf("Get_Version_Command OK\r\n");
#ifdef USE_STM32_CT_BOOTLOADER
  if(OptionByte[0] == CT_BOOTLOADER_SPEC_CHAR_1 && OptionByte[1] == CT_BOOTLOADER_SPEC_CHAR_2)
  {
    printf("CT 2nd bootloader version: 0x%02x\r\n", version[0]);
  }
  else
  {
    printf("STM32 org bootloader version: 0x%02x\r\n", version[0]);
  }
#endif
  vTaskDelay(10);
  
  ret = Get_Command(command);
  if(ret < 0)
    return FWDL_errcode_info;
  printf("Get_Command OK\r\n");
  vTaskDelay(10);
  
  ret = Get_ID_Command(id);
  if(ret < 0)
    return FWDL_errcode_info;
  printf("Get_ID_Command OK\r\n");
  vTaskDelay(10);
  
#if 1//#ifdef BROAN_SECURE_RELEASE
  ret = Disable_Read_Out_Protect();
  if(ret < 0)
    return FWDL_errcode_info;
  //wait for system reset
  vTaskDelay(1000/portTICK_RATE_MS);
  Check_StartBit();
  printf("Disable_Read_Out_Protect OK\r\n");
  vTaskDelay(10);
#endif

  ret = Erase_All_Flash();
  if(ret < 0)
    return FWDL_errcode_info;
  printf("Erase_All_Flash OK\r\n");
  vTaskDelay(10);
//  uint8_t erase_sector_buffer[7] = {0,1,2,4,5,6,7};
//  FWDL_Erase_Flash(411, 7, erase_sector_buffer);
//  if(ret < 0)
//    return FWDL_errcode_info;
//  printf("FWDL_Erase_Flash OK\r\n");
//  vTaskDelay(10);
   
#ifdef ALLOW_UPDATE_BOOTLOADER
  if(bUpdateBootloader)
  {
    flash_address = 0x08000000;
    total_data_length = sizeof(_STM_bootloader_bin);
    total_data_transfer_counter = total_data_length / 128;
    if(total_data_length % 128 > 0)
      total_data_transfer_counter++;

    load_index = 0;
    for(int i=0; i<total_data_transfer_counter; i++)
    {
      if(total_data_length >= 128)
        trnsfer_length = 128;
      else
      	trnsfer_length = total_data_length;
      load_start_address = load_index * 128;
      load_end_address = load_start_address + trnsfer_length;
      memcpy(flash_temp, _STM_bootloader_bin+load_start_address, trnsfer_length);
      
      ret = Write_Flash(flash_address, flash_temp, trnsfer_length);
      if(ret < 0) {
        return FWDL_errcode_info;
      }
      printf("Write_Flash OK, i = %d, from flash_address = 0x%X\r\n", i, flash_address);

      load_index = load_index + 1;
      flash_address = flash_address + trnsfer_length;
      total_data_length = total_data_length - trnsfer_length;
      if(total_data_length == 0)
      {
          break;
      }

      vTaskDelay(10/portTICK_RATE_MS);
    }

    flash_address = 0x08000000;
    total_data_length = sizeof(_STM_bootloader_bin);
    total_data_transfer_counter = total_data_length / 128;
    if(total_data_length % 128 > 0)
      total_data_transfer_counter++;

    load_index = 0;
    for(int i=0; i<total_data_transfer_counter; i++)
    {
      if(total_data_length >= 128)
        trnsfer_length = 128;
      else
        trnsfer_length = total_data_length;
      load_start_address = load_index * 128;
      load_end_address = load_start_address + trnsfer_length;

      memset(Read_flash_temp, 0, sizeof(Read_flash_temp));
      ret = Read_Flash(flash_address, Read_flash_temp, trnsfer_length);
      if(ret < 0) {
        return FWDL_errcode_info;
      }
      printf("Read OK, i = %d, from flash_address = 0x%X\r\n", i, flash_address);

      memset(flash_temp, 0, sizeof(flash_temp));
      memcpy(flash_temp, _STM_bootloader_bin+load_start_address, trnsfer_length);
      if (memcmp(flash_temp, Read_flash_temp, trnsfer_length)){	
        printf("Read Flash compare fail, i = %d, from flash_address = 0x%X\r\n", i, flash_address);
        return FWDL_errcode_info;
      }

      load_index = load_index + 1;
      flash_address = flash_address + trnsfer_length;
      total_data_length = total_data_length - trnsfer_length;
      if(total_data_length == 0)
      {
          break;
      }
      
      vTaskDelay(10/portTICK_RATE_MS);
    }
  }
#endif

  flash_address = 0x08000000;                 
#ifdef USE_STM32_CT_BOOTLOADER
  if(STM_CT_BOOTLOADER_VER_bValid)
  {
    flash_address = STM_FW_GO_ADDR;
  }
#endif
#ifdef ALLOW_UPDATE_BOOTLOADER
  if(bUpdateBootloader)
  {
    flash_address = STM_FW_GO_ADDR;
  }
#endif
  total_data_length = binfile_length;
  total_data_transfer_counter = total_data_length / 128;
  if(total_data_length % 128 > 0)
    total_data_transfer_counter++;
  
  load_index = 0;
  //printf("total_data_transfer_counter = %lld \r\n", total_data_transfer_counter);
  
    const esp_partition_t *partition = esp_partition_find_first(64, ESP_PARTITION_SUBTYPE_ANY, "stm_fw_0");
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
	p += binfile_offset;
  
  for(int i=0; i<total_data_transfer_counter; i++)
  {
    if(total_data_length >= 128)
      trnsfer_length = 128;
    else
      trnsfer_length = total_data_length;
    
    load_start_address = load_index * 128;
    load_end_address = load_start_address + trnsfer_length;

    // for(int j=0; j<trnsfer_length; j++)
      // flash_temp[j] = binfile[load_start_address + j];
	
	memcpy(flash_temp, p+load_start_address, trnsfer_length);
    
    ret = Write_Flash(flash_address, flash_temp, trnsfer_length);
    if(ret < 0) {
      spi_flash_munmap(map_handle);
      return FWDL_errcode_info;
    }
	printf("Write_Flash OK, i = %d, from flash_address = 0x%X\r\n", i, flash_address);
    
    load_index = load_index + 1;
    flash_address = flash_address + trnsfer_length;
    //if(flash_address == 0x0800c000){              //check sector3 or not, because sector3 used to save calibration data, so can not write code.
    //  flash_address = 0x08010000;                 //jump to sector4
    //  load_index = load_index + (16*1024/128);    //index = index + 128(jump to sector4)
    //}
    total_data_length = total_data_length - trnsfer_length;
    if(total_data_length == 0)
    {
        break;
    }
    
    vTaskDelay(10/portTICK_RATE_MS);
  }
  
  
  
  flash_address = 0x08000000;                 
#ifdef USE_STM32_CT_BOOTLOADER
  if(STM_CT_BOOTLOADER_VER_bValid)
  {
    flash_address = STM_FW_GO_ADDR;
  }
#endif
#ifdef ALLOW_UPDATE_BOOTLOADER
  if(bUpdateBootloader)
  {
    flash_address = STM_FW_GO_ADDR;
  }
#endif
  total_data_length = binfile_length;
  total_data_transfer_counter = total_data_length / 128;
  if(total_data_length % 128 > 0)
    total_data_transfer_counter++;
  
  load_index = 0;
	for(int i=0; i<total_data_transfer_counter; i++)
	{
		if(total_data_length >= 128)
		  trnsfer_length = 128;
		else
		  trnsfer_length = total_data_length;
	  
		load_start_address = load_index * 128;
		load_end_address = load_start_address + trnsfer_length;
	  
		memset(Read_flash_temp, 0, sizeof(Read_flash_temp));
		ret = Read_Flash(flash_address, Read_flash_temp, trnsfer_length);
		if(ret < 0) {
		  spi_flash_munmap(map_handle);
		  return FWDL_errcode_info;
		}
		printf("Read OK, i = %d, from flash_address = 0x%X\r\n", i, flash_address);
		
		memset(flash_temp, 0, sizeof(flash_temp));
		memcpy(flash_temp, p+load_start_address, trnsfer_length);
		if (memcmp(flash_temp, Read_flash_temp, trnsfer_length)){	
			printf("Read Flash compare fail, i = %d, from flash_address = 0x%X\r\n", i, flash_address);
			return FWDL_errcode_info;
		}
		
		
		load_index = load_index + 1;
		flash_address = flash_address + trnsfer_length;
		//if(flash_address == 0x0800c000){              //check sector3 or not, because sector3 used to save calibration data, so can not write code.
		//  flash_address = 0x08010000;                 //jump to sector4
		//  load_index = load_index + (16*1024/128);    //index = index + 128(jump to sector4)
		//}
		total_data_length = total_data_length - trnsfer_length;
		if(total_data_length == 0)
		{
			break;
		}

		vTaskDelay(10/portTICK_RATE_MS);
	}

#ifdef USE_STM32_CT_BOOTLOADER
  flash_address = STM_FW_SIZE_ADDR;
  ret = Write_Flash(flash_address, (uint8_t *)&binfile_length, sizeof(binfile_length));
  if(ret < 0) {
  	printf("Write FW size OK\r\n");
  } else {
    printf("Write FW size FAIL\r\n");
  }
#endif

#ifdef BROAN_SECURE_RELEASE
  if (!STM_FW_SIGN_bOTAValid)
  {
    memset(STM_FW_SIGN_ARR, 0x28, sizeof(STM_FW_SIGN_ARR));
  }
  flash_address = STM_FW_SIGN_DATA_ADDR;
  //write STM_FW_SIGN_ARR
  ret = Write_Flash(flash_address, STM_FW_SIGN_ARR, sizeof(STM_FW_SIGN_ARR));
  if(ret < 0) {
    spi_flash_munmap(map_handle);
    return FWDL_errcode_info;
  }
  printf("Write STM FW sign OK, from flash_address = 0x%X\r\n", flash_address);

  //read STM_FW_SIGN_ARR
  memset(Read_flash_temp, 0, sizeof(Read_flash_temp));
  ret = Read_Flash(flash_address, Read_flash_temp, sizeof(STM_FW_SIGN_ARR));
  if(ret < 0) {
    spi_flash_munmap(map_handle);
    return FWDL_errcode_info;
  }
  if (memcmp(Read_flash_temp, STM_FW_SIGN_ARR, sizeof(STM_FW_SIGN_ARR)) == 0)
  {
    printf("Read STM FW sign data OK\r\n");
  }
  else
  {
    printf("Read STM FW sign data FAIL\r\n");
  }

////read out protection is done by STM32 itself
//  ret = Enable_Read_Out_Protect();
//  if(ret < 0) {
//    spi_flash_munmap(map_handle);
//    return FWDL_errcode_info;
//  }
//  //wait for system reset
//  vTaskDelay(1000/portTICK_RATE_MS);
//  Check_StartBit();
#endif
   
  // LeaveBootLoaderMode();
  
  /* Unmap the partition to data memory */
  spi_flash_munmap(map_handle);
  return ret;
}

void StartSTM_SetFWParameter(uint32_t offset, uint32_t size)
{
	binfile_offset = offset;
	binfile_length = size;
}

int StartSTM_FWup(void)
{
	int res;
			printf("STM go Bootloader mode....\r\n");
#ifdef USE_STM32_CT_BOOTLOADER
			if (STM_CT_BOOTLOADER_VER_bValid) {
				//don't need to switch to org bootloader
			} else {
				gpio_set_level(STM_BOOT0_PIN, 1);
			}
#else
			gpio_set_level(STM_BOOT0_PIN, 1);
#endif
			vTaskDelay(100/portTICK_RATE_MS);
			gpio_set_level(STM_RST_PIN, STM_RST_RESET);
			vTaskDelay(200/portTICK_RATE_MS);
			gpio_set_level(STM_RST_PIN, STM_RST_SET);
			vTaskDelay(1000/portTICK_RATE_MS);
			printf("Start Firmware write....\r\n");
			storage_WriteSTM32OTAFlag(WRITING_STM_FLAG_UP);
			uart_flushData();
			res = BootLoader_Test_Flow();
			if (res < 0){
				printf("Firmware write error, FWDL_FWDL_errcode_info: %d\r\n", res);
			}else{
				printf("Firmware write finsished, res: %d\r\n", res);
			}
			uart_flushData();
			storage_WriteSTM32OTAFlag(WRITE_STM_FLAG_DOWN);

#ifdef USE_STM32_CT_BOOTLOADER
			vTaskDelay(100/portTICK_RATE_MS);
			printf("STM reset 1\r\n");
			if (STM_CT_BOOTLOADER_VER_bValid) {
				Go_Command(STM_FW_GO_ADDR);
			} else {
				gpio_set_level(STM_BOOT0_PIN, 0);
				gpio_set_level(STM_RST_PIN, STM_RST_RESET);
				vTaskDelay(200/portTICK_RATE_MS);
				gpio_set_level(STM_RST_PIN, STM_RST_SET);
			}

			printf("STM reset 2\r\n");
			//If STM32 enables RDP and then goes to standby mode, ESP32 need to reset it explicitly
			vTaskDelay(1000/portTICK_RATE_MS);
			gpio_set_level(STM_RST_PIN, STM_RST_RESET);
			vTaskDelay(200/portTICK_RATE_MS);
			gpio_set_level(STM_RST_PIN, STM_RST_SET);

			vTaskDelay(200/portTICK_RATE_MS);
			BootLoader_GoCmd();
			vTaskDelay(200/portTICK_RATE_MS);
#else
			gpio_set_level(STM_BOOT0_PIN, 0);
			vTaskDelay(100/portTICK_RATE_MS);
			gpio_set_level(STM_RST_PIN, STM_RST_RESET);
			vTaskDelay(200/portTICK_RATE_MS);
			gpio_set_level(STM_RST_PIN, STM_RST_SET);
			
			//If STM32 enables RDP and then goes to standby mode, ESP32 need to reset it explicitly
			vTaskDelay(1000/portTICK_RATE_MS);
			gpio_set_level(STM_RST_PIN, STM_RST_RESET);
			vTaskDelay(200/portTICK_RATE_MS);
			gpio_set_level(STM_RST_PIN, STM_RST_SET);
#endif
		return res;
}

void STM_Boot2OrgBootLoader(void)
{
	if (!STM_CT_BOOTLOADER_VER_bValid) {
		printf("No STM 2nd bootloader\r\n");
		return;
	}

	vTaskDelay(1000/portTICK_RATE_MS);
	STM_bootloader_test = 1;
	switchAppCmd |= (1UL << SW_APPCMD_STM_BOOTLOADER_ONOFF);
	if (switchAppCmd & (1UL << SW_APPCMD_STM_BOOTLOADER_ONOFF)) {
		vTaskDelay(100/portTICK_RATE_MS);
	}
	vTaskDelay(200/portTICK_RATE_MS);

	//suspend some tasks
	Suspend_SensorTask = 1;
	Suspend_Uart = 1;
	vTaskDelay(1000/portTICK_RATE_MS);

	//set to bootloader
	gpio_set_level(STM_BOOT0_PIN, 1);
	vTaskDelay(100/portTICK_RATE_MS);
	gpio_set_level(STM_RST_PIN, STM_RST_RESET);
	vTaskDelay(200/portTICK_RATE_MS);
	gpio_set_level(STM_RST_PIN, STM_RST_SET);
	vTaskDelay(500/portTICK_RATE_MS);
	STM_CT_BOOTLOADER_VER_bValid = false;
	printf("Boot to org bootloader\r\n");
	//only power-cycle the device to resume normal operation  
}
