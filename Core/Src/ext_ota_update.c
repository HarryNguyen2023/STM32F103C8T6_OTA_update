/*
 * ext_ota_update.c
 *
 *  Created on: Aug 19, 2024
 *      Author: 84935
 */

#include "ext_ota_update.h"
#include "main.h"

#include <stdio.h>
#include <string.h>

// Receive buffer
static uint8_t rcv_buffer[EXT_OTA_PACKET_MAX_SIZE];

// OTA state
static EXT_OTA_STATE ota_state = EXT_OTA_STATE_IDLE;

// Update firmware total size
static uint32_t ota_fw_total_size;
// Update firmware image 's CRC32
static uint32_t ota_fw_crc;
// Firmware size that we have received
static uint32_t ota_fw_received_size;
// Slot number to write to the received firmware
static uint8_t slot_num_to_write_fw;
// Configuration
EXT_GNRL_CONFIG *cfg_flash = (EXT_GNRL_CONFIG*) (EXT_CONFIG_FLASH_ADD);

/********************************* Private Functions Prototypes *****************************************/

static uint16_t EXT_OTA_Receive_Chunk(uint8_t* buffer, uint16_t max_len);
static EXT_OTA_EX EXT_OTA_Process_Data(uint8_t* buffer, uint16_t len);
static void EXT_OTA_Send_Resp(uint8_t resp_type);
static HAL_StatusTypeDef EXT_OTA_Slot_Data_Write(uint8_t* data, uint8_t slot_num, uint16_t data_len, uint8_t is_first_block);
static uint8_t EXT_OTA_Get_Available_Slot_Number(void);
static HAL_StatusTypeDef EXT_OTA_Write_Config(EXT_GNRL_CONFIG* cfg);
static HAL_StatusTypeDef EXT_OTA_App_Data_Write(uint8_t* data, uint32_t data_len);

/******************************** Private Functions Code ***********************************************/

/*
 * @brief Receive a chunk of data
 * @param buffer: buffer to store the received data
 * @param max_len: maximum length of data to be received
 * @retval uint16_t
 */
static uint16_t EXT_OTA_Receive_Chunk(uint8_t* buffer, uint16_t max_len)
{
	HAL_StatusTypeDef ret;
	uint16_t idx = 0u;
	uint16_t data_len;
	uint32_t cal_data_crc = 0u;
	uint32_t rec_data_crc = 0u;

	do
	{
		// Receive the SOF byte
		ret = HAL_UART_Receive(&huart1, &buffer[idx], 1, HAL_MAX_DELAY);
		if(ret != HAL_OK)
		{
			break;
		}
		// Check if the received byte is the SOF
		if(buffer[idx++] != EXT_OTA_SOF)
		{
			ret = EXT_OTA_EX_ERR;
			break;
		}
		// Receive the packet type
		ret = HAL_UART_Receive(&huart1, &buffer[idx++], 1, HAL_MAX_DELAY);
		if(ret != HAL_OK)
		{
			break;
		}
		// Get the data length of the packet
		ret = HAL_UART_Receive(&huart1, &buffer[idx], 2, HAL_MAX_DELAY);
		if(ret != HAL_OK)
		{
			break;
		}
		data_len = *(uint16_t *)&buffer[idx];
		idx += 2;

		// Receive the data
		for(uint16_t i = 0; i < data_len; ++i)
		{
			ret = HAL_UART_Receive(&huart1, &buffer[idx++], 1, HAL_MAX_DELAY);
			if(ret != HAL_OK)
			{
				break;
			}
		}

		// Get the CRC of the data packet
		ret = HAL_UART_Receive( &huart1, &buffer[idx], 4, HAL_MAX_DELAY);
		if( ret != HAL_OK )
		{
		  break;
		}
		rec_data_crc = *(uint32_t*)&buffer[idx];
		idx += 4;
		// Receive EOF byte
		ret = HAL_UART_Receive(&huart1, &buffer[idx], 1, HAL_MAX_DELAY);
		if(ret != HAL_OK)
		{
			break;
		}
		// Check if the received byte is the SOF
		if(buffer[idx++] != EXT_OTA_EOF)
		{
			ret = EXT_OTA_EX_ERR;
			break;
		}
		// Validate the CRC
		cal_data_crc = CalcCRC((uint8_t*)&buffer[4], data_len);
		if(rec_data_crc != cal_data_crc)
		{
			printf("CRC mismatch [Cal CRC = 0x%08lX] [Rec CRC = 0x%08lX]\r\n", cal_data_crc, rec_data_crc);
			ret = EXT_OTA_EX_ERR;
			break;
		}
	}
	while(0);

	// Check for error
	if(ret != HAL_OK)
	{
		printf("Received error!\r\n");
		idx = 0;
	}

	if(max_len < idx)
	{
		printf("Received more data than expected. Expected = %d, Received = %d\r\n", max_len, idx);
		idx = 0;
	}
	return idx;
}

/*
 * @brief Process the data received
 * param buffer: the received buffer
 * param len: the data len to be received
 * retval ETX_OTA_EX
 */
static EXT_OTA_EX EXT_OTA_Process_Data(uint8_t* buffer, uint16_t len)
{
	EXT_OTA_EX ret = EXT_OTA_EX_ERR;

	do
	{
		// Check the receive buffer
		if(buffer == NULL || len == 0)
			break;
		// Check if we receive OTA Abort command
		EXT_OTA_COMMAND* cmd = (EXT_OTA_COMMAND*)&buffer;
		if(cmd->packet_type == EXT_OTA_PACKET_TYPE_CMD)
		{
			if(cmd->cmd == EXT_OTA_CMD_ABORT)
			{
				break;
			}
		}

		switch(ota_state)
		{
		case EXT_OTA_STATE_IDLE:
		{
			printf("EXT_OTA_STATE_IDLE...\r\n");
			ret = EXT_OTA_EX_OK;
		}
			break;

		case EXT_OTA_STATE_START:
		{
			EXT_OTA_COMMAND* cmd = (EXT_OTA_COMMAND*)buffer;
			if(cmd->packet_type == EXT_OTA_PACKET_TYPE_CMD)
			{
				if(cmd->cmd == EXT_OTA_CMD_START)
				{
					printf("Received OTA START command\r\n");
					ota_state = EXT_OTA_STATE_HEADER;
					ret = EXT_OTA_EX_OK;
				}
			}
		}
			break;

		case EXT_OTA_STATE_HEADER:
		{
			EXT_OTA_HEADER* header = (EXT_OTA_HEADER*)buffer;
			if(header->packet_type == EXT_OTA_PACKET_TYPE_HEADER)
			{
				ota_fw_total_size = header->meta_data.packet_size;
				ota_fw_crc = header->meta_data.packet_crc;
				printf("Received OTA Header. FW Size = %lu\r\n", ota_fw_total_size);
				// Get the slot number to write
				slot_num_to_write_fw = EXT_OTA_Get_Available_Slot_Number();
				if(slot_num_to_write_fw != 0xFF)
				{
					ota_state = EXT_OTA_STATE_DATA;
					ret = EXT_OTA_EX_OK;
				}
			}
		}
			break;

		case EXT_OTA_STATE_DATA:
		{
			EXT_OTA_DATA* data = (EXT_OTA_DATA*)buffer;
			uint16_t data_len = data->data_len;
			HAL_StatusTypeDef ex;

			if(data->packet_type == EXT_OTA_PACKET_TYPE_DATA)
			{
				uint8_t is_first_block = 0;
				// Check for the first data block
				if(ota_fw_received_size == 0)
				{
					is_first_block = 1;

					// Read the configuration
					EXT_GNRL_CONFIG cfg;
					memcpy(&cfg, cfg_flash, sizeof(EXT_GNRL_CONFIG));

					// Reset the available slot
					cfg.slot_table[slot_num_to_write_fw].is_this_slot_valid = 1;

					// Write the updated configuration to the flash memory
					ret = EXT_OTA_Write_Config(&cfg);
					if(ret != EXT_OTA_EX_OK)
					{
						break;
					}
				}
				// Write received data to the block space
				ex = EXT_OTA_Slot_Data_Write(buffer + 4, slot_num_to_write_fw, data_len, is_first_block);
				if(ex == HAL_OK)
				{
					printf("[%ld/%ld]\r\n", ota_fw_received_size/EXT_OTA_DATA_MAX_SIZE, ota_fw_total_size/EXT_OTA_DATA_MAX_SIZE);
					if(ota_fw_received_size >= ota_fw_total_size)
					{
						ota_state = EXT_OTA_STATE_END;
					}
					ret = EXT_OTA_EX_OK;
				}
			}
		}
			break;

		case EXT_OTA_STATE_END:
		{
			EXT_OTA_COMMAND* cmd = (EXT_OTA_COMMAND*)buffer;
			if(cmd->packet_type == EXT_OTA_PACKET_TYPE_CMD)
			{
				if(cmd->cmd == EXT_OTA_CMD_END)
				{
					printf("Received OTA END command\r\n");

					// Check if the received binary has been modified
					uint32_t slot_address = (slot_num_to_write_fw == 0) ? EXT_APP_SLOT0_FLASH_ADD : EXT_APP_SLOT1_FLASH_ADD;
					// Verify the CRC of the firmware's image
					uint32_t cal_crc = CalcCRC((uint8_t*)slot_address, ota_fw_total_size);
					if(cal_crc != ota_fw_crc)
					{
						printf("Error: CRC mismatch of fw image!\r\n");
						break;
					}

					// Read the configuration
					EXT_GNRL_CONFIG cfg;
					memcpy(&cfg, cfg_flash, sizeof(EXT_GNRL_CONFIG));

					// Update the slot information
					cfg.slot_table[slot_num_to_write_fw].fw_crc 					= cal_crc;
					cfg.slot_table[slot_num_to_write_fw].fw_size 					= ota_fw_total_size;
					cfg.slot_table[slot_num_to_write_fw].is_this_slot_valid 		= 0;
					cfg.slot_table[slot_num_to_write_fw].should_we_run_this_slot_fw = 1;

					// Reset the condition of other slots
					for(uint8_t i = 0; i < EXT_SLOT_NO; ++i)
					{
						if(slot_num_to_write_fw != i)
						{
							cfg.slot_table[i].should_we_run_this_slot_fw = 0;
						}
					}
					// Update the reboot reason
					cfg.reboot_cause = EXT_NORMAL_BOOT;
					// Update the configuration into the Flash memory
					ret = EXT_OTA_Write_Config(&cfg);
					if(ret == EXT_OTA_EX_OK)
					{
						ota_state = EXT_OTA_STATE_IDLE;
						ret = EXT_OTA_EX_OK;
					}
					ret = EXT_OTA_EX_OK;
				}
			}
		}
			break;

		default:
		{
			ret = EXT_OTA_EX_ERR;
		}
			break;
		}
	}
	while(0);

	return ret;
}

/*
 * @brief Send the response from the MCU
 * @param resp_type: ACK or NACK
 * @retval none
 */
static void EXT_OTA_Send_Resp(uint8_t resp_type)
{
	EXT_OTA_RESP rsp =
	{
		.sof 			= EXT_OTA_SOF,
		.packet_type 	= EXT_OTA_PACKET_TYPE_RESPONSE,
		.data_len 		= 1,
		.status 		= resp_type,
		.eof			= EXT_OTA_EOF
	};
	rsp.crc = CalcCRC((uint8_t*)&rsp.status, 1);
	HAL_UART_Transmit(&huart1, (uint8_t*)&rsp, sizeof(EXT_OTA_RESP), 100);
}

/*
 * @brief Write data application to the actual flash memory
 * @param data: data to be written
 * @param data_len: length of the data to be written
 * @param is_first_block: true - if this is the first block
 */
static HAL_StatusTypeDef EXT_OTA_Slot_Data_Write(uint8_t* data, uint8_t slot_num, uint16_t data_len, uint8_t is_first_block)
{
	HAL_StatusTypeDef ret = HAL_OK;
	// Data write sequence
	do
	{
		// Validate the input condition
		if(slot_num >= EXT_SLOT_NO)
		{
			ret = HAL_ERROR;
			break;
		}
		// Unlock flash memory
		ret = HAL_FLASH_Unlock();
		if(ret != HAL_OK)
		{
			printf("Unable to unlock Flash memory, update stopped!");
			break;
		}
		// Erase the flash in the first time
		if(is_first_block)
		{
			printf("Erasing flash memory");

			FLASH_EraseInitTypeDef EraseInitStruct;
			uint32_t sector_error;

			EraseInitStruct.TypeErase 	= FLASH_TYPEERASE_PAGES;
			// Select the slot to erase
			EraseInitStruct.PageAddress = (slot_num == 0) ? EXT_APP_SLOT0_FLASH_ADD : EXT_APP_SLOT1_FLASH_ADD;
			EraseInitStruct.NbPages 	= DATA_FLASH_SIZE;	// 13 KB
			ret = HAL_FLASHEx_Erase(&EraseInitStruct, &sector_error);
			if(ret != HAL_OK)
			{
				printf("Unable to erase Flash memory, updating stopped");
				break;
			}
		}

		uint32_t slot_address = (slot_num == 0) ? EXT_APP_SLOT0_FLASH_ADD : EXT_APP_SLOT1_FLASH_ADD;

		// Write data to the flash memory
		for(uint16_t i = 0; i < data_len / 2; ++i)
		{
			uint16_t halfword_data = data[i * 2] | (data[i * 2 + 1] << 8);
			ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (slot_address + ota_fw_received_size), halfword_data);
			if(ret == HAL_OK)
			{
				ota_fw_received_size += 2;
			}
			else
			{
				printf("Error: Unable to write to Flash, update stopped!");
				break;
			}
		}
		if(ret != HAL_OK)
		{
			break;
		}

		// Lock the Flash memory
		ret = HAL_FLASH_Lock();
		if(ret != HAL_OK)
		{
			printf("Error: Unable to lock Flash, update stopped!");
			break;
		}
	}
	while(0);

	return ret;
}

/*
 * @brief Get the Flash data slot for firmware update
 * @param none
 * @retval uint8_t
 */
static uint8_t EXT_OTA_Get_Available_Slot_Number()
{
	uint8_t data_slot = 0xFFu;

	// Read the configuration
	EXT_GNRL_CONFIG cfg;
	memcpy(&cfg, cfg_flash, sizeof(EXT_GNRL_CONFIG));

	// Check if there is any valid slot
	for(uint8_t i = 0; i < EXT_SLOT_NO; ++i)
	{
		if(cfg.slot_table[i].is_this_slot_valid != 0 || cfg.slot_table[i].is_this_slot_active == 0)
		{
			data_slot = i;
			printf("Find slot %u available for OTA update\r\n", i);
			break;
		}
	}
	return data_slot;
}

/*
 * @brief Write data from the suitable firmware slot to the application memory
 * @param data: Data to be written to the application memory
 * @param data_len: length of the data to be written
 * @retval HAL_StatusTypeDef
 */
static HAL_StatusTypeDef EXT_OTA_App_Data_Write(uint8_t* data, uint32_t data_len)
{
	HAL_StatusTypeDef ret;

	do
	{
		// Erase the Flash memory of the application
		ret = HAL_FLASH_Unlock();
		if(ret != HAL_OK)
			break;

		printf("Erasing application flash memory");

		FLASH_EraseInitTypeDef EraseInitStruct;
		uint32_t sector_error;

		EraseInitStruct.TypeErase 	= FLASH_TYPEERASE_PAGES;
		EraseInitStruct.PageAddress = EXT_APP_START_ADD;
		EraseInitStruct.NbPages 	= DATA_FLASH_SIZE;	// 13 KB
		ret = HAL_FLASHEx_Erase(&EraseInitStruct, &sector_error);
		if(ret != HAL_OK)
		{
			printf("Unable to erase Flash memory, updating stopped");
			break;
		}
		// Program the new application into the Flash memory
		for(uint32_t i = 0; i < data_len / 2; ++i)
		{
			uint16_t halfword_data = data[i * 2] | (data[i * 2 + 1] << 8);
			ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (EXT_APP_START_ADD + (i * 2)), halfword_data);
			if(ret != HAL_OK)
			{
				printf("Error: Unable to write to Flash, update stopped!");
				break;
			}
		}
		if(ret != HAL_OK)
		{
			break;
		}

		// Lock the Flash memory
		ret = HAL_FLASH_Lock();
		if(ret != HAL_OK)
		{
			printf("Error: Unable to lock Flash, update stopped!");
			break;
		}
	}
	while(0);

	return ret;
}

/*
 * @brief Write configuration information into Flash memory
 * @param cfg: current configuration
 * @retval HAL_StatusTypeDef
 */
static HAL_StatusTypeDef EXT_OTA_Write_Config(EXT_GNRL_CONFIG* cfg)
{
	HAL_StatusTypeDef ret;

	do
	{
		// Check the input condition
		if(cfg == NULL)
		{
			ret = HAL_ERROR;
			break;
		}
		// Erase the Flash memory of the application
		ret = HAL_FLASH_Unlock();
		if(ret != HAL_OK)
			break;

		printf("Erasing config flash memory");

		FLASH_EraseInitTypeDef EraseInitStruct;
		uint32_t sector_error;

		EraseInitStruct.TypeErase 	= FLASH_TYPEERASE_PAGES;
		EraseInitStruct.PageAddress = EXT_CONFIG_FLASH_ADD;
		EraseInitStruct.NbPages 	= CONFIG_FLASH_SIZE;	// 6 KB
		ret = HAL_FLASHEx_Erase(&EraseInitStruct, &sector_error);
		if(ret != HAL_OK)
		{
			printf("Unable to erase Flash memory, updating stopped");
			break;
		}
		// Program the new application into the Flash memory
		uint8_t* data = (uint8_t*)cfg;
		for(uint32_t i = 0; i < sizeof(EXT_GNRL_CONFIG) / 2; ++i)
		{
			uint16_t halfword_data = data[i * 2] | (data[i * 2 + 1] << 8);
			ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (EXT_CONFIG_FLASH_ADD + (i * 2)), halfword_data);
			if(ret != HAL_OK)
			{
				printf("Error: Unable to write to Flash, update stopped!");
				break;
			}
		}
		if(ret != HAL_OK)
		{
			break;
		}

		// Lock the Flash memory
		ret = HAL_FLASH_Lock();
		if(ret != HAL_OK)
		{
			printf("Error: Unable to lock Flash, update stopped!");
			break;
		}
	}
	while(0);

	return ret;
}

/******************************** General Function Code *****************************/
/*
 * @brief Function to perform the OTA update sequence
 * @param none
 * @retval EXT_OTA_EX
 */
EXT_OTA_EX EXT_OTA_Update(void)
{
	EXT_OTA_EX ret = EXT_OTA_EX_OK;
	uint16_t len = 0;

	printf("Waiting for the OTA firmware\r\n");

	// Reset the variables
	ota_fw_total_size 		= 0u;
	ota_fw_received_size 	= 0u;
	ota_fw_crc				= 0u;
	ota_state				= EXT_OTA_STATE_START;
	slot_num_to_write_fw	= 0xFFu;

	do
	{
		memset(rcv_buffer, 0, EXT_OTA_PACKET_MAX_SIZE);

		len = EXT_OTA_Receive_Chunk(rcv_buffer, EXT_OTA_PACKET_MAX_SIZE);

		if(len != 0)
		{
			ret = EXT_OTA_Process_Data(rcv_buffer, len);
		}
		else
		{
			ret = EXT_OTA_EX_ERR;
		}

		if(ret == EXT_OTA_EX_OK)
		{
			printf("Sending ACK\r\n");
			EXT_OTA_Send_Resp(EXT_OTA_ACK);
		}
		else
		{
			printf("Sending NACK\r\n");
			EXT_OTA_Send_Resp(EXT_OTA_NACK);
			break;
		}
	}
	while(ota_state != EXT_OTA_STATE_IDLE);

	return ret;
}

/*
 * @brief Funciton to load the suitable firmware into the application slot
 * @param none
 * @retval none
 */
void EXT_OTA_Load_New_App()
{
	uint8_t is_update_available = 0;
	uint8_t slot_num;
	HAL_StatusTypeDef ret;

	// Read the configuration
	EXT_GNRL_CONFIG cfg;
	memcpy(&cfg, cfg_flash, sizeof(EXT_GNRL_CONFIG));

	// Check if there is a new application
	for(uint8_t i = 0; i < EXT_SLOT_NO; ++i)
	{
		if(cfg.slot_table[i].should_we_run_this_slot_fw == 1)
		{
			printf("New application is available at slot %u\r\n", i);
			is_update_available = 1;
			slot_num = i;

			// Update the slot information
			cfg.slot_table[i].is_this_slot_active = 1;
			cfg.slot_table[i].should_we_run_this_slot_fw = 0;

			break;
		}
	}

	// Update the new slot
	if(is_update_available)
	{
		// Deactivate other slots
		for(uint8_t i = 0; i < EXT_SLOT_NO; ++i)
		{
			if(slot_num != i)
			{
				cfg.slot_table[i].is_this_slot_active = 0;
			}
		}

		uint32_t slot_address = (slot_num == 0) ? EXT_APP_SLOT0_FLASH_ADD : EXT_APP_SLOT1_FLASH_ADD;

		ret = EXT_OTA_App_Data_Write((uint8_t*)slot_address, cfg.slot_table[slot_num].fw_size);
		if(ret != HAL_OK)
		{
			printf("Error: Unable to update the new app!\r\n");
		}
		else
		{
			ret = EXT_OTA_Write_Config(&cfg);
			if(ret != HAL_OK)
			{
				printf("Error: Unable to write config Flash\r\n");
			}
		}
	}
	else
	{
		// Find the slot that is active in case update is not available
		for(uint8_t i = 0; i < EXT_SLOT_NO; ++i)
		{
			if(cfg.slot_table[i].is_this_slot_active == 1)
			{
				slot_num = i;
				break;
			}
		}
		// Verify if the application is corrupted or intervened
		printf("Verifying the new application...\r\n");

		uint32_t cal_crc = CalcCRC((uint8_t*)EXT_APP_START_ADD, cfg.slot_table[slot_num].fw_size);

		// Verify the CRC of the firmware image
		if(cal_crc != cfg.slot_table[slot_num].fw_crc)
		{
			printf("Error: invalid application!\r\n");
			while(1);
		}
		else
		{
			printf("Done uploading new application\r\n");
		}
	}
}

uint32_t CalcCRC(uint8_t * pData, uint32_t DataLength)
{
    uint32_t Checksum = 0xFFFFFFFF;
    for(unsigned int i=0; i < DataLength; i++)
    {
        uint8_t top = (uint8_t)(Checksum >> 24);
        top ^= pData[i];
        Checksum = (Checksum << 8) ^ crc_table[top];
    }
    return Checksum;
}
