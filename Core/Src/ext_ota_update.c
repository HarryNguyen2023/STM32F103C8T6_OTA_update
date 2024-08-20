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

/********************************* Private Functions Prototypes *****************************************/

static uint16_t EXT_OTA_Receive_Chunk(uint8_t* buffer, uint16_t max_len);
static EXT_OTA_EX EXT_OTA_Process_Data(uint8_t* buffer, uint16_t len);
static void EXT_OTA_Send_Resp(uint8_t resp_type);
static HAL_StatusTypeDef EXT_OTA_Flash_Data_Write(uint8_t* data, uint16_t data_len, uint8_t is_first_block);

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
	uint16_t idx = 0;
	uint16_t data_len;

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
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				ota_fw_total_size = header->meta_data.packet_size;
				ota_fw_crc = header->meta_data.packet_crc;
				printf("Received OTA Header. FW Size = %lu\r\n", ota_fw_total_size);
				ota_state = EXT_OTA_STATE_DATA;
				ret = EXT_OTA_EX_OK;
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
				ex = EXT_OTA_Flash_Data_Write(buffer + 4, data_len, (ota_fw_received_size == 0));
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
					ota_state = EXT_OTA_STATE_IDLE;
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
		.crc			= 0,
		.eof			= EXT_OTA_EOF
	};
	HAL_UART_Transmit(&huart1, (uint8_t*)&rsp, sizeof(EXT_OTA_RESP), 100);
}

/*
 * @brief Write data application to the actual flash memory
 * @param data: data to be written
 * @param data_len: length of the data to be written
 * @param is_first_block: true - if this is the first block
 */
static HAL_StatusTypeDef EXT_OTA_Flash_Data_Write(uint8_t* data, uint16_t data_len, uint8_t is_first_block)
{
	HAL_StatusTypeDef ret = HAL_OK;
	// Data write sequence
	do
	{
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
			EraseInitStruct.PageAddress = EXT_APP_START_ADD;
			EraseInitStruct.NbPages 	= 47;	// 47 KB
			ret = HAL_FLASHEx_Erase(&EraseInitStruct, &sector_error);
			if(ret != HAL_OK)
			{
				printf("Unable to erase Flash memory, updating stopped");
				break;
			}
		}

		// Write data to the flash memory
		for(uint16_t i = 0; i < data_len / 2; ++i)
		{
			uint16_t halfword_data = data[i * 2] | (data[i * 2 + 1] << 8);
			ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (EXT_APP_START_ADD + ota_fw_received_size), halfword_data);
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

/******************************** General Function *****************************/

EXT_OTA_EX EXT_OTA_Update(void)
{
	EXT_OTA_EX ret = EXT_OTA_EX_OK;
	uint16_t len = 0;

	printf("Waiting for the OTA firmware\r\n");

	// Reset the variables
	ota_fw_total_size 		= 0;
	ota_fw_received_size 	= 0;
	ota_fw_crc				= 0;
	ota_state				= EXT_OTA_STATE_START;

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


