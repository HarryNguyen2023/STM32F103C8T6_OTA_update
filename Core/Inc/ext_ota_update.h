#ifndef EXT_OTA_UPDATE_H
#define EXT_OTA_UPDATE_H

#include "main.h"

// Define some special message
#define EXT_OTA_SOF 	0xAA	// Start of frame
#define EXT_OTA_EOF 	0xBB	// End of frame
#define EXT_OTA_ACK 	0x00	// ACK
#define EXT_OTA_NACK	0x01	// NACK

// Application Flash start address
#define EXT_APP_START_ADD	0x08004400

// Some data characteristics
#define EXT_OTA_DATA_MAX_SIZE	1024
#define EXT_OTA_DATA_OVERHEAD	9
#define EXT_OTA_PACKET_MAX_SIZE	(EXT_OTA_DATA_MAX_SIZE + EXT_OTA_DATA_OVERHEAD)

// Exception code
typedef enum
{
	EXT_OTA_EX_OK,
	EXT_OTA_EX_ERR,
}EXT_OTA_EX;

// State of the OTA process
typedef enum
{
	EXT_OTA_STATE_IDLE,
	EXT_OTA_STATE_START,
	EXT_OTA_STATE_HEADER,
	EXT_OTA_STATE_DATA,
	EXT_OTA_STATE_END,
}EXT_OTA_STATE;

// Packet type
typedef enum
{
	EXT_OTA_PACKET_TYPE_CMD,
	EXT_OTA_PACKET_TYPE_DATA,
	EXT_OTA_PACKET_TYPE_HEADER,
	EXT_OTA_PACKET_TYPE_RESPONSE,
}EXT_OTA_PACKET_TYPE;

// OTA commands
typedef enum
{
	EXT_OTA_CMD_START,
	EXT_OTA_CMD_END,
	EXT_OTA_CMD_ABORT,
}EXT_OTA_CMD;

// OTA meta info
typedef struct
{
	uint32_t packet_size;
	uint32_t packet_crc;
	uint32_t reserved_1;
	uint32_t reserved_2;
}__attribute__((packed)) meta_info;

/*
 * OTA Command format
 *
 * ________________________________________
 * |     | Packet |     |     |     |     |
 * | SOF | Type   | Len | CMD | CRC | EOF |
 * |_____|________|_____|_____|_____|_____|
 *   1B      1B     2B    1B     4B    1B
 */
typedef struct
{
  uint8_t   sof;
  uint8_t   packet_type;
  uint16_t  data_len;
  uint8_t   cmd;
  uint32_t  crc;
  uint8_t   eof;
}__attribute__((packed)) EXT_OTA_COMMAND;

/*
 * OTA Header format
 *
 * __________________________________________
 * |     | Packet |     | Header |     |     |
 * | SOF | Type   | Len |  Data  | CRC | EOF |
 * |_____|________|_____|________|_____|_____|
 *   1B      1B     2B     16B     4B    1B
 */
typedef struct
{
  uint8_t     sof;
  uint8_t     packet_type;
  uint16_t    data_len;
  meta_info   meta_data;
  uint32_t    crc;
  uint8_t     eof;
}__attribute__((packed)) EXT_OTA_HEADER;

/*
 * OTA Data format
 *
 * __________________________________________
 * |     | Packet |     |        |     |     |
 * | SOF | Type   | Len |  Data  | CRC | EOF |
 * |_____|________|_____|________|_____|_____|
 *   1B      1B     2B    nBytes   4B    1B
 */
typedef struct
{
  uint8_t     sof;
  uint8_t     packet_type;
  uint16_t    data_len;
  uint8_t     *data;
}__attribute__((packed)) EXT_OTA_DATA;

/*
 * OTA Response format
 *
 * __________________________________________
 * |     | Packet |     |        |     |     |
 * | SOF | Type   | Len | Status | CRC | EOF |
 * |_____|________|_____|________|_____|_____|
 *   1B      1B     2B      1B     4B    1B
 */
typedef struct
{
  uint8_t   sof;
  uint8_t   packet_type;
  uint16_t  data_len;
  uint8_t   status;
  uint32_t  crc;
  uint8_t   eof;
}__attribute__((packed)) EXT_OTA_RESP;

// Function prototypes
EXT_OTA_EX EXT_OTA_Update(void);

#endif
