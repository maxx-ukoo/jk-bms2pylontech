/*
 * pylon_485.h
 *
 *  Created on: Nov 11, 2022
 *      Author: maxx
 */

#ifndef INC_PYLON_485_H_
#define INC_PYLON_485_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>


#define COMMAND_GET_VALUES				0x42
#define COMMAND_GET_ALARM_INFO			0x44
#define COMMAND_GET_SYSTEM_PARAMETER	0x47
#define COMMAND_GET_PROTOCOL_VERSION	0x4F
#define COMMAND_GET_MANUFACTURER_INFO	0x51
#define COMMAND_GET_MANAGEMENT_INFO		0x92
#define COMMAND_GET_SN_NUMBER			0x93
#define COMMAND_SET_MANAGEMENT_INFO		0x94
#define COMMAND_TURN_OFF				0x95
#define COMMAND_GET_FIRMWARE_INFO		0x96


struct pylon_rs485_frame {
	uint8_t ver;
	uint8_t adr;
	uint8_t cid1;
	uint8_t cid2;
	uint16_t len;
	uint8_t info;
	uint16_t crc;
};

uint16_t Pylon_485_Calculate_CRC(uint8_t *data, uint16_t frame_size);
bool  Pylon_485_Check_CRC(uint8_t *data, uint16_t frame_size);
struct pylon_rs485_frame Pylon_485_decode_frame(uint8_t *data, uint16_t frame_size);

void pylon_rs485_process_request(UART_HandleTypeDef uart, struct pylon_rs485_frame *frame);
void pylon_rs485_process_protocol_version_request(UART_HandleTypeDef uart, struct pylon_rs485_frame *frame);
void pylon_rs485_process_manufacturer_info_request(UART_HandleTypeDef uart, struct pylon_rs485_frame *frame);
void pylon_rs485_processget_system_parameters_request(UART_HandleTypeDef uart, struct pylon_rs485_frame *frame);
void pylon_rs485_send_frame(UART_HandleTypeDef uart, uint16_t len, uint8_t *info);

//uint8_t pylon_rs485_get_responce_protocol_version(struct pylon_rs485_frame *frame, uint8_t *data);
//uint8_t pylon_rs485_encode_frame(uint8_t *frame, uint8_t *raw_frame, uint16_t frame_size);
//uint8_t pylon_rs485_format_frame(uint8_t *data, uint16_t len, uint8_t info);



#endif /* INC_PYLON_485_H_ */
