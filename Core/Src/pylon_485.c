/*
 * pylon_485.c
 *
 *  Created on: Nov 11, 2022
 *      Author: maxx
 */
#include "pylon_485.h"


uint16_t Pylon_485_Calculate_CRC(uint8_t *data, uint16_t frame_size) {

	if (frame_size < 10) {
		return 0;
	}
	uint16_t crc = 0;

	for (int i=1; i< frame_size - 5; i++) {
		crc += data[i];
	}
	crc = 65536 - crc;
	return crc;
}

bool Pylon_485_Check_CRC(uint8_t *data, uint16_t frame_size) {
	uint16_t calculated_crc = Pylon_485_Calculate_CRC(data, frame_size);
	char hexstring[5];
	hexstring[0] = data[frame_size-5];
	hexstring[1] = data[frame_size-4];
	hexstring[2] = data[frame_size-3];
	hexstring[3] = data[frame_size-2];
	uint16_t received_crc  = (uint16_t)strtol(hexstring, NULL, 16);
	return calculated_crc == received_crc;
}

struct pylon_rs485_frame Pylon_485_decode_frame(uint8_t *data, uint16_t frame_size) {
	struct pylon_rs485_frame decoded_frame;
	uint8_t raw_frame[frame_size/2];
	int count = 0;
	raw_frame[count++] = data[0];
	char hexstring[3];
	hexstring[2] = 0;
	for (int i=1; i<= ((frame_size-1)/2); i++) {
		hexstring[0] = data[i*2-1];
		hexstring[1] = data[i*2];
		raw_frame[count++] = (uint8_t)strtol(hexstring, NULL, 16);
	}
	raw_frame[count++] = data[frame_size-1];
	decoded_frame.ver = raw_frame[1];
	decoded_frame.adr = raw_frame[2];
	decoded_frame.cid1 = raw_frame[3];
	decoded_frame.cid2 = raw_frame[4];
	decoded_frame.len = (raw_frame[5] << 8) + raw_frame[6];
	count = 7;

	if (decoded_frame.len != 0) {
		uint16_t lenid = decoded_frame.len & 0x0fff;
		//uint16_t lenid_crc_received = (decoded_frame.len & 0xf000) >> 12;
		//uint16_t lenid_crc_calculated = 16 - (decoded_frame.len & 0xf) + ((decoded_frame.len >> 4) & 0xf) + ((decoded_frame.len >> 8) & 0xf);
		decoded_frame.len = lenid;
		decoded_frame.info = raw_frame[count++];
	}
	decoded_frame.crc = (raw_frame[count] << 8) + raw_frame[count+1];

	return decoded_frame;
}

void pylon_rs485_process_request(UART_HandleTypeDef uart, struct pylon_rs485_frame *frame) {
	if (COMMAND_GET_PROTOCOL_VERSION == frame->cid2) {
		return pylon_rs485_process_protocol_version_request(uart, frame);
	}
}

void pylon_rs485_process_protocol_version_request(UART_HandleTypeDef uart, struct pylon_rs485_frame *frame) {
	pylon_rs485_send_frame(uart, 0, 0);
}

uint8_t pylon_rs485_encode_frame(UART_HandleTypeDef uart, uint8_t *frame, uint16_t frame_size) {
	uint8_t count = 0;
	uint8_t raw_frame[frame_size*2];
	raw_frame[count++] = frame[0];
	char hex_string[5];
	hex_string[3] = 0;
	for (int i=1; i<frame_size-1; i++) {
		sprintf(hex_string, "%02x", frame[i]);
		raw_frame[count++] = hex_string[0];
		raw_frame[count++] = hex_string[1];
	}
	raw_frame[count] = frame[frame_size-1];

	uint16_t crc =  Pylon_485_Calculate_CRC(raw_frame, count+1);
	hex_string[3] = 0;
	sprintf(hex_string, "%04x", crc);
	raw_frame[count-4] = hex_string[0];
	raw_frame[count-3] = hex_string[1];
	raw_frame[count-2] = hex_string[2];
	raw_frame[count-1] = hex_string[3];

	HAL_UART_Transmit(&uart, raw_frame, count+1, 1000);
}

void pylon_rs485_send_frame(UART_HandleTypeDef uart, uint16_t len, uint8_t info) {
	uint8_t* frame = (uint8_t*)malloc(32 * sizeof(uint8_t));
	frame[0] = 0x7E;
	frame[1] = 0x20;
	frame[2] = 0x00;
	frame[3] = 0x46; // CID1
	frame[4] = 0x00; // CID2
	frame[5] = 0x00; // LENGHT
	frame[6] = 0x00; // LENGHT
	frame[7] = 0x00; // CRC
	frame[8] = 0x00; // CRC
	frame[9] = 0x0D; // EOI

	return pylon_rs485_encode_frame(uart, frame, 10);
}
