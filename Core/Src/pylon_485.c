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
	} else if (COMMAND_GET_MANUFACTURER_INFO == frame->cid2) {
		return pylon_rs485_process_manufacturer_info_request(uart, frame);
	} else if (COMMAND_GET_SYSTEM_PARAMETER == frame->cid2) {
		return pylon_rs485_processget_system_parameters_request(uart, frame);
	}
}

void pylon_rs485_process_protocol_version_request(UART_HandleTypeDef uart, struct pylon_rs485_frame *frame) {
	pylon_rs485_send_frame(uart, 0, 0);
}

void pylon_rs485_process_manufacturer_info_request(UART_HandleTypeDef uart, struct pylon_rs485_frame *frame) {
	char *data = "Pylon  EMU10Glory  to  Ukraine !";
	pylon_rs485_send_frame(uart, 32, data);
}

void pylon_rs485_processget_system_parameters_request(UART_HandleTypeDef uart, struct pylon_rs485_frame *frame) {
/*
	"CellHighVoltageLimit" / ToVolt(construct.Int16ub),
    "CellLowVoltageLimit" / ToVolt(construct.Int16ub),
    "CellUnderVoltageLimit" / ToVolt(construct.Int16sb),
    "ChargeHighTemperatureLimit" / ToCelsius(construct.Int16sb),
    "ChargeLowTemperatureLimit" / ToCelsius(construct.Int16sb),
    "ChargeCurrentLimit" / DivideBy100(construct.Int16sb),
    "ModuleHighVoltageLimit" / ToVolt(construct.Int16ub),
    "ModuleLowVoltageLimit" / ToVolt(construct.Int16ub),
    "ModuleUnderVoltageLimit" / ToVolt(construct.Int16ub),
    "DischargeHighTemperatureLimit" / ToCelsius(construct.Int16sb),
    "DischargeLowTemperatureLimit" / ToCelsius(construct.Int16sb),
    "DischargeCurrentLimit" / DivideBy100(construct.Int16sb),
*/
	uint8_t data[25];

	data[0] = 0X00; // todo CHECK info FLAG
	data[1] = 3650 >> 8;
	data[2] = 3650 & 0xff;
	data[3] = 2500 >> 8;
	data[4] = 2500 & 0xff;
	data[5] = 2000 >> 8;
	data[6] = 2000 & 0xff;
	data[7] = 3582 >> 8;
	data[8] = 3582 & 0xff;
	data[9] = 2735 >> 8;
	data[10] = 2735 & 0xff;
	data[11] = 2000 >> 8;
	data[12] = 2000 & 0xff;
	data[13] = 55000 >> 8;
	data[14] = 55000 & 0xff;
	data[15] = 45000 >> 8;
	data[16] = 45000 & 0xff;
	data[17] = 42000 >> 8;
	data[18] = 42000 & 0xff;
	data[19] = 3582 >> 8;
	data[20] = 3582 & 0xff;
	data[21] = 2735 >> 8;
	data[22] = 2735 & 0xff;
	data[23] = 10000 >> 8;
	data[24] = 10000 & 0xff;
	pylon_rs485_send_frame(uart, 25, data);
}

void pylon_rs485_encode_frame(UART_HandleTypeDef uart, uint8_t *frame, uint16_t frame_size) {
	uint8_t count = 0;
	uint8_t raw_frame[(frame_size-1)*2];
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

uint16_t pylon_rs483_get_lenid(uint16_t size) {
	if (size == 0) {
		return 0;
	}
	uint16_t lencrc = 16 - (size & 0xf) + ((size >> 4) & 0xf) + ((size >> 8) & 0xf);
	return (lencrc << 12) + size;
}

void pylon_rs485_send_frame(UART_HandleTypeDef uart, uint16_t len, uint8_t *info) {
	uint8_t* frame = (uint8_t*)malloc((10 + len) * sizeof(uint8_t));
	frame[0] = 0x7E;
	frame[1] = 0x20;
	frame[2] = 0x00;
	frame[3] = 0x46; // CID1
	frame[4] = 0x00; // CID2
	uint16_t lenid = pylon_rs483_get_lenid(len);
	frame[5] = lenid << 8;
	frame[6] = lenid & 0xff;
	uint8_t count = 7;
	for (int i=0; i<len; i++) {
		frame[count++] = info[i];
	}
	//frame[6] = 0x00; // LENGHT
	frame[count++] = 0x00; // CRC
	frame[count++] = 0x00; // CRC
	frame[count++] = 0x0D; // EOI

	pylon_rs485_encode_frame(uart, frame, count);
	free(frame);
}
