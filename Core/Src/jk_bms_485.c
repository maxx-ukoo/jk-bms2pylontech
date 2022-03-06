/*
 * jk_bms.c
 *
 *  Created on: Mar 4, 2022
 *      Author: maxx
 */

#include "jk_bms_485.h"

uint8_t request_Status_Frame[] = {0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x29};

uint16_t getCurrent(const uint16_t value) {
	// TODO test current data on real BMS
	if ((value & 0x8000) == 0x8000) {
		return (value & 0x7FFF);
	} else {
		return (value & 0x7FFF) * -1;
	}
}

int8_t getTemperature(const int16_t value) {
   if (value > 100)
     return (100 - (int16_t) value);

   return value;
};

void Request_JK_Battery_485_Status_Frame(UART_HandleTypeDef uart) {
	HAL_UART_Transmit(&uart, request_Status_Frame, 21, 1000);
}

bool JK_Battery_485_Check_Frame_CRC(uint8_t *data, uint16_t frame_size) {
	if (frame_size < 4) {
		return false;
	}
	uint16_t data_len = (uint16_t)data[2] << 8 | data[2 + 1];
	uint16_t computed_crc = 0;
	for (uint16_t i = 0; i < data_len-2; i++) {
		computed_crc = computed_crc + data[i];
	}
	uint16_t remote_crc = (uint16_t)data[data_len] << 8 | data[data_len + 1];
	if (computed_crc == remote_crc) {
		return true;
	}
	return false;
}

void Parse_JK_Battery_485_Status_Frame(uint8_t *data) {
	  uint8_t cells = data[1] / 3;
	  jk_bms_battery_info.cells_number = cells;
	  for (uint8_t i = 0; i < cells; i++) {
		  jk_bms_battery_info.cells_voltage[i].cell_number = data[3*i + 2];
		  jk_bms_battery_info.cells_voltage[i].cell_voltage = (uint16_t) data[3*i + 3] << 8 | data[3*i + 4];
	  }
	  cells++;
	  uint16_t pos = (cells - 1) * 3 + 2; // 50

	  // 0x80 0x00 0x1D: Read power tube temperature                 29°C                      1.0 °C
	   // --->  99 = 99°C, 100 = 100°C, 101 = -1°C, 140 = -40°C

	  // 0x80 0x00 0x1D: Read power tube temperature                 29°C                      1.0 °C
	  //51 52
	  jk_bms_battery_info.battery_status.power_tube_temperature = getTemperature((uint16_t) data[pos + 1] << 8 | data[pos + 2]);
	  pos += 3; // 53
	  //54 55
	  jk_bms_battery_info.battery_status.sensor_temperature_1 = getTemperature((uint16_t) data[pos + 1] << 8 | data[pos + 2]);
	  pos += 3; // 56
	  //57 58
	  jk_bms_battery_info.battery_status.sensor_temperature_2 = getTemperature((uint16_t) data[pos + 1] << 8 | data[pos + 2]);

	  pos += 3; // 59


	  // 0x83 0x14 0xEF: Total battery voltage                       5359 * 0.01 = 53.59V      0.01 V
	  // 60 61
	  jk_bms_battery_info.battery_status.battery_voltage = (uint16_t) data[pos + 1] << 8 | data[pos + 2];
	  pos += 3; // 62
	  // 0x84 0x80 0xD0: Current data                                32976                     0.01 A
	  //63 64
	  jk_bms_battery_info.battery_status.battery_current = getCurrent((uint16_t) data[pos + 1] << 8 | data[pos + 2]);
	  pos += 3; // 65
	  // 0x85 0x0F: Battery remaining capacity                       15 %
	  // 66
	  jk_bms_battery_info.battery_status.battery_soc = (uint8_t) data[pos + 1];
	  pos += 2; // 67
	  // 0x86 0x02: Number of battery temperature sensors             2                        1.0  count
	  // 68
	  jk_bms_battery_info.battery_status.temperature_sensor_count = (uint8_t) data[pos + 1];
	  pos += 2; // 69
	  // 0x87 0x00 0x04: Number of battery cycles                     4                        1.0  count
	  // 70 71
	  jk_bms_battery_info.battery_status.battery_cycles = (uint16_t) data[pos + 1] << 8 | data[pos + 2];
	  pos += 3; // 72
	  // 0x89 0x00 0x00 0x00 0x00: Total battery cycle capacity
	  // 73 74 75 76
	  jk_bms_battery_info.battery_status.battery_cycle_capacity = (uint32_t) data[pos + 1] << 24 | (uint32_t) data[pos + 2] << 16 | (uint32_t) data[pos + 3] << 8 | data[pos + 4];
	  pos += 5; // 77
	  // ignore strings number
	  pos += 3;

	  // 0x8B 0x00 0x00: Battery warning message                     0000 0000 0000 0000
	  uint16_t alarms = (uint16_t) data[pos + 1] << 8 | data[pos + 2];
	  jk_bms_battery_info.battery_alarms.alarm_data = alarms;

	  jk_bms_battery_info.battery_alarms.low_capacity = alarms & 0x01;
	  jk_bms_battery_info.battery_alarms.power_tube_overtemperature = (alarms >> 1) & 0x01;
	  jk_bms_battery_info.battery_alarms.charging_overvoltage = alarms >> 2 & 0x01;
	  jk_bms_battery_info.battery_alarms.discharging_undervoltage = alarms >> 3 & 0x01;
	  jk_bms_battery_info.battery_alarms.battery_over_temperature = alarms >> 4 & 0x01;
	  jk_bms_battery_info.battery_alarms.charging_overcurrent = alarms >> 5 & 0x01;
	  jk_bms_battery_info.battery_alarms.discharging_overcurrent = alarms >> 6 & 0x01;
	  jk_bms_battery_info.battery_alarms.cell_pressure_difference = alarms >> 7 & 0x01;
	  jk_bms_battery_info.battery_alarms.overtemperature_alarm_battery_box = alarms >> 8 & 0x01;
	  jk_bms_battery_info.battery_alarms.battery_low_temperature = alarms >> 9 & 0x01;
	  jk_bms_battery_info.battery_alarms.cell_overvoltage = alarms >> 10 & 0x01;
	  jk_bms_battery_info.battery_alarms.cell_undervoltage = alarms >> 11 & 0x01;
	  jk_bms_battery_info.battery_alarms.a_protection_309_1 = alarms >> 12 & 0x01;
	  jk_bms_battery_info.battery_alarms.a_protection_309_2 = alarms >> 13 & 0x01;

	  pos += 3;
	  // 0x8C 0x00 0x07: Battery status information  - ignore
	  pos += 3;
	  // 0x8E 0x16 0x26: Total voltage overvoltage protection        5670 * 0.01 = 56.70V     0.01 V
	  jk_bms_battery_info.battery_limits.battery_charge_voltage = (uint16_t) data[pos + 1] << 8 | data[pos + 2];
	  pos += 3;
	  // 0x8F 0x10 0xAE: Total voltage undervoltage protection       4270 * 0.01 = 42.70V     0.01 V
	  jk_bms_battery_info.battery_limits.battery_discharge_voltage = (uint16_t) data[pos + 1] << 8 | data[pos + 2];
	  pos += 24;
	  // 0x97 0x00 0x07: Discharge overcurrent protection value       7A                         1.0 A
	  jk_bms_battery_info.battery_limits.battery_discharge_current_limit = (uint16_t) data[pos + 1] << 8 | data[pos + 2];
	  pos += 6;
	  // 0x99 0x00 0x05: Charging overcurrent protection value        5A                         1.0 A
	  jk_bms_battery_info.battery_limits.battery_charge_current_limit = (uint16_t) data[pos + 1] << 8 | data[pos + 2];


}
