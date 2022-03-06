/*
 * jk_bms_485.h
 *
 *  Created on: Mar 4, 2022
 *      Author: maxx
 */

#ifndef INC_JK_BMS_485_H_
#define INC_JK_BMS_485_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

struct jk_bms_cell_voltage {
	uint8_t cell_number;
	uint16_t cell_voltage;
};

struct jk_bms_limits {
	uint16_t battery_charge_voltage;
	uint16_t battery_charge_current_limit;
	uint16_t battery_discharge_current_limit;
	uint16_t battery_discharge_voltage;
};

struct jk_bms_alarms {
	uint16_t alarm_data;
	bool low_capacity;
	bool power_tube_overtemperature;
	bool charging_overvoltage;
	bool discharging_undervoltage;
	bool battery_over_temperature;
	bool charging_overcurrent;
	bool discharging_overcurrent;
	bool cell_pressure_difference;
	bool overtemperature_alarm_battery_box;
	bool battery_low_temperature;
	bool cell_overvoltage;
	bool cell_undervoltage;
	bool a_protection_309_1;
	bool a_protection_309_2;
};

struct jk_bms_battery_status {
	int8_t power_tube_temperature;
	int8_t sensor_temperature_1;
	int8_t sensor_temperature_2;
	int8_t temperature_sensor_count;
	uint16_t battery_voltage;
	int16_t battery_current;
	uint8_t battery_soc;
	uint16_t battery_cycles;
	uint32_t battery_cycle_capacity;
};

struct jk_bms_battery_info {
	uint8_t cells_number;
	struct jk_bms_cell_voltage cells_voltage[24];
	struct jk_bms_battery_status battery_status;
	struct jk_bms_alarms battery_alarms;
	struct jk_bms_limits battery_limits;
};

struct jk_bms_battery_info jk_bms_battery_info;

void Request_JK_Battery_485_Status_Frame(UART_HandleTypeDef uart);
bool JK_Battery_485_Check_Frame_CRC(uint8_t *data, uint16_t frame_size);
void Parse_JK_Battery_485_Status_Frame(uint8_t *data);

#endif /* INC_JK_BMS_485_H_ */
