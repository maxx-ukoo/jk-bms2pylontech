/*
 * jk_bms_pylon.c
 *
 *  Created on: Mar 5, 2022
 *      Author: maxx
 */
#include "jk_bms_pylon.h"
#include "jk_bms_485.h"
#include "pylon_can_210124.h"
#include "stm32f4xx_hal.h"
#define LED_D2_Pin GPIO_PIN_1
#define LED_D2_GPIO_Port GPIOA

CAN_TxHeaderTypeDef		TxHeader;
uint8_t            		TxData[8];
uint32_t				TxMailbox;

void TX_CAN_Message(CAN_HandleTypeDef hcan) {

	//HAL_GPIO_WritePin(LED_D2_GPIO_Port, LED_D2_Pin, GPIO_PIN_SET);
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) < 1) {};
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		//printf("TX error: %ld\r\n", TxHeader.StdId);
	}
	//HAL_GPIO_TogglePin(LED_D2_GPIO_Port, LED_D2_Pin); //Toggle the state of pin
	//waiting for message to leave
	while(HAL_CAN_IsTxMessagePending(&hcan, TxMailbox)) {}
	//HAL_GPIO_WritePin(LED_D2_GPIO_Port, LED_D2_Pin, GPIO_PIN_RESET);
}

void Tx_JK_BMS_Status_via_CAN(CAN_HandleTypeDef hcan) {
	struct pylon_can_210124_battery_limits_t battery_limits = {
			// JK <= 6720 = 67.20V = 672 => PYLON
			.battery_charge_voltage = jk_bms_battery_info.battery_limits.battery_charge_voltage / 10,
			// JK <= 25 = 25A = 250 => PYLON
			.battery_charge_current_limit = jk_bms_battery_info.battery_limits.battery_charge_current_limit * 10,
			// JK <= 300 = 300A = -3000 => PYLON
			.battery_discharge_current_limit = jk_bms_battery_info.battery_limits.battery_discharge_current_limit * 10,
			// JK <= 4512 = 45.12V = 451 => PYLON
			.battery_discharge_voltage = jk_bms_battery_info.battery_limits.battery_discharge_voltage / 10
	};
	Create_Limits_Frame(&TxHeader, TxData, &battery_limits); //0x351
	TX_CAN_Message(hcan);
	struct pylon_can_210124_battery_so_c_so_h_t battery_soc_soh = { .so_c =
			jk_bms_battery_info.battery_status.battery_soc, .so_h = 95 };
	Create_SOH_SOC_Frame(&TxHeader, TxData, &battery_soc_soh); //0x355
	TX_CAN_Message(hcan);

	int16_t temperature = jk_bms_battery_info.battery_status.power_tube_temperature;
	if (jk_bms_battery_info.battery_status.sensor_temperature_1 > temperature) {
		temperature = jk_bms_battery_info.battery_status.sensor_temperature_1;
	}
	if (jk_bms_battery_info.battery_status.sensor_temperature_2 > temperature) {
		temperature = jk_bms_battery_info.battery_status.sensor_temperature_2;
	}

	struct pylon_can_210124_battery_actual_values_u_it_t actual_values = {
			// JK <= 6267 = 62.67V = 6267 => PYLON
			.battery_voltage = jk_bms_battery_info.battery_status.battery_voltage,
			// TODO test get current on real env
			.battery_current = 0,
			// JK <= 10 = 10 C = 100 => PYLON
			.battery_temperature = temperature * 10
	};
	Create_Actual_Values_Frame(&TxHeader, TxData, &actual_values); //0x356
	TX_CAN_Message(hcan);
	Create_Manufacturer_Frame(&TxHeader, TxData); //0x35e
	TX_CAN_Message(hcan);
	bool request_charge = 0;
	if (battery_soc_soh.so_c < 20) {
		request_charge = 1;
	}
	struct pylon_can_210124_battery_request_t battery_request = {
			.full_charge_req = 0, .force_charge_req_ii = 0,
			.force_charge_req_i = request_charge, .discharge_enable = 1,
			.charge_enable = 1 };
	Create_Battery_Request_Frame(&TxHeader, TxData, &battery_request); //0x35c
	TX_CAN_Message(hcan);

	Create_Alive_Msg_Frame(&TxHeader, TxData); //0x305
	TX_CAN_Message(hcan);
	if (jk_bms_battery_info.battery_alarms.alarm_data > 0) {
		struct pylon_can_210124_battery_error_warnings_t errors_warnings = {
				.overvoltage_err = jk_bms_battery_info.battery_alarms.charging_overvoltage,
				.undervoltage_err = jk_bms_battery_info.battery_alarms.discharging_undervoltage,
				.overtemperature_err = jk_bms_battery_info.battery_alarms.battery_over_temperature,
				.undertemperature_err = jk_bms_battery_info.battery_alarms.battery_low_temperature,
				.overcurrent_discharge_err = jk_bms_battery_info.battery_alarms.discharging_overcurrent,
				.charge_overcurrent_err = jk_bms_battery_info.battery_alarms.charging_overcurrent,
				.system_error = 0,
				.voltage_high_warn = jk_bms_battery_info.battery_alarms.cell_overvoltage,
				.voltage_low_warn = jk_bms_battery_info.battery_alarms.cell_undervoltage,
				.temperature_high_warn = jk_bms_battery_info.battery_alarms.power_tube_overtemperature,
				.temperature_low_warn = jk_bms_battery_info.battery_alarms.battery_low_temperature,
				.discharge_current_high_warn = 0,
				.charge_current_high_warn = 0, .internal_error_warn = 0,
				.module_numbers = 1
		};
		Create_Errors_Warnings_Frame(&TxHeader, TxData, &errors_warnings); //0x359
		TX_CAN_Message(hcan);
	}

}

//https://www.setfirelabs.com/green-energy/pylontech-can-reading-can-replication

//0x351 – 14 02 74 0E 74 0E CC 01 – Battery voltage + current limits
void Create_Limits_Frame(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p, const struct pylon_can_210124_battery_limits_t *battery_limits) {
		TxHeader->IDE = CAN_ID_STD;
		TxHeader->StdId = PYLON_CAN_210124_BATTERY_LIMITS_FRAME_ID;
		TxHeader->RTR = CAN_RTR_DATA;
		TxHeader->DLC = pylon_can_210124_battery_limits_pack(dst_p, battery_limits, PYLON_CAN_210124_BATTERY_LIMITS_LENGTH);
}

//0x355 – 1A 00 64 00 – State of Health (SOH) / State of Charge (SOC)
void Create_SOH_SOC_Frame(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p, const struct pylon_can_210124_battery_so_c_so_h_t *battery_soc_soh) {
	TxHeader->IDE = CAN_ID_STD;
	TxHeader->StdId = PYLON_CAN_210124_BATTERY_SO_C_SO_H_FRAME_ID;
	TxHeader->RTR = CAN_RTR_DATA;
	TxHeader->DLC = pylon_can_210124_battery_so_c_so_h_pack(dst_p, battery_soc_soh, PYLON_CAN_210124_BATTERY_SO_C_SO_H_LENGTH);
}

//0x356 – 4e 13 02 03 04 05 – Voltage / Current / Temp
void Create_Actual_Values_Frame(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p, const struct pylon_can_210124_battery_actual_values_u_it_t *actual_values) {
	TxHeader->IDE = CAN_ID_STD;
	TxHeader->StdId = PYLON_CAN_210124_BATTERY_ACTUAL_VALUES_U_IT_FRAME_ID;
	TxHeader->RTR = CAN_RTR_DATA;
	TxHeader->DLC = pylon_can_210124_battery_actual_values_u_it_pack(dst_p, actual_values, PYLON_CAN_210124_BATTERY_ACTUAL_VALUES_U_IT_LENGTH);
}

//0x35E – 50 59 4C 4F 4E 20 20 20 – Manufacturer name (“PYLON “)
void Create_Manufacturer_Frame(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p) {
	struct pylon_can_210124_battery_manufacturer_t manufacturer = {
			.manufaturer_string = 0x50594C4F4E202020
	};

	TxHeader->IDE = CAN_ID_STD;
	TxHeader->StdId = PYLON_CAN_210124_BATTERY_MANUFACTURER_FRAME_ID;
	TxHeader->RTR = CAN_RTR_DATA;
	TxHeader->DLC = pylon_can_210124_battery_manufacturer_pack(dst_p, &manufacturer, PYLON_CAN_210124_BATTERY_MANUFACTURER_LENGTH);
}

//0x35C – C0 00 – Battery charge request flags
void Create_Battery_Request_Frame(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p, const struct pylon_can_210124_battery_request_t *battery_request) {
	TxHeader->IDE = CAN_ID_STD;
	TxHeader->StdId = PYLON_CAN_210124_BATTERY_REQUEST_FRAME_ID;
	TxHeader->RTR = CAN_RTR_DATA;
	TxHeader->DLC = pylon_can_210124_battery_request_pack(dst_p, battery_request, PYLON_CAN_210124_BATTERY_REQUEST_LENGTH);
}

//0x305 – alive message
void Create_Alive_Msg_Frame(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p) {
	struct pylon_can_210124_network_alive_msg_t alive_message = {
		    .alive_packet = 33
	};

	TxHeader->IDE = CAN_ID_STD;
	TxHeader->StdId = PYLON_CAN_210124_NETWORK_ALIVE_MSG_FRAME_ID;
	TxHeader->RTR = CAN_RTR_DATA;
	TxHeader->DLC = pylon_can_210124_network_alive_msg_pack(dst_p, &alive_message, PYLON_CAN_210124_NETWORK_ALIVE_MSG_LENGTH);
}

//0x359 – 00 00 00 00 0A 50 4E – Protection & Alarm flags
void Create_Errors_Warnings_Frame(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p, const struct pylon_can_210124_battery_error_warnings_t *errors_warnings) {
	TxHeader->IDE = CAN_ID_STD;
	TxHeader->StdId = PYLON_CAN_210124_BATTERY_ERROR_WARNINGS_FRAME_ID;
	TxHeader->RTR = CAN_RTR_DATA;
	TxHeader->DLC = pylon_can_210124_battery_error_warnings_pack(dst_p, errors_warnings, PYLON_CAN_210124_BATTERY_ERROR_WARNINGS_LENGTH);
}
