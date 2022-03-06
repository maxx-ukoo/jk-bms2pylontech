/*
 * jk_bms_pylon.h
 *
 *  Created on: Mar 5, 2022
 *      Author: maxx
 */

#ifndef INC_JK_BMS_PYLON_H_
#define INC_JK_BMS_PYLON_H_

#include "stm32f4xx_hal.h"
#include "pylon_can_210124.h"

void Tx_JK_BMS_Status_via_CAN(CAN_HandleTypeDef hcan1);
void Create_Limits_Frame(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p, const struct pylon_can_210124_battery_limits_t *battery_limits);
void Create_SOH_SOC_Frame(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p, const struct pylon_can_210124_battery_so_c_so_h_t *battery_soc_soh);
void Create_Actual_Values_Frame(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p, const struct pylon_can_210124_battery_actual_values_u_it_t *actual_values);
void Create_Manufacturer_Frame(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p);
void Create_Battery_Request_Frame(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p, const struct pylon_can_210124_battery_request_t *battery_request);
void Create_Alive_Msg_Frame(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p);
void Create_Errors_Warnings_Frame(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p, const struct pylon_can_210124_battery_error_warnings_t *errors_warnings);

#endif /* INC_JK_BMS_PYLON_H_ */
