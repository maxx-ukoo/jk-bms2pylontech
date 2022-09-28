/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2018-2019 Erik Moqvist
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * This file was generated by cantools version 37.0.2 Sun Jan 23 14:34:46 2022.
 */

#include <string.h>

#include "pylon_can_210124.h"

static inline uint8_t pack_left_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_left_shift_u16(
    uint16_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_left_shift_u64(
    uint64_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_right_shift_u16(
    uint16_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value >> shift) & mask);
}

static inline uint8_t pack_right_shift_u64(
    uint64_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value >> shift) & mask);
}

static inline uint16_t unpack_left_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) << shift);
}

static inline uint64_t unpack_left_shift_u64(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint64_t)((uint64_t)(value & mask) << shift);
}

static inline uint8_t unpack_right_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value & mask) >> shift);
}

static inline uint16_t unpack_right_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) >> shift);
}

static inline uint64_t unpack_right_shift_u64(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint64_t)((uint64_t)(value & mask) >> shift);
}

int pylon_can_210124_network_alive_msg_pack(
    uint8_t *dst_p,
    const struct pylon_can_210124_network_alive_msg_t *src_p,
    size_t size)
{
    uint64_t alive_packet;

    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    alive_packet = (uint64_t)src_p->alive_packet;
    dst_p[0] |= pack_left_shift_u64(alive_packet, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u64(alive_packet, 8u, 0xffu);
    dst_p[2] |= pack_right_shift_u64(alive_packet, 16u, 0xffu);
    dst_p[3] |= pack_right_shift_u64(alive_packet, 24u, 0xffu);
    dst_p[4] |= pack_right_shift_u64(alive_packet, 32u, 0xffu);
    dst_p[5] |= pack_right_shift_u64(alive_packet, 40u, 0xffu);
    dst_p[6] |= pack_right_shift_u64(alive_packet, 48u, 0xffu);
    dst_p[7] |= pack_right_shift_u64(alive_packet, 56u, 0xffu);

    return (8);
}

int pylon_can_210124_network_alive_msg_unpack(
    struct pylon_can_210124_network_alive_msg_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint64_t alive_packet;

    if (size < 8u) {
        return (-EINVAL);
    }

    alive_packet = unpack_right_shift_u64(src_p[0], 0u, 0xffu);
    alive_packet |= unpack_left_shift_u64(src_p[1], 8u, 0xffu);
    alive_packet |= unpack_left_shift_u64(src_p[2], 16u, 0xffu);
    alive_packet |= unpack_left_shift_u64(src_p[3], 24u, 0xffu);
    alive_packet |= unpack_left_shift_u64(src_p[4], 32u, 0xffu);
    alive_packet |= unpack_left_shift_u64(src_p[5], 40u, 0xffu);
    alive_packet |= unpack_left_shift_u64(src_p[6], 48u, 0xffu);
    alive_packet |= unpack_left_shift_u64(src_p[7], 56u, 0xffu);
    dst_p->alive_packet = (int64_t)alive_packet;

    return (0);
}

int64_t pylon_can_210124_network_alive_msg_alive_packet_encode(double value)
{
    return (int64_t)(value);
}

double pylon_can_210124_network_alive_msg_alive_packet_decode(int64_t value)
{
    return ((double)value);
}

bool pylon_can_210124_network_alive_msg_alive_packet_is_in_range(int64_t value)
{
    (void)value;

    return (true);
}

int pylon_can_210124_battery_manufacturer_pack(
    uint8_t *dst_p,
    const struct pylon_can_210124_battery_manufacturer_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u64(src_p->manufaturer_string, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u64(src_p->manufaturer_string, 8u, 0xffu);
    dst_p[2] |= pack_right_shift_u64(src_p->manufaturer_string, 16u, 0xffu);
    dst_p[3] |= pack_right_shift_u64(src_p->manufaturer_string, 24u, 0xffu);
    dst_p[4] |= pack_right_shift_u64(src_p->manufaturer_string, 32u, 0xffu);
    dst_p[5] |= pack_right_shift_u64(src_p->manufaturer_string, 40u, 0xffu);
    dst_p[6] |= pack_right_shift_u64(src_p->manufaturer_string, 48u, 0xffu);
    dst_p[7] |= pack_right_shift_u64(src_p->manufaturer_string, 56u, 0xffu);

    return (8);
}

int pylon_can_210124_battery_manufacturer_unpack(
    struct pylon_can_210124_battery_manufacturer_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->manufaturer_string = unpack_right_shift_u64(src_p[0], 0u, 0xffu);
    dst_p->manufaturer_string |= unpack_left_shift_u64(src_p[1], 8u, 0xffu);
    dst_p->manufaturer_string |= unpack_left_shift_u64(src_p[2], 16u, 0xffu);
    dst_p->manufaturer_string |= unpack_left_shift_u64(src_p[3], 24u, 0xffu);
    dst_p->manufaturer_string |= unpack_left_shift_u64(src_p[4], 32u, 0xffu);
    dst_p->manufaturer_string |= unpack_left_shift_u64(src_p[5], 40u, 0xffu);
    dst_p->manufaturer_string |= unpack_left_shift_u64(src_p[6], 48u, 0xffu);
    dst_p->manufaturer_string |= unpack_left_shift_u64(src_p[7], 56u, 0xffu);

    return (0);
}

uint64_t pylon_can_210124_battery_manufacturer_manufaturer_string_encode(double value)
{
    return (uint64_t)(value);
}

double pylon_can_210124_battery_manufacturer_manufaturer_string_decode(uint64_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_manufacturer_manufaturer_string_is_in_range(uint64_t value)
{
    (void)value;

    return (true);
}

int pylon_can_210124_battery_request_pack(
    uint8_t *dst_p,
    const struct pylon_can_210124_battery_request_t *src_p,
    size_t size)
{
    if (size < 2u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 2);

    dst_p[0] |= pack_left_shift_u8(src_p->full_charge_req, 3u, 0x08u);
    dst_p[0] |= pack_left_shift_u8(src_p->force_charge_req_ii, 4u, 0x10u);
    dst_p[0] |= pack_left_shift_u8(src_p->force_charge_req_i, 5u, 0x20u);
    dst_p[0] |= pack_left_shift_u8(src_p->discharge_enable, 6u, 0x40u);
    dst_p[0] |= pack_left_shift_u8(src_p->charge_enable, 7u, 0x80u);

    return (2);
}

int pylon_can_210124_battery_request_unpack(
    struct pylon_can_210124_battery_request_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 2u) {
        return (-EINVAL);
    }

    dst_p->full_charge_req = unpack_right_shift_u8(src_p[0], 3u, 0x08u);
    dst_p->force_charge_req_ii = unpack_right_shift_u8(src_p[0], 4u, 0x10u);
    dst_p->force_charge_req_i = unpack_right_shift_u8(src_p[0], 5u, 0x20u);
    dst_p->discharge_enable = unpack_right_shift_u8(src_p[0], 6u, 0x40u);
    dst_p->charge_enable = unpack_right_shift_u8(src_p[0], 7u, 0x80u);

    return (0);
}

uint8_t pylon_can_210124_battery_request_full_charge_req_encode(double value)
{
    return (uint8_t)(value);
}

double pylon_can_210124_battery_request_full_charge_req_decode(uint8_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_request_full_charge_req_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t pylon_can_210124_battery_request_force_charge_req_ii_encode(double value)
{
    return (uint8_t)(value);
}

double pylon_can_210124_battery_request_force_charge_req_ii_decode(uint8_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_request_force_charge_req_ii_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t pylon_can_210124_battery_request_force_charge_req_i_encode(double value)
{
    return (uint8_t)(value);
}

double pylon_can_210124_battery_request_force_charge_req_i_decode(uint8_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_request_force_charge_req_i_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t pylon_can_210124_battery_request_discharge_enable_encode(double value)
{
    return (uint8_t)(value);
}

double pylon_can_210124_battery_request_discharge_enable_decode(uint8_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_request_discharge_enable_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t pylon_can_210124_battery_request_charge_enable_encode(double value)
{
    return (uint8_t)(value);
}

double pylon_can_210124_battery_request_charge_enable_decode(uint8_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_request_charge_enable_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

int pylon_can_210124_battery_actual_values_u_it_pack(
    uint8_t *dst_p,
    const struct pylon_can_210124_battery_actual_values_u_it_t *src_p,
    size_t size)
{
    uint16_t battery_current;
    uint16_t battery_temperature;
    uint16_t battery_voltage;

    if (size < 6u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 6);

    battery_voltage = (uint16_t)src_p->battery_voltage;
    dst_p[0] |= pack_left_shift_u16(battery_voltage, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(battery_voltage, 8u, 0xffu);
    battery_current = (uint16_t)src_p->battery_current;
    dst_p[2] |= pack_left_shift_u16(battery_current, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(battery_current, 8u, 0xffu);
    battery_temperature = (uint16_t)src_p->battery_temperature;
    dst_p[4] |= pack_left_shift_u16(battery_temperature, 0u, 0xffu);
    dst_p[5] |= pack_right_shift_u16(battery_temperature, 8u, 0xffu);

    return (6);
}

int pylon_can_210124_battery_actual_values_u_it_unpack(
    struct pylon_can_210124_battery_actual_values_u_it_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t battery_current;
    uint16_t battery_temperature;
    uint16_t battery_voltage;

    if (size < 6u) {
        return (-EINVAL);
    }

    battery_voltage = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    battery_voltage |= unpack_left_shift_u16(src_p[1], 8u, 0xffu);
    dst_p->battery_voltage = (int16_t)battery_voltage;
    battery_current = unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    battery_current |= unpack_left_shift_u16(src_p[3], 8u, 0xffu);
    dst_p->battery_current = (int16_t)battery_current;
    battery_temperature = unpack_right_shift_u16(src_p[4], 0u, 0xffu);
    battery_temperature |= unpack_left_shift_u16(src_p[5], 8u, 0xffu);
    dst_p->battery_temperature = (int16_t)battery_temperature;

    return (0);
}

int16_t pylon_can_210124_battery_actual_values_u_it_battery_voltage_encode(double value)
{
    return (int16_t)(value / 0.01);
}

double pylon_can_210124_battery_actual_values_u_it_battery_voltage_decode(int16_t value)
{
    return ((double)value * 0.01);
}

bool pylon_can_210124_battery_actual_values_u_it_battery_voltage_is_in_range(int16_t value)
{
    return (value >= 0);
}

int16_t pylon_can_210124_battery_actual_values_u_it_battery_current_encode(double value)
{
    return (int16_t)(value / 0.1);
}

double pylon_can_210124_battery_actual_values_u_it_battery_current_decode(int16_t value)
{
    return ((double)value * 0.1);
}

bool pylon_can_210124_battery_actual_values_u_it_battery_current_is_in_range(int16_t value)
{
    return ((value >= -2500) && (value <= 2500));
}

int16_t pylon_can_210124_battery_actual_values_u_it_battery_temperature_encode(double value)
{
    return (int16_t)(value / 0.1);
}

double pylon_can_210124_battery_actual_values_u_it_battery_temperature_decode(int16_t value)
{
    return ((double)value * 0.1);
}

bool pylon_can_210124_battery_actual_values_u_it_battery_temperature_is_in_range(int16_t value)
{
    return ((value >= -500) && (value <= 750));
}

int pylon_can_210124_battery_so_c_so_h_pack(
    uint8_t *dst_p,
    const struct pylon_can_210124_battery_so_c_so_h_t *src_p,
    size_t size)
{
    if (size < 4u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 4);

    dst_p[0] |= pack_left_shift_u16(src_p->so_c, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(src_p->so_c, 8u, 0xffu);
    dst_p[2] |= pack_left_shift_u16(src_p->so_h, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(src_p->so_h, 8u, 0xffu);

    return (4);
}

int pylon_can_210124_battery_so_c_so_h_unpack(
    struct pylon_can_210124_battery_so_c_so_h_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 4u) {
        return (-EINVAL);
    }

    dst_p->so_c = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    dst_p->so_c |= unpack_left_shift_u16(src_p[1], 8u, 0xffu);
    dst_p->so_h = unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    dst_p->so_h |= unpack_left_shift_u16(src_p[3], 8u, 0xffu);

    return (0);
}

uint16_t pylon_can_210124_battery_so_c_so_h_so_c_encode(double value)
{
    return (uint16_t)(value);
}

double pylon_can_210124_battery_so_c_so_h_so_c_decode(uint16_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_so_c_so_h_so_c_is_in_range(uint16_t value)
{
    return (value <= 100u);
}

uint16_t pylon_can_210124_battery_so_c_so_h_so_h_encode(double value)
{
    return (uint16_t)(value);
}

double pylon_can_210124_battery_so_c_so_h_so_h_decode(uint16_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_so_c_so_h_so_h_is_in_range(uint16_t value)
{
    return (value <= 100u);
}

int pylon_can_210124_battery_limits_pack(
    uint8_t *dst_p,
    const struct pylon_can_210124_battery_limits_t *src_p,
    size_t size)
{
    uint16_t battery_charge_current_limit;
    uint16_t battery_charge_voltage;
    uint16_t battery_discharge_current_limit;

    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    battery_charge_voltage = (uint16_t)src_p->battery_charge_voltage;
    dst_p[0] |= pack_left_shift_u16(battery_charge_voltage, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(battery_charge_voltage, 8u, 0xffu);
    battery_charge_current_limit = (uint16_t)src_p->battery_charge_current_limit;
    dst_p[2] |= pack_left_shift_u16(battery_charge_current_limit, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(battery_charge_current_limit, 8u, 0xffu);
    battery_discharge_current_limit = (uint16_t)src_p->battery_discharge_current_limit;
    dst_p[4] |= pack_left_shift_u16(battery_discharge_current_limit, 0u, 0xffu);
    dst_p[5] |= pack_right_shift_u16(battery_discharge_current_limit, 8u, 0xffu);
    dst_p[6] |= pack_left_shift_u16(src_p->battery_discharge_voltage, 0u, 0xffu);
    dst_p[7] |= pack_right_shift_u16(src_p->battery_discharge_voltage, 8u, 0xffu);

    return (8);
}

int pylon_can_210124_battery_limits_unpack(
    struct pylon_can_210124_battery_limits_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t battery_charge_current_limit;
    uint16_t battery_charge_voltage;
    uint16_t battery_discharge_current_limit;

    if (size < 8u) {
        return (-EINVAL);
    }

    battery_charge_voltage = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    battery_charge_voltage |= unpack_left_shift_u16(src_p[1], 8u, 0xffu);
    dst_p->battery_charge_voltage = (int16_t)battery_charge_voltage;
    battery_charge_current_limit = unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    battery_charge_current_limit |= unpack_left_shift_u16(src_p[3], 8u, 0xffu);
    dst_p->battery_charge_current_limit = (int16_t)battery_charge_current_limit;
    battery_discharge_current_limit = unpack_right_shift_u16(src_p[4], 0u, 0xffu);
    battery_discharge_current_limit |= unpack_left_shift_u16(src_p[5], 8u, 0xffu);
    dst_p->battery_discharge_current_limit = (int16_t)battery_discharge_current_limit;
    dst_p->battery_discharge_voltage = unpack_right_shift_u16(src_p[6], 0u, 0xffu);
    dst_p->battery_discharge_voltage |= unpack_left_shift_u16(src_p[7], 8u, 0xffu);

    return (0);
}

int16_t pylon_can_210124_battery_limits_battery_charge_voltage_encode(double value)
{
    return (int16_t)(value / 0.1);
}

double pylon_can_210124_battery_limits_battery_charge_voltage_decode(int16_t value)
{
    return ((double)value * 0.1);
}

bool pylon_can_210124_battery_limits_battery_charge_voltage_is_in_range(int16_t value)
{
    return ((value >= 0) && (value <= 750));
}

int16_t pylon_can_210124_battery_limits_battery_charge_current_limit_encode(double value)
{
    return (int16_t)(value / 0.1);
}

double pylon_can_210124_battery_limits_battery_charge_current_limit_decode(int16_t value)
{
    return ((double)value * 0.1);
}

bool pylon_can_210124_battery_limits_battery_charge_current_limit_is_in_range(int16_t value)
{
    return ((value >= 0) && (value <= 5000));
}

int16_t pylon_can_210124_battery_limits_battery_discharge_current_limit_encode(double value)
{
    return (int16_t)(value / 0.1);
}

double pylon_can_210124_battery_limits_battery_discharge_current_limit_decode(int16_t value)
{
    return ((double)value * 0.1);
}

bool pylon_can_210124_battery_limits_battery_discharge_current_limit_is_in_range(int16_t value)
{
    return ((value >= -5000) && (value <= 0));
}

uint16_t pylon_can_210124_battery_limits_battery_discharge_voltage_encode(double value)
{
    return (uint16_t)(value / 0.1);
}

double pylon_can_210124_battery_limits_battery_discharge_voltage_decode(uint16_t value)
{
    return ((double)value * 0.1);
}

bool pylon_can_210124_battery_limits_battery_discharge_voltage_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

int pylon_can_210124_battery_error_warnings_pack(
    uint8_t *dst_p,
    const struct pylon_can_210124_battery_error_warnings_t *src_p,
    size_t size)
{
    if (size < 7u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 7);

    dst_p[0] |= pack_left_shift_u8(src_p->overvoltage_err, 1u, 0x02u);
    dst_p[0] |= pack_left_shift_u8(src_p->undervoltage_err, 2u, 0x04u);
    dst_p[0] |= pack_left_shift_u8(src_p->overtemperature_err, 3u, 0x08u);
    dst_p[0] |= pack_left_shift_u8(src_p->undertemperature_err, 4u, 0x10u);
    dst_p[0] |= pack_left_shift_u8(src_p->overcurrent_discharge_err, 7u, 0x80u);
    dst_p[1] |= pack_left_shift_u8(src_p->charge_overcurrent_err, 0u, 0x01u);
    dst_p[1] |= pack_left_shift_u8(src_p->system_error, 3u, 0x08u);
    dst_p[2] |= pack_left_shift_u8(src_p->voltage_high_warn, 1u, 0x02u);
    dst_p[2] |= pack_left_shift_u8(src_p->voltage_low_warn, 2u, 0x04u);
    dst_p[2] |= pack_left_shift_u8(src_p->temperature_high_warn, 3u, 0x08u);
    dst_p[2] |= pack_left_shift_u8(src_p->temperature_low_warn, 4u, 0x10u);
    dst_p[2] |= pack_left_shift_u8(src_p->discharge_current_high_warn, 7u, 0x80u);
    dst_p[3] |= pack_left_shift_u8(src_p->charge_current_high_warn, 0u, 0x01u);
    dst_p[3] |= pack_left_shift_u8(src_p->internal_error_warn, 3u, 0x08u);
    dst_p[4] |= pack_left_shift_u8(src_p->module_numbers, 0u, 0xffu);
    dst_p[5] = 0x50;
    dst_p[6] = 0x4e;

    return (7);
}

int pylon_can_210124_battery_error_warnings_unpack(
    struct pylon_can_210124_battery_error_warnings_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 7u) {
        return (-EINVAL);
    }

    dst_p->overvoltage_err = unpack_right_shift_u8(src_p[0], 1u, 0x02u);
    dst_p->undervoltage_err = unpack_right_shift_u8(src_p[0], 2u, 0x04u);
    dst_p->overtemperature_err = unpack_right_shift_u8(src_p[0], 3u, 0x08u);
    dst_p->undertemperature_err = unpack_right_shift_u8(src_p[0], 4u, 0x10u);
    dst_p->overcurrent_discharge_err = unpack_right_shift_u8(src_p[0], 7u, 0x80u);
    dst_p->charge_overcurrent_err = unpack_right_shift_u8(src_p[1], 0u, 0x01u);
    dst_p->system_error = unpack_right_shift_u8(src_p[1], 3u, 0x08u);
    dst_p->voltage_high_warn = unpack_right_shift_u8(src_p[2], 1u, 0x02u);
    dst_p->voltage_low_warn = unpack_right_shift_u8(src_p[2], 2u, 0x04u);
    dst_p->temperature_high_warn = unpack_right_shift_u8(src_p[2], 3u, 0x08u);
    dst_p->temperature_low_warn = unpack_right_shift_u8(src_p[2], 4u, 0x10u);
    dst_p->discharge_current_high_warn = unpack_right_shift_u8(src_p[2], 7u, 0x80u);
    dst_p->charge_current_high_warn = unpack_right_shift_u8(src_p[3], 0u, 0x01u);
    dst_p->internal_error_warn = unpack_right_shift_u8(src_p[3], 3u, 0x08u);
    dst_p->module_numbers = unpack_right_shift_u8(src_p[4], 0u, 0xffu);

    return (0);
}

uint8_t pylon_can_210124_battery_error_warnings_overvoltage_err_encode(double value)
{
    return (uint8_t)(value);
}

double pylon_can_210124_battery_error_warnings_overvoltage_err_decode(uint8_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_error_warnings_overvoltage_err_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t pylon_can_210124_battery_error_warnings_undervoltage_err_encode(double value)
{
    return (uint8_t)(value);
}

double pylon_can_210124_battery_error_warnings_undervoltage_err_decode(uint8_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_error_warnings_undervoltage_err_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t pylon_can_210124_battery_error_warnings_overtemperature_err_encode(double value)
{
    return (uint8_t)(value);
}

double pylon_can_210124_battery_error_warnings_overtemperature_err_decode(uint8_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_error_warnings_overtemperature_err_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t pylon_can_210124_battery_error_warnings_undertemperature_err_encode(double value)
{
    return (uint8_t)(value);
}

double pylon_can_210124_battery_error_warnings_undertemperature_err_decode(uint8_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_error_warnings_undertemperature_err_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t pylon_can_210124_battery_error_warnings_overcurrent_discharge_err_encode(double value)
{
    return (uint8_t)(value);
}

double pylon_can_210124_battery_error_warnings_overcurrent_discharge_err_decode(uint8_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_error_warnings_overcurrent_discharge_err_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t pylon_can_210124_battery_error_warnings_charge_overcurrent_err_encode(double value)
{
    return (uint8_t)(value);
}

double pylon_can_210124_battery_error_warnings_charge_overcurrent_err_decode(uint8_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_error_warnings_charge_overcurrent_err_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t pylon_can_210124_battery_error_warnings_system_error_encode(double value)
{
    return (uint8_t)(value);
}

double pylon_can_210124_battery_error_warnings_system_error_decode(uint8_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_error_warnings_system_error_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t pylon_can_210124_battery_error_warnings_voltage_high_warn_encode(double value)
{
    return (uint8_t)(value);
}

double pylon_can_210124_battery_error_warnings_voltage_high_warn_decode(uint8_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_error_warnings_voltage_high_warn_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t pylon_can_210124_battery_error_warnings_voltage_low_warn_encode(double value)
{
    return (uint8_t)(value);
}

double pylon_can_210124_battery_error_warnings_voltage_low_warn_decode(uint8_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_error_warnings_voltage_low_warn_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t pylon_can_210124_battery_error_warnings_temperature_high_warn_encode(double value)
{
    return (uint8_t)(value);
}

double pylon_can_210124_battery_error_warnings_temperature_high_warn_decode(uint8_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_error_warnings_temperature_high_warn_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t pylon_can_210124_battery_error_warnings_temperature_low_warn_encode(double value)
{
    return (uint8_t)(value);
}

double pylon_can_210124_battery_error_warnings_temperature_low_warn_decode(uint8_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_error_warnings_temperature_low_warn_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t pylon_can_210124_battery_error_warnings_discharge_current_high_warn_encode(double value)
{
    return (uint8_t)(value);
}

double pylon_can_210124_battery_error_warnings_discharge_current_high_warn_decode(uint8_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_error_warnings_discharge_current_high_warn_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t pylon_can_210124_battery_error_warnings_charge_current_high_warn_encode(double value)
{
    return (uint8_t)(value);
}

double pylon_can_210124_battery_error_warnings_charge_current_high_warn_decode(uint8_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_error_warnings_charge_current_high_warn_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t pylon_can_210124_battery_error_warnings_internal_error_warn_encode(double value)
{
    return (uint8_t)(value);
}

double pylon_can_210124_battery_error_warnings_internal_error_warn_decode(uint8_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_error_warnings_internal_error_warn_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t pylon_can_210124_battery_error_warnings_module_numbers_encode(double value)
{
    return (uint8_t)(value);
}

double pylon_can_210124_battery_error_warnings_module_numbers_decode(uint8_t value)
{
    return ((double)value);
}

bool pylon_can_210124_battery_error_warnings_module_numbers_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}