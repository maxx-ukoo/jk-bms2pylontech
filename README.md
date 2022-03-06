
# JKBMS to Pylontech protocol converter

Read data from JK BMS via RS485 protocol and send updates to solar inverter via Pylontech low voltage US2000 battery protocol

# TODO

* Test current sensor on charge and discharde
* Check converting alarms from JK BMS to Pytontech 

# Current state
Tested with Deye 3 phase hybrid inverter but only with CAN connection. Battery configured but not enabled
Inverter show battery information:
* current battery voltage
* current battery temperature
* SOC/SOH
* battery charge voltage
* charge current limit
* discharge current limit


# Hardware

Tested on [STM32F4XX_M](https://stm32-base.org/boards/STM32F407VGT6-STM32F4XX-M.html) board

1. PD1/PD2 CAN interface to inverter
2. PA11/PA12 - serial output to USB for logs
3. PA9/PA10 - RS485 interface to JKBMS
4. Support ILI9341 LCD via SPI. (can be disabled)


## Disable LCD

* to disable LCD comment line #define ENABLE_LCD in main.h


# Useful links

[STM32F4xx board description](https://stm32-base.org/boards/STM32F407VGT6-STM32F4XX-M.html)
[Pylon battery real data](https://www.setfirelabs.com/green-energy/pylontech-can-reading-can-replication)
[ESPHome component for parse JKBMS response](https://github.com/syssi/esphome-jk-bms/)