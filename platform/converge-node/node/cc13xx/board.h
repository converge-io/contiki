/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/** \addtogroup cc26xx-srf-tag
 * @{
 *
 * \defgroup srf06-cc13xx-peripherals Peripherals for the SmartRF06EB + CC1310EM
 *
 * Defines related to the SmartRF06 Evaluation Board with a CC1310EM
 *
 * This file provides connectivity information on LEDs, Buttons, UART and
 * other peripherals
 *
 * This file can be used as the basis to configure other boards using the
 * CC13xx/CC26xx code as their basis.
 *
 * This file is not meant to be modified by the user.
 * @{
 *
 * \file
 * Header file with definitions related to the I/O connections on the TI
 * SmartRF06 Evaluation Board with a CC1310EM
 *
 * \note   Do not include this file directly. It gets included by contiki-conf
 *         after all relevant directives have been set.
 */
/*---------------------------------------------------------------------------*/
#ifndef BOARD_H_
#define BOARD_H_
/*---------------------------------------------------------------------------*/
#include "ioc.h"
/*---------------------------------------------------------------------------*/
/**
 * \name LED configurations
 *
 * Those values are not meant to be modified by the user
 * @{
 */
/* Some files include leds.h before us, so we need to get rid of defaults in
 * leds.h before we provide correct definitions */
#undef LEDS_GREEN
#undef LEDS_YELLOW
#undef LEDS_RED
#undef LEDS_CONF_ALL

#define LEDS_RED       1 /**< LED1 (Red)    */
#define LEDS_YELLOW    2 /**< LED2 (Yellow) */
#define LEDS_GREEN     4 /**< LED3 (Green)  */

#define LEDS_CONF_ALL 7

/* Notify various examples that we have LEDs */
#define PLATFORM_HAS_LEDS        1
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name LED IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_LED_1          IOID_3
#define BOARD_IOID_LED_2          IOID_2
#define BOARD_IOID_LED_3          IOID_1
#define BOARD_LED_1               (1 << BOARD_IOID_LED_1)
#define BOARD_LED_2               (1 << BOARD_IOID_LED_2)
#define BOARD_LED_3               (1 << BOARD_IOID_LED_3)
#define BOARD_LED_ALL             (BOARD_LED_1 | BOARD_LED_2 | BOARD_LED_3)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name UART IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_UART_RX        IOID_27
#define BOARD_IOID_UART_TX        IOID_4
#define BOARD_IOID_UART_CTS       IOID_UNUSED
#define BOARD_IOID_UART_RTS       IOID_UNUSED
#define BOARD_UART_RX             (1 << BOARD_IOID_UART_RX)
#define BOARD_UART_TX             (1 << BOARD_IOID_UART_TX)
#define BOARD_UART_CTS            (1 << BOARD_IOID_UART_CTS)
#define BOARD_UART_RTS            (1 << BOARD_IOID_UART_RTS)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Button IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_KEY_SELECT     IOID_8
#define BOARD_KEY_SELECT          (1 << BOARD_IOID_KEY_SELECT)
/** @} */
/**
 * \name Device string used on startup
 * @{
 */
#define BOARD_STRING "Converge Node v3.0.1 - Brock"
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Converge Node I/O Mapping
 *
 * Set all these to high-z until used
 * @{
 */
#define BOARD_IOID_TEMP_D         IOID_5
#define BOARD_IOID_SDA            IOID_6
#define BOARD_IOID_SCL            IOID_7
#define BOARD_IOID_MISO           IOID_9
#define BOARD_IOID_MOSI           IOID_11
#define BOARD_IOID_SCK            IOID_10
#define BOARD_IOID_CS             IOID_12
#define BOARD_IOID_SD_PWR         IOID_13
#define BOARD_IOID_UC_IO1         IOID_14
#define BOARD_IOID_JTAG_TDO       IOID_16
#define BOARD_IOID_JTAG_TDI       IOID_17
#define BOARD_IOID_SD_CD          IOID_18
#define BOARD_IOID_UC_IO2         IOID_20
#define BOARD_IOID_RPI_CD         IOID_21
#define BOARD_IOID_V_BATT         IOID_23
#define BOARD_IOID_VC_ADC         IOID_24
#define BOARD_IOID_ACC_INT2       IOID_25
#define BOARD_IOID_ACC_INT1       IOID_26
#define BOARD_IOID_ACC_VDD        IOID_28

#define BOARD_TEMP_D         (1 << BOARD_IOID_TEMP_D)
#define BOARD_SDA            (1 << BOARD_IOID_SDA)
#define BOARD_SCL            (1 << BOARD_IOID_SCL)
#define BOARD_MISO           (1 << BOARD_IOID_MISO)
#define BOARD_MOSI           (1 << BOARD_IOID_MOSI)
#define BOARD_SCK            (1 << BOARD_IOID_SCK)
#define BOARD_CS             (1 << BOARD_IOID_CS)
#define BOARD_SD_PWR         (1 << BOARD_IOID_SD_PWR)
#define BOARD_UC_IO1         (1 << BOARD_IOID_UC_IO1)
#define BOARD_JTAG_TDO       (1 << BOARD_IOID_JTAG_TDO)
#define BOARD_JTAG_TDI       (1 << BOARD_IOID_JTAG_TDI)
#define BOARD_SD_CD          (1 << BOARD_IOID_SD_CD)
#define BOARD_UC_IO2         (1 << BOARD_IOID_UC_IO2)
#define BOARD_RPI_CD         (1 << BOARD_IOID_RPI_CD)
#define BOARD_V_BATT         (1 << BOARD_IOID_V_BATT)
#define BOARD_VC_ADC         (1 << BOARD_IOID_VC_ADC)
#define BOARD_ACC_INT2       (1 << BOARD_IOID_ACC_INT2)
#define BOARD_ACC_INT1       (1 << BOARD_IOID_ACC_INT1)
#define BOARD_ACC_VDD        (1 << BOARD_IOID_ACC_VDD)
/** @} */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
#endif /* BOARD_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */