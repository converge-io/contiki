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
/**
 * \addtogroup cc26xx-platforms
 * @{
 *
 * \defgroup cc26xx-examples CC26xx Example Projects
 *
 * Example projects for CC26xx-based platforms.
 * @{
 *
 * \defgroup cc26xx-demo CC26xx Demo Project
 *
 *   Example project demonstrating the CC26xx platforms
 *
 *   This example will work for the following boards:
 *   - srf06-cc26xx: SmartRF06EB + CC26XX EM
 *   - sensortag-cc26xx: CC26XX sensortag
 *
 *   By default, the example will build for the srf06-cc26xx board. To switch
 *   between platforms:
 *   - make clean
 *   - make BOARD=sensortag-cc26xx savetarget
 *
 *     or
 *
 *     make BOARD=srf06-cc26xx savetarget
 *
 *   This is an IPv6/RPL-enabled example. Thus, if you have a border router in
 *   your installation (same RDC layer, same PAN ID and RF channel), you should
 *   be able to ping6 this demo node.
 *
 *   This example also demonstrates CC26xx BLE operation. The process starts
 *   the BLE beacon daemon (implemented in the RF driver). The daemon will
 *   send out a BLE beacon periodically. Use any BLE-enabled application (e.g.
 *   LightBlue on OS X or the TI BLE Multitool smartphone app) and after a few
 *   seconds the cc26xx device will be discovered.
 *
 * - etimer/clock : Every CC26XX_DEMO_LOOP_INTERVAL clock ticks the LED defined
 *                  as CC26XX_DEMO_LEDS_PERIODIC will toggle and the device
 *                  will print out readings from some supported sensors
 * - sensors      : Some sensortag sensors are read asynchronously (see sensor
 *                  documentation). For those, this example will print out
 *                  readings in a staggered fashion at a random interval
 * - Buttons      : CC26XX_DEMO_SENSOR_1 button will toggle CC26XX_DEMO_LEDS_BUTTON
 *                - CC26XX_DEMO_SENSOR_2 turns on LEDS_REBOOT and causes a
 *                  watchdog reboot
 *                - The remaining buttons will just print something
 *                - The example also shows how to retrieve the duration of a
 *                  button press (in ticks). The driver will generate a
 *                  sensors_changed event upon button release
 * - Reed Relay   : Will toggle the sensortag buzzer on/off
 *
 * @{
 *
 * \file
 *     Example demonstrating the cc26xx platforms
 */
#include "contiki.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "dev/leds.h"
#include "dev/watchdog.h"
#include "random.h"
#include "button-sensor.h"
#include "batmon-sensor.h"
#include "board-peripherals.h"
//#include "rf-core/rf-ble.h"

#include "ti-lib.h"

#include <stdio.h>
#include <stdint.h>
/*---------------------------------------------------------------------------*/
#define CC26XX_DEMO_LOOP_INTERVAL       (CLOCK_SECOND * 20)
#define CC26XX_DEMO_LEDS_PERIODIC       LEDS_YELLOW
#define CC26XX_DEMO_LEDS_BUTTON         LEDS_RED
#define CC26XX_DEMO_LEDS_REBOOT         LEDS_ALL
/*---------------------------------------------------------------------------*/
#define CC26XX_DEMO_SENSOR_NONE         (void *)0xFFFFFFFF

#define CC26XX_DEMO_SENSOR_1     &button_select_sensor

#if BOARD_SENSORTAG
#define CC26XX_DEMO_SENSOR_3     CC26XX_DEMO_SENSOR_NONE
#define CC26XX_DEMO_SENSOR_4     CC26XX_DEMO_SENSOR_NONE
#define CC26XX_DEMO_SENSOR_5     &reed_relay_sensor
#else
#define CC26XX_DEMO_SENSOR_3     &button_up_sensor
#define CC26XX_DEMO_SENSOR_4     &button_down_sensor
#define CC26XX_DEMO_SENSOR_5     &button_select_sensor
#endif
/*---------------------------------------------------------------------------*/
static struct etimer et;
/*---------------------------------------------------------------------------*/
PROCESS(cc26xx_demo_process, "cc26xx demo process");
AUTOSTART_PROCESSES(&cc26xx_demo_process);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
static void
get_sync_sensor_readings(void)
{
  int value;

  printf("-----------------------------------------\n");

  value = batmon_sensor.value(BATMON_SENSOR_TYPE_TEMP);
  printf("Bat: Temp=%d C\n", value);

  value = batmon_sensor.value(BATMON_SENSOR_TYPE_VOLT);
  printf("Bat: Volt=%d mV\n", (value * 125) >> 5);

  return;
}
/*---------------------------------------------------------------------------*/
static void
init_sensors(void)
{


  SENSORS_ACTIVATE(batmon_sensor);
}
/*---------------------------------------------------------------------------*/
static void
init_sensor_readings(void)
{

}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc26xx_demo_process, ev, data)
{

  PROCESS_BEGIN();

  printf("CC26XX demo\n");

  init_sensors();

  /* Init the BLE advertisement daemon */
  //rf_ble_beacond_config(0, BOARD_STRING);
  //rf_ble_beacond_start();

  etimer_set(&et, CC26XX_DEMO_LOOP_INTERVAL);
  get_sync_sensor_readings();
  init_sensor_readings();

  while(1) {

    PROCESS_YIELD();

    if(ev == PROCESS_EVENT_TIMER) {
      if(data == &et) {
        leds_toggle(CC26XX_DEMO_LEDS_PERIODIC);

        get_sync_sensor_readings();

        etimer_set(&et, CC26XX_DEMO_LOOP_INTERVAL);
      }
    } else if(ev == sensors_event) {
      if(data == CC26XX_DEMO_SENSOR_1) {
        printf("Left: Pin %d, press duration %d clock ticks\n",
               (CC26XX_DEMO_SENSOR_1)->value(BUTTON_SENSOR_VALUE_STATE),
               (CC26XX_DEMO_SENSOR_1)->value(BUTTON_SENSOR_VALUE_DURATION));

        if((CC26XX_DEMO_SENSOR_1)->value(BUTTON_SENSOR_VALUE_DURATION) >
           CLOCK_SECOND) {
          printf("Long button press!\n");
        }

        leds_toggle(CC26XX_DEMO_LEDS_BUTTON);
      }  else if(data == CC26XX_DEMO_SENSOR_5) {
#if BOARD_SENSORTAG
        if(buzzer_state()) {
          buzzer_stop();
        } else {
          buzzer_start(1000);
        }
      } else if(ev == sensors_event && data == &bmp_280_sensor) {
        get_bmp_reading();
      } else if(ev == sensors_event && data == &opt_3001_sensor) {
        get_light_reading();
      } else if(ev == sensors_event && data == &hdc_1000_sensor) {
        get_hdc_reading();
      } else if(ev == sensors_event && data == &tmp_007_sensor) {
        get_tmp_reading();
      } else if(ev == sensors_event && data == &mpu_9250_sensor) {
        get_mpu_reading();
#else
        printf("Sel: Pin %d, press duration %d clock ticks\n",
               button_select_sensor.value(BUTTON_SENSOR_VALUE_STATE),
               button_select_sensor.value(BUTTON_SENSOR_VALUE_DURATION));
#endif
      }
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */
