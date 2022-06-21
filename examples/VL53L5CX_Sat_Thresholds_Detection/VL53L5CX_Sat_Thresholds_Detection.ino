/**
 ******************************************************************************
 * @file    VL53L5CX_Sat_Thresholds_Detection.ino
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    11 November 2021
 * @brief   Arduino test application for the STMicrolectronics VL53L5CX
 *          proximity sensor satellite based on FlightSense.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2021 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/*
 * To use these examples you need to connect the VL53L5CX satellite sensor directly to the Nucleo board with wires as explained below:
 * pin 1 (GND) of the VL53L5CX satellite connected to GND of the Nucleo board
 * pin 2 (IOVDD) of the VL53L5CX satellite connected to 3V3 pin of the Nucleo board
 * pin 3 (AVDD) of the VL53L5CX satellite connected to 5V pin of the Nucleo board
 * pin 4 (PWREN) of the VL53L5CX satellite connected to pin A5 of the Nucleo board
 * pin 5 (LPn) of the VL53L5CX satellite connected to pin A3 of the Nucleo board
 * pin 6 (SCL) of the VL53L5CX satellite connected to pin D15 (SCL) of the Nucleo board
 * pin 7 (SDA) of the VL53L5CX satellite connected to pin D14 (SDA) of the Nucleo board
 * pin 8 (I2C_RST) of the VL53L5CX satellite connected to pin A1 of the Nucleo board
 * pin 9 (INT) of the VL53L5CX satellite connected to pin A2 of the Nucleo board
 */
/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <vl53l5cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#ifdef ARDUINO_SAM_DUE
  #define DEV_I2C Wire1
#else
  #define DEV_I2C Wire
#endif
#define SerialPort Serial

#ifndef LED_BUILTIN
  #define LED_BUILTIN 13
#endif
#define LedPin LED_BUILTIN

#define LPN_PIN A3
#define I2C_RST_PIN A1
#define PWREN_PIN A5
#define INT_PIN A2

// Components.
VL53L5CX sensor_vl53l5cx_sat(&DEV_I2C, LPN_PIN, I2C_RST_PIN);

volatile int interruptCount = 0;

void measure(void);
void blink_led_loop(void);

void measure(void)
{
  interruptCount = 1;
}

void blink_led_loop(void)
{
  do {
    // Blink the led forever
    digitalWrite(LedPin, HIGH);
    delay(500);
    digitalWrite(LedPin, LOW);
  } while (1);
}

/* Setup ---------------------------------------------------------------------*/

void setup()
{
  VL53L5CX_DetectionThresholds thresholds[VL53L5CX_NB_THRESHOLDS];

  // Led.
  pinMode(LedPin, OUTPUT);

  // Enable PWREN pin if present
  if (PWREN_PIN >= 0) {
    pinMode(PWREN_PIN, OUTPUT);
    digitalWrite(PWREN_PIN, HIGH);
    delay(10);
  }

  // Set interrupt pin
  pinMode(INT_PIN, INPUT_PULLUP);
  attachInterrupt(INT_PIN, measure, FALLING);

  // Initialize serial for output.
  SerialPort.begin(115200);
  SerialPort.println("Initialize... Please wait, it may take few seconds...");

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Configure VL53L5CX satellite component.
  sensor_vl53l5cx_sat.begin();

  sensor_vl53l5cx_sat.init_sensor();

  /*********************************/
  /*  Program detection thresholds */
  /*********************************/

  /* In this example, we want 2 thresholds per zone for a 4x4 resolution */
  /* Create array of thresholds (size cannot be changed) */

  /* Set all values to 0 */
  memset(&thresholds, 0, sizeof(thresholds));

  /* Add thresholds for all zones (16 zones in resolution 4x4, or 64 in 8x8) */
  for (int i = 0; i < 16; i++) {
    /* The first wanted thresholds is GREATER_THAN mode. Please note that the
     * first one must always be set with a mathematic_operation
     * VL53L5CX_OPERATION_NONE.
     * For this example, the signal thresholds is set to 150 kcps/spads
     * (the format is automatically updated inside driver)
     */
    thresholds[2 * i].zone_num = i;
    thresholds[2 * i].measurement = VL53L5CX_SIGNAL_PER_SPAD_KCPS;
    thresholds[2 * i].type = VL53L5CX_GREATER_THAN_MAX_CHECKER;
    thresholds[2 * i].mathematic_operation = VL53L5CX_OPERATION_NONE;
    thresholds[2 * i].param_low_thresh = 150;
    thresholds[2 * i].param_high_thresh = 150;

    /* The second wanted checker is IN_WINDOW mode. We will set a
     * mathematical thresholds VL53L5CX_OPERATION_OR, to add the previous
     * checker to this one.
     * For this example, distance thresholds are set between 200mm and
     * 400mm (the format is automatically updated inside driver).
     */
    thresholds[2 * i + 1].zone_num = i;
    thresholds[2 * i + 1].measurement = VL53L5CX_DISTANCE_MM;
    thresholds[2 * i + 1].type = VL53L5CX_IN_WINDOW;
    thresholds[2 * i + 1].mathematic_operation = VL53L5CX_OPERATION_OR;
    thresholds[2 * i + 1].param_low_thresh = 200;
    thresholds[2 * i + 1].param_high_thresh = 400;
  }

  /* The last thresholds must be clearly indicated. As we have 32
   * checkers (16 zones x 2), the last one is the 31 */
  thresholds[31].zone_num = VL53L5CX_LAST_THRESHOLD | thresholds[31].zone_num;

  /* Send array of thresholds to the sensor */
  sensor_vl53l5cx_sat.vl53l5cx_set_detection_thresholds(thresholds);

  /* Enable detection thresholds */
  sensor_vl53l5cx_sat.vl53l5cx_set_detection_thresholds_enable(1);

  sensor_vl53l5cx_sat.vl53l5cx_set_ranging_frequency_hz(10);

  // Start Measurements
  sensor_vl53l5cx_sat.vl53l5cx_start_ranging();
}

void loop()
{
  static uint8_t loop_count = 0;
  VL53L5CX_ResultsData Results;
  char report[128];
  int status;

  if (loop_count < 100) {
    if (interruptCount) {
      interruptCount = 0;

      //Led on
      digitalWrite(LedPin, HIGH);

      status = sensor_vl53l5cx_sat.vl53l5cx_get_ranging_data(&Results);

      /* As the sensor is set in 4x4 mode by default, we have a total
       * of 16 zones to print.
       */

      snprintf(report, sizeof(report), "Print data no : %3u\r\n", sensor_vl53l5cx_sat.get_stream_count());
      SerialPort.print(report);
      for (int i = 0; i < 16; i++) {
        snprintf(report, sizeof(report), "Zone : %3d, Status : %3u, Distance : %4d mm, Signal : %5lu kcps/SPADs\r\n",
                 i,
                 Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE * i],
                 Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * i],
                 Results.signal_per_spad[VL53L5CX_NB_TARGET_PER_ZONE * i]);
        SerialPort.print(report);
      }
      SerialPort.println("");
      loop_count++;
      digitalWrite(LedPin, LOW);
    }
  } else if (loop_count == 100) {
    /* Stop measurements */
    status = sensor_vl53l5cx_sat.vl53l5cx_stop_ranging();
    if (status) {
      snprintf(report, sizeof(report), "vl53l5cx_stop_ranging failed, status %u\r\n", status);
      SerialPort.print(report);
      blink_led_loop();
    }

    loop_count++;
    /* End of the demo */
    SerialPort.println("End of ULD demo");
  } else {
    delay(1000);
  }
}
