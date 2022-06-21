/**
 ******************************************************************************
 * @file    VL53L5CX_Sat_Calibrate_Xtalk.ino
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

// Put it to true if you want to display Xtalk data
bool display_xtalk_data = false;

// Components.
VL53L5CX sensor_vl53l5cx_sat(&DEV_I2C, LPN_PIN, I2C_RST_PIN);

void blink_led_loop(void);

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
  char report[64];
  uint8_t status;
  /* Buffer containing Xtalk data */
  uint8_t xtalk_data[VL53L5CX_XTALK_BUFFER_SIZE];

  // Led.
  pinMode(LedPin, OUTPUT);

  // Enable PWREN pin if present
  if (PWREN_PIN >= 0) {
    pinMode(PWREN_PIN, OUTPUT);
    digitalWrite(PWREN_PIN, HIGH);
    delay(10);
  }

  // Initialize serial for output.
  SerialPort.begin(115200);
  SerialPort.println("Initialize... Please wait, it may take few seconds...");

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Configure VL53L5CX satellite component.
  sensor_vl53l5cx_sat.begin();

  sensor_vl53l5cx_sat.init_sensor();

  /*********************************/
  /*    Start Xtalk calibration    */
  /*********************************/

  /* Start Xtalk calibration with a 3% reflective target at 600mm for the
   * sensor, using 4 samples.
   */
  SerialPort.println("Running Xtalk calibration...");

  status = sensor_vl53l5cx_sat.vl53l5cx_calibrate_xtalk(3, 4, 600);
  if (status) {
    snprintf(report, sizeof(report), "vl53l5cx_calibrate_xtalk failed, status %u\r\n", status);
    SerialPort.print(report);
    blink_led_loop();
  } else {
    SerialPort.println("Xtalk calibration done");

    /* Get Xtalk calibration data, in order to use them later */
    status = sensor_vl53l5cx_sat.vl53l5cx_get_caldata_xtalk(xtalk_data);

    if (status) {
      snprintf(report, sizeof(report), "vl53l5cx_get_caldata_xtalk failed, status %u\r\n", status);
      SerialPort.print(report);
      blink_led_loop();
    }

    /* Set Xtalk calibration data */
    status = sensor_vl53l5cx_sat.vl53l5cx_set_caldata_xtalk(xtalk_data);

    if (status) {
      snprintf(report, sizeof(report), "vl53l5cx_set_caldata_xtalk failed, status %u\r\n", status);
      SerialPort.print(report);
      blink_led_loop();
    }
  }

  if (display_xtalk_data) {
    /* Visualize Xtalk grid and Xtalk shape */
    uint32_t i, j;
    union Block_header *bh_ptr;
    uint32_t xtalk_signal_kcps_grid[VL53L5CX_RESOLUTION_8X8];
    uint16_t xtalk_shape_bins[144];

    /* Swap buffer */
    sensor_vl53l5cx_sat.SwapBuffer(xtalk_data, VL53L5CX_XTALK_BUFFER_SIZE);

    /* Get data */
    for (i = 0; i < VL53L5CX_XTALK_BUFFER_SIZE; i = i + 4) {
      bh_ptr = (union Block_header *) & (xtalk_data[i]);
      if (bh_ptr->idx == 0xA128) {
        snprintf(report, sizeof(report), "Xtalk shape bins located at position %#06x\r\n", (unsigned int)i);
        SerialPort.print(report);
        for (j = 0; j < 144; j++) {
          memcpy(&(xtalk_shape_bins[j]), &(xtalk_data[i + 4 + j * 2]), 2);
          snprintf(report, sizeof(report), "xtalk_shape_bins[%d] = %u\r\n", (int)j, xtalk_shape_bins[j]);
          SerialPort.print(report);
        }
      }
      if (bh_ptr->idx == 0x9FFC) {
        snprintf(report, sizeof(report), "Xtalk signal kcps located at position %#06x\r\n", (unsigned int)i);
        SerialPort.print(report);
        for (j = 0; j < VL53L5CX_RESOLUTION_8X8; j++) {
          memcpy(&(xtalk_signal_kcps_grid[j]), &(xtalk_data[i + 4 + j * 4]), 4);
          xtalk_signal_kcps_grid[j] /= 2048;
          snprintf(report, sizeof(report), "xtalk_signal_kcps_grid[%d] = %d\r\n", (int)j, (int)xtalk_signal_kcps_grid[j]);
          SerialPort.print(report);
        }
      }
    }

    /* Re-Swap buffer (in case of re-using data later) */
    sensor_vl53l5cx_sat.SwapBuffer(xtalk_data, VL53L5CX_XTALK_BUFFER_SIZE);
  }

  // Start Measurements
  sensor_vl53l5cx_sat.vl53l5cx_start_ranging();
}

void loop()
{
  static uint8_t loop_count = 0;
  VL53L5CX_ResultsData Results;
  uint8_t NewDataReady = 0;
  char report[64];
  uint8_t status;

  if (loop_count < 10) {

    do {
      status = sensor_vl53l5cx_sat.vl53l5cx_check_data_ready(&NewDataReady);
    } while (!NewDataReady);

    //Led on
    digitalWrite(LedPin, HIGH);

    if ((!status) && (NewDataReady != 0)) {
      status = sensor_vl53l5cx_sat.vl53l5cx_get_ranging_data(&Results);

      /* As the sensor is set in 4x4 mode by default, we have a total
       * of 16 zones to print.
       */

      snprintf(report, sizeof(report), "Print data no : %3u\r\n", sensor_vl53l5cx_sat.get_stream_count());
      SerialPort.print(report);
      for (int i = 0; i < 16; i++) {
        snprintf(report, sizeof(report), "Zone : %3d, Status : %3u, Distance : %4d mm\r\n",
                 i,
                 Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE * i],
                 Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * i]);
        SerialPort.print(report);
      }
      SerialPort.println("");
      loop_count++;
    }

    digitalWrite(LedPin, LOW);
  } else if (loop_count == 10) {
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
