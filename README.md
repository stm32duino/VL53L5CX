# VL53L5CX
Arduino library to support the VL53L5CX Time-of-Flight 8x8 multizone ranging sensor with wide field view.

## API

This sensor uses I2C to communicate. And I2C instance is required to access to the sensor.
The APIs provide simple distance measure and multizone detection in both polling and interrupt modes.

## Examples

The examples contained in this library are based on VL53L5CX-SATEL sensor board. You need to connect the VL53L5CX-SATEL sensor board directly to the Nucleo board with wires as explained below:
- pin 1 (GND) of the VL53L5CX satellite connected to GND of the Nucleo board
- pin 2 (IOVDD) of the VL53L5CX satellite connected to 3V3 pin of the Nucleo board
- pin 3 (AVDD) of the VL53L5CX satellite connected to 5V pin of the Nucleo board
- pin 4 (PWREN) of the VL53L5CX satellite connected to pin A5 of the Nucleo board
- pin 5 (LPn) of the VL53L5CX satellite connected to pin A3 of the Nucleo board
- pin 6 (SCL) of the VL53L5CX satellite connected to pin D15 (SCL) of the Nucleo board
- pin 7 (SDA) of the VL53L5CX satellite connected to pin D14 (SDA) of the Nucleo board
- pin 8 (I2C_RST) of the VL53L5CX satellite connected to pin A1 of the Nucleo board
- pin 9 (INT) of the VL53L5CX satellite connected to pin A2 of the Nucleo board 

There are 11 examples with the VL53L5CX library:

* VL53L5CX_Sat_Calibrate_Xtalk: This example code is to show how to perform the crosstalk calibration.

* VL53L5CX_Sat_Get_Set_Params: This example code is to show how to get/set some parameters of the 
  VL53L5CX sensor.

* VL53L5CX_Sat_HelloWorld: This example code is to show how to get multizone detection and proximity
  values of the VL53L5CX satellite sensor in polling mode.

* VL53L5CX_Sat_HelloWorld_Interrupt: This example code is to show how to get multizone detection and proximity
  values of the VL53L5CX satellite sensor in interrupt mode.

* VL53L5CX_Sat_I2C_And_RAM_Optimization: This example code is to show how to optimize the code in terms of 
  number of I2C transactions and RAM occupation.

* VL53L5CX_Sat_Motion_Indicator: This example code is to show how to configure the motion indicator.

* VL53L5CX_Sat_Motion_Indicator_With_Thresholds_Detection: This example code is to show how to configure 
  the motion indicator together with the thresholds detection.
  
* VL53L5CX_Sat_Multiple_Targets_Per_Zone: This example code is to show how to configure multiple targets 
  per zone.

* VL53L5CX_Sat_Power_Modes: This example code is to show how to change the power mode of the VL53L5CX 
  sensor.

* VL53L5CX_Sat_Ranging_Modes: This example code is to show how to change the ranging mode of the VL53L5CX 
  sensor.

* VL53L5CX_Sat_Thresholds_Detection: This example code is to show how to configure the thresholds 
  detection.

## Documentation

You can find the source files at
https://github.com/stm32duino/VL53L5CX

The VL53L5CX datasheet is available at
https://www.st.com/en/imaging-and-photonics-solutions/vl53l5cx.html
