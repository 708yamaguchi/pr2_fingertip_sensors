pr2_fingertip_sensors
=====================

The goal of this repository is to create a home-made sensor board to replace regular tactile sensors.

This repository will develop boards and firmware with the following features.
 - Develop a board that fits at the fingertips of PR2
 - In the board, connect microcontroller with many sensors (e.g. I2C proximity sensors, IMU, ADC force sensor, I2S microphone)
 - In the board, connect SPI slave microcontroller with PR2
 - Output `/pressure/l(r)_gripper_motor` rostopic instead of regular tactile sensor

# System

![](https://user-images.githubusercontent.com/19769486/173611179-30e323f4-dfa1-4e34-83eb-a86424cd380a.png)
*Fig. Development using an evaluation board before designing a board*

# Directories

- CubeIDE

  - Nucleo_G491RE

    Main project of this repository. Nucleo G491RE firmware for PR2 fingertip. SPI communication loop with DMA runs on FreeRTOS.

  - Nucleo_G491RE_PR2_slave

    Nucleo G491RE firmware communicating with PR2 as fingertip SPI slave.

  - Nucleo_G491RE_SPH0645LM4H

    Firmware to use SPH0645LM4H microphone with Nucleo G491RE. I2S communication with DMA runs on FreeRTOS.

  - Nucleo_G491RE_VCNL4040

    Firmware to use VCNL4040 proximity sensor with Nucleo G491RE. I2C communication with DMA runs on FreeRTOS.

  - Arduino_Nano_PR2_slave

    Arduino Nano firmware communicating with PR2 as fingertip SPI slave. This firmware is for reference only in this project and is not for actual use.

- Datasheets

  Datasheets of microcontrollers and sensors

# Documents

  For JSK users only

- [PR2 official documents](https://drive.google.com/drive/u/0/folders/10u_ev0fsHuU6k2bqzVA1QDX5yvDKZRMa)

- [Our progress 1](https://docs.google.com/presentation/d/1_63MSYOCmoeexlYo3aRt_9dm243HZhIHg_bWbS5awlA/edit?usp=sharing)

- [Our progress 2](https://docs.google.com/presentation/d/1VxRJWDqeDk_ryu-x1Vhj3_6BDu3gscwvNpngHKwfR4M/edit?usp=sharing)
