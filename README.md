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

# Usage

- Create ROS workspace

  ```
  mkdir ~/pr2_fingertip_ws/src -p
  cd ~/pr2_fingertip_ws/src
  git clone https://github.com/708yamaguchi/pr2_fingertip_sensors
  cd ..
  rosdep install --from-paths src --ignore-src -y -r
  catkin build
  ```

  When you use programs in this package, do not forget to set ROS_MASTER_URI and source this workspace

  ```
  ### Set ROS_MASTER_URI to your PR2, like rossetmaster pr1012 ###

  # Source this workspace
  source ~/pr2_fingertip_ws/devel/setup.bash
  ```

- Write `PFS-01` project to main board (PFS-01A)

- Calibrate sensors

  Launch nodes for calibration

  ```
  roslaunch pr2_fingertip_sensors calibrate_pfs.launch
  ```

  Calibrate proximity parameters 'b' in I = (a / d^2) + b. Run the following command when nothing is near the PFS finger.

  ```
  rosservice call /pfs/no_object "{}"
  ```

  Calibrate proximity parameters 'a' in I = (a / d^2) + b. Run the following command after wrapping the PFS finger with white conver. This command must be called after the above command.

  ```
  rosservice call /pfs/near_object "{}"
  ```

  Calibrate force sensor preload. Run the following command when nothing touches the PFS finger.

  ```
  rosservice call /pfs/preload "{}"
  ```

  Dump calibration parameters under pr2_fingertip_sensors/data/pfs_params.yaml

  ```
  rosservice call /pfs/dump_pfs_params "{}"
  ```

- Run parser and visualizer node for PFS sensor data. In PFS-01 project, two rostopics are combined to represent the whole sensor data.

  ```
  roslaunch pr2_fingertip_sensors pfs.launch
  ```

- Subscribe sensor data

  ```
  rostopic echo /pfs/l_gripper/l_fingertip
  rostopic echo /pfs/l_gripper/r_fingertip
  rostopic echo /pfs/r_gripper/l_fingertip
  rostopic echo /pfs/r_gripper/r_fingertip
  ```

- View sensor data with sample rosbag

  ```
  roslaunch pr2_fingertip_sensors sample_pfs.launch
  ```

# Directories

- CubeIDE

  - PFS-01

    A main project of this repository. STM32 G491RE firmware for PR2 fingertip.

    PFS-01: First revision of PR2 Finger Sensorboard.

    	    develop-PFS-01A: main branch for developping PFS-01 (now only test PFS-01A)

  - Nucleo_G491RE_DMA

    A project used in final verification of sensor behavior with Nucleo G491RE. All communication except ADC is implemented by DMA.

  - Nucleo_G491RE

    A project that has been in development for a long time. Various communication types can be tested with the define flag.

  - Nucleo_G491RE_PR2_slave

    Nucleo G491RE firmware communicating with PR2 as fingertip SPI slave.

  - Nucleo_G491RE_SPH0645LM4H

    Firmware to use SPH0645LM4H microphone with Nucleo G491RE. I2S communication with DMA runs on FreeRTOS.

  - Nucleo_G491RE_VCNL4040

    Firmware to use VCNL4040 proximity sensor with Nucleo G491RE. I2C communication with DMA runs on FreeRTOS.

  - Arduino_Nano_PR2_slave

    Arduino Nano firmware communicating with PR2 as fingertip SPI slave. This firmware is for reference only in this project and is not for actual use.

  - KJS-03

    A sample project of sensor boards using a STM32 micro microcontroller.

    KJS-03: Third revision of Kondo Jointbase Sensorboard.

- Datasheets

  Datasheets of microcontrollers and sensors

# Documents

  For JSK users only

- [PR2 official documents](https://drive.google.com/drive/u/0/folders/10u_ev0fsHuU6k2bqzVA1QDX5yvDKZRMa)

- [Our progress 1](https://docs.google.com/presentation/d/1_63MSYOCmoeexlYo3aRt_9dm243HZhIHg_bWbS5awlA/edit?usp=sharing)

- [Our progress 2](https://docs.google.com/presentation/d/1VxRJWDqeDk_ryu-x1Vhj3_6BDu3gscwvNpngHKwfR4M/edit?usp=sharing)

- [BOM](https://drive.google.com/file/d/1cyWvVVvMDZyYSMVXUAKyK9CF-KghAjUk/view)

# Notes

  - Why G491RE?
    - STM32 specifications require a wait time between SPI transmission and reception. The wait time depends on the clock frequency.
    - According to [PR2 official documents](https://drive.google.com/drive/u/0/folders/10u_ev0fsHuU6k2bqzVA1QDX5yvDKZRMa), the minimum time between transmission and reception of SPI communication in PR2 is 4us.
    - The clock frequency of the STM32 that satisfies this requirement is more than 180 MHz by simple calculation.
    - Reference https://zenn.dev/nanase_t/articles/0280ef51007267
    - We have verified that the Nucleo G491RE, operating at up to 170 MHz, can work as an SPI slave.
    - The other option is the H743ZI2, but we chose the G491RE which has a smaller footprint.
  - Current consumption constraints
    - According to [PR2 official documents](https://drive.google.com/drive/u/0/folders/10u_ev0fsHuU6k2bqzVA1QDX5yvDKZRMa), current consumption should be kept within 30 mA total for both fingers.
    - Minimum power consumption mode for [VCNL4040 proximity sensor](https://github.com/708yamaguchi/pr2_fingertip_sensors/blob/master/Datasheets/vcnl4040.pdf) requires 50mA peak. However, when the on/off ratio of the IR LEDs was set to 1:320, the power supplied by PR2 was found to be sufficient.
  - CPU resource limitations
    - The board developed in this project must function as an SPI slave for PR2.
    - Responding to every data request from SPI master (PR2) makes it difficult for the microcontroller to read other sensors at the appropriate timing.
    - To save microcontroller CPU resources as much as possible, the DMA function is effective.

# Design of PCB
For JSK users only
  - Sensor board
    - PFS-01A

    A main circuit board of PFS-01.

    PFS-01: First revision of PR2 Finger Sensorboard.

    [PCB project](https://drive.google.com/drive/u/1/folders/1ek_gkk0nL_mesC0ZjxZoTLl6uysGZHwz)

    - PFS-01B

    Front, side, top circuit boards of PFS-01.

  - Conversion board
    - ICSC-01
    
    ICSC-01: First revision of ICS Conversionboard.

    [PCB project](https://drive.google.com/drive/u/1/folders/1huTPeMoCkxRJL4r8LULCaVruhAkjkXQm)

    A conversion board of ICS(12V) to ICS(5V).

    Function1:step down ICS VCC(12V) to ICS VCC(5V).

    Function2:connect UART_TX and UART_RX via 2.2k resistor [Document P9](https://kondo-robot.com/w/wp-content/uploads/ICS3.5_SoftwareManual_1_1.pdf).
