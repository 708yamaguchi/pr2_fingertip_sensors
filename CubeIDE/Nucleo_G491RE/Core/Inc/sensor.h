#ifndef __SENSOR_H
#define __SENSOR_H

#include "stm32g4xx_hal.h"

// ICM-42688-P IMU REGISTER and ADDR
static const uint8_t ICM_42688_GYRO_CONFIG0 =  0x4F;
static const uint8_t ICM_42688_GYRO_CONFIG1 =  0x51;
static const uint8_t ICM_42688_ACCEL_CONFIG0 =  0x50;
static const uint8_t ICM_42688_ACCEL_CONFIG1 =  0x53;
static const uint8_t ICM_42688_GYRO_ACCEL_CONFIG0 =  0x52;
static const uint8_t ICM_42688_PWR_MGMT0 =  0x4E;
static const uint8_t ICM_42688_GYRO_DATA_X1 =  0x25;
static const uint8_t ICM_42688_ACCEL_DATA_X1 =  0x1F;
static const uint8_t ICM_42688_PING_ADDRESS =  0x75;

// ICM-42688 IMU CONFIG VAL
static const uint8_t ICM_42688_GYRO_CONFIG0_VAL =  0x06;//GYRO_FS_SEL = 0: Full scale set to 2000 deg/sec, 0x05: ODR = 2kHz, 0x06: ODR = 1kHz(default)
static const uint8_t ICM_42688_ACCEL_CONFIG0_VAL =  0x06;//ACCEL_FS_SEL = 0: Full scale set to +/-16G, 0x05: ODR = 2kHz, 0x06: ODR = 1kHz(default)
static const uint8_t ICM_42688_GYRO_ACCEL_CONFIG0_VAL = 0x00;//Setting for Bandwidth of LPF(acc,gyro), 0x00: ODR/2=500Hz, 0x11:max(400, ODR)/4=250Hz(defalut)
static const uint8_t ICM_42688_PWR_MGMT0_VAL =  0x0F;//Turn on gyro and acc with Low Noise Mode

// IMU CS PIN
#define IMU_CS_PORT GPIOB
#define IMU_CS_PIN GPIO_PIN_0

// IMU CONST
#define ACC_CHANNEL_NUM 3
#define GYRO_CHANNEL_NUM 3
static const uint8_t IMU_EN = 0x01;
static const uint8_t IMU_NOT_EN = 0x00;

// ADC CONST
#define ADC_CHANNEL_NUM 4
static const uint8_t ADC_CHANNEL_ARRAY[4] = {0,2,3,4};
#define ADC_GETCHANNEL_NUM 4

#endif



