#ifndef __SENSOR_H
#define __SENSOR_H

#include "stm32g4xx_hal.h"

// COMMUNICATION const
static const uint8_t READ_COMMAND = 0x12;

// IMU selection
static const uint8_t SELECT_ICM_20600 = 0x00;
static const uint8_t SELECT_ICM_42605 = 0x01;
static const uint8_t SELECT_ICM_42688_SPI = 0x02;
static const uint8_t SELECT_ICM_42688_I2C = 0x03;

// ICM-20600 IMU REGISTER and ADDR
static const uint8_t ICM_20600_PWR_MGMT_1 =  0x6B;
static const uint8_t ICM_20600_CONFIG = 0x1A;
static const uint8_t ICM_20600_GYRO_CONFIG = 0x1B;
static const uint8_t ICM_20600_ACCEL_CONFIG = 0x1C;
static const uint8_t ICM_20600_ACCEL_CONFIG2 = 0x1D;
static const uint8_t ICM_20600_GYRO_XOUT_H =  0x43;
static const uint8_t ICM_20600_ACCEL_XOUT_H =  0x3B;
static const uint8_t ICM_20600_PING_ADDRESS =  0x75;

// ICM-20600 IMU CONFIG VAL
static const uint8_t ICM_20600_PWR_MGMT_1_VAL = 0x01;//auto selects the best available clock source
static const uint8_t ICM_20600_GYRO_DLPF_CFG = 0x01;//0x04 //0: 250Hz, 0.97ms; 3: 41Hz, 5.9ms(kduino); 4: 20Hz: 9.9ms
static const uint8_t ICM_20600_GYRO_CONFIG_VAL = 0x18;//GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
static const uint8_t ICM_20600_ACCEL_CONFIG_VAL = 0x18;//ACCEL_CONFIG  -- ACCEL_FS_SEL=3 (Full Scale = +/-16G)
static const uint8_t ICM_20600_ACC_DLPF_CFG = 0x03;

// ICM-42605 IMU REGISTER and ADDR
static const uint8_t ICM_42605_GYRO_CONFIG0 =  0x4F;
static const uint8_t ICM_42605_GYRO_CONFIG1 =  0x51;
static const uint8_t ICM_42605_ACCEL_CONFIG0 =  0x50;
static const uint8_t ICM_42605_ACCEL_CONFIG1 =  0x53;
static const uint8_t ICM_42605_GYRO_ACCEL_CONFIG0 =  0x52;
static const uint8_t ICM_42605_PWR_MGMT0 =  0x4E;
static const uint8_t ICM_42605_GYRO_DATA_X1 =  0x25;
static const uint8_t ICM_42605_ACCEL_DATA_X1 =  0x1F;
static const uint8_t ICM_42605_PING_ADDRESS =  0x75;

// ICM-42605 IMU CONFIG VAL
static const uint8_t ICM_42605_GYRO_CONFIG0_VAL =  0x06;//GYRO_FS_SEL = 0: Full scale set to 2000 deg/sec, 0x05: ODR = 2kHz, 0x06: ODR = 1kHz(default)
static const uint8_t ICM_42605_ACCEL_CONFIG0_VAL =  0x06;//ACCEL_FS_SEL = 0: Full scale set to +/-16G, 0x05: ODR = 2kHz, 0x06: ODR = 1kHz(default)
static const uint8_t ICM_42605_GYRO_ACCEL_CONFIG0_VAL = 0x00;//Setting for Bandwidth of LPF(acc,gyro), 0x00: ODR/2=500Hz, 0x11:max(400, ODR)/4=250Hz(defalut)
static const uint8_t ICM_42605_PWR_MGMT0_VAL =  0x0F;//Turn on gyro and acc with Low Noise Mode

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

// ICM-42688-P IMU CONFIG VAL
static const uint8_t ICM_42688_GYRO_CONFIG0_VAL =  0x06;//GYRO_FS_SEL = 0: Full scale set to 2000 deg/sec, 0x05: ODR = 2kHz, 0x06: ODR = 1kHz(default)
static const uint8_t ICM_42688_ACCEL_CONFIG0_VAL =  0x06;//ACCEL_FS_SEL = 0: Full scale set to +/-16G, 0x05: ODR = 2kHz, 0x06: ODR = 1kHz(default)
static const uint8_t ICM_42688_GYRO_ACCEL_CONFIG0_VAL = 0x00;//Setting for Bandwidth of LPF(acc,gyro), 0x00: ODR/2=500Hz, 0x11:max(400, ODR)/4=250Hz(defalut)
static const uint8_t ICM_42688_PWR_MGMT0_VAL =  0x0F;//Turn on gyro and acc with Low Noise Mode

// ICM-42688-P I2C VAL
static const uint8_t ICM_42688_I2C_ADDR = 0x68<<1;

// IMU CS PIN
#define IMU_CS_PORT GPIOD
#define IMU_CS_PIN GPIO_PIN_2

// IMU CONST
#define ACC_CHANNEL_NUM 3
#define GYRO_CHANNEL_NUM 3
static const uint8_t IMU_EN = 0x01;
static const uint8_t IMU_NOT_EN = 0x00;
static const uint8_t IMU_WHO_AM_I_20600 = 0x11;
static const uint8_t IMU_WHO_AM_I_42605 = 0x42;
static const uint8_t IMU_WHO_AM_I_42688 = 0x47;
#define IMU_SPI_MODE 0
#define IMU_I2C_MODE !IMU_SPI_MODE

// PS CMD
static const uint8_t PS_CONF1 = 0x03;
static const uint8_t PS_CONF3 = 0x04;
static const uint8_t PS_DATA_L = 0x08;
static const uint8_t ID_L = 0x0C;

// PS VAL
static const uint8_t ID_L_VAL = 0x86;
static const uint8_t ID_H_VAL = 0x01;
static const uint8_t PS_EN = 0x01;
static const uint8_t PS_NOT_EN = 0x00;

// PS ADDR
static const uint8_t VCNL4040_ADDR = 0x60<<1;
static const uint8_t PCA9547_ADDR = 0x70<<1; //1,1,1,0,A2,A1,A0 A2=A1=A0=0
static const uint8_t PCA9548_ADDR = 0x70<<1; //1,1,1,0,A2,A1,A0 A2=A1=A0=0
static const uint8_t PS_CHANNEL_ARRAY_PCA9458[4] = {0x01,0x08,0x10,0x80};//KJS-03-revA PCA9458 channel format ch0,3,4,7
static const uint8_t PS_CHANNEL_ARRAY_PCA9457[4] = {0x00,0x03,0x04,0x07};//KJS-03-revB PCA9457 channel format ch0,3,4,7
#define PS_CHANNEL_NUM 1

// ADC CONST
#define ADC_CHANNEL_NUM 4

// Buffer CONST
#define TXBUFF_LENGTH 44

#define SPI_SLAVE_SENSOR_EN 1
#define SPISLAVE_PERIOD 30//長くすると上手く行く場合がある

// I2S CONST
#define MIC_BUFF_SIZE 16
#define MIC_PERIOD 1000//長くすると上手く行く場合がある
#define MIC_CHANNEL_NUM 4

// Debug buffer
#define DEBUG_EN 1
#define UPDATE_SINGLE_THREAD 0
#define TIMER_SPISLAVE 0
#define SERIAL_PERIOD 1000

uint8_t debug_buffer[2048];
uint8_t gyro_buffer[512];
uint8_t acc_buffer[512];
uint8_t adc_buffer[512];
uint8_t i2s_buffer[512];
uint8_t ps_buffer[512];

struct sensor_params {
	//buffer
	uint8_t rxbuff[1];
	uint8_t txbuff[TXBUFF_LENGTH];

	// read write data
	uint8_t id;

	// write data
	uint8_t imu_select;
	uint8_t board_select;

	// read data
	uint8_t ps[PS_CHANNEL_NUM * 2];//0H,0L,1H,1L,4H,4L,7H,7L
	uint16_t ps_print[PS_CHANNEL_NUM];
	uint8_t ps_en[PS_CHANNEL_NUM]; //0x00:disable 0x01:enable
	uint16_t ps_elapsed_time;
	uint8_t gyro[GYRO_CHANNEL_NUM * 2];
	int16_t gyro_print[GYRO_CHANNEL_NUM];
	uint8_t acc[ACC_CHANNEL_NUM * 2];
	int16_t acc_print[ACC_CHANNEL_NUM];
	uint8_t gyro_acc[GYRO_CHANNEL_NUM * 2 + ACC_CHANNEL_NUM * 2];
	uint16_t imu_elapsed_time;
	uint8_t imu_en; //0x00:disable 0x01:enable
	uint8_t imu_dma_en; //0x00:disable 0x01:enable
	uint8_t adc[ADC_CHANNEL_NUM * 2];
	uint16_t adc_print[ADC_CHANNEL_NUM];
	uint16_t adc_elapsed_time;
	int32_t i2s_rx_buff[MIC_BUFF_SIZE];
	int32_t i2s_buff_sifted[MIC_BUFF_SIZE];
	uint16_t mic_elapsed_time;

	// control
	uint8_t com_en;

	uint8_t count;
	uint8_t flag;//0:imu, 1:mic
	uint16_t error_count;
	uint16_t rx_counter;
	uint8_t i2c1_dma_flag;
};

extern struct sensor_params sp;

#endif



