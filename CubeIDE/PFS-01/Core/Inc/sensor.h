#ifndef __SENSOR_H
#define __SENSOR_H

#include "stm32g4xx_hal.h"

// COMMUNICATION const
static const uint8_t READ_COMMAND = 0x12;

//BOARD selection
static const uint8_t SELECT_PFS_01_SINGLE = 0x00;
static const uint8_t SELECT_PFS_01_ASM = 0x01;

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
static const uint8_t ICM_42688_GYRO_CONFIG0_VAL_32khz =  0x01;//GYRO_FS_SEL = 0: Full scale set to 2000 deg/sec, 0x01: ODR = 32kHz
static const uint8_t ICM_42688_GYRO_CONFIG0_VAL_4khz =  0x04;//GYRO_FS_SEL = 0: Full scale set to 2000 deg/sec, 0x04: ODR = 4kHz
static const uint8_t ICM_42688_GYRO_CONFIG0_VAL_1khz =  0x06;//GYRO_FS_SEL = 0: Full scale set to 2000 deg/sec, 0x06: ODR = 1kHz(default)
static const uint8_t ICM_42688_GYRO_CONFIG0_VAL_100hz =  0x08;//GYRO_FS_SEL = 0: Full scale set to 2000 deg/sec, 0x08: ODR = 100Hz
static const uint8_t ICM_42688_GYRO_CONFIG0_VAL_12_5hz =  0x0b;//GYRO_FS_SEL = 0: Full scale set to 2000 deg/sec, 0x08: ODR = 12.5Hz
static const uint8_t ICM_42688_ACCEL_CONFIG0_VAL_32khz =  0x01;//ACCEL_FS_SEL = 0: Full scale set to +/-16G, 0x04: ODR = 4kHz
static const uint8_t ICM_42688_ACCEL_CONFIG0_VAL_4khz =  0x04;//ACCEL_FS_SEL = 0: Full scale set to +/-16G, 0x04: ODR = 4kHz
static const uint8_t ICM_42688_ACCEL_CONFIG0_VAL_1khz =  0x06;//ACCEL_FS_SEL = 0: Full scale set to +/-16G, 0x06: ODR = 1kHz(default)
static const uint8_t ICM_42688_ACCEL_CONFIG0_VAL_100hz =  0x08;//ACCEL_FS_SEL = 0: Full scale set to +/-16G, 0x08 ODR = 100Hz
static const uint8_t ICM_42688_ACCEL_CONFIG0_VAL_12_5hz =  0x0b;//ACCEL_FS_SEL = 0: Full scale set to +/-16G, 0x08 ODR = 100Hz
static const uint8_t ICM_42688_GYRO_ACCEL_CONFIG0_VAL = 0x00;//Setting for Bandwidth of LPF(acc,gyro), 0x00: ODR/2=500Hz, 0x11:max(400, ODR)/4=250Hz(defalut)
static const uint8_t ICM_42688_PWR_MGMT0_VAL =  0x0F;//Turn on gyro and acc with Low Noise Mode

// ICM-42688-P I2C VAL
static const uint8_t ICM_42688_I2C_ADDR = 0x68<<1;

// IMU CONST
#define ACC_CHANNEL_NUM 3
#define GYRO_CHANNEL_NUM 3
static const uint8_t IMU_EN = 0x01;
static const uint8_t IMU_NOT_EN = 0x00;
static const uint8_t IMU_WHO_AM_I_20600 = 0x11;
static const uint8_t IMU_WHO_AM_I_42605 = 0x42;
static const uint8_t IMU_WHO_AM_I_42688 = 0x47;
#define IMU_SPI_MODE 1
#define IMU_I2C_MODE !IMU_SPI_MODE
#define IMU_I2C_DMA 0
#define IMU_GYRO_ACC_DELAY 5

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
#define PS_CHANNEL_NUM 8
#define PCA9547_NUM 5
static const uint8_t VCNL4040_ADDR = 0x60<<1;
static const uint8_t PCA9547_ADDR = 0x74<<1; //1,1,1,0,A2,A1,A0 A2=1, A1=A0=0
static const uint8_t PS_CHANNEL_ARRAY_PCA9457[PS_CHANNEL_NUM] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07};//PCA9457 channel format ch0,1,2,3,4,5,6,7

static const uint8_t PCA9547_ADDR_ARRAY[PCA9547_NUM] = {0x74<<1, 0x70<<1, 0x71<<1, 0x72<<1, 0x73<<1};
//PFS-01A       : 1,1,1,0,A2,A1,A0 A2=1, A1=0, A0=0
//PFS-01B(Right): 1,1,1,0,A2,A1,A0 A2=0, A1=0, A0=0
//PFS-01A(Left) : 1,1,1,0,A2,A1,A0 A2=0, A1=0, A0=1
//PFS-01A(Front): 1,1,1,0,A2,A1,A0 A2=0, A1=1, A0=0
//PFS-01A(Top)  : 1,1,1,0,A2,A1,A0 A2=0, A1=1, A0=1
static const uint8_t PS_CHANNEL_NUM_ARRAY[PCA9547_NUM] = {8,4,4,4,4};

static const uint8_t PS_CHANNEL_2DARRAY_PCA9457[PCA9547_NUM][PS_CHANNEL_NUM] =
{
		{0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07},
		{0x00,0x02,0x05,0x07,0x10,0x10,0x10,0x10},
		{0x00,0x02,0x05,0x07,0x10,0x10,0x10,0x10},
		{0x00,0x02,0x05,0x07,0x10,0x10,0x10,0x10},
		{0x00,0x02,0x05,0x07,0x10,0x10,0x10,0x10}
};//PCA9457 channel format

#define PS_I2C_DMA 0

// ADC CONST
#define ADC_CHANNEL_NUM 9
#define FS_CHANNEL_NUM 8 //force sensor

// ADS7828 ADDR
#define ADC_CHANNEL_NUM_ADS 4
#define ADS7828_NUM 4
static const uint8_t ADC_CHANNEL_ARRAY[ADC_CHANNEL_NUM_ADS] = {0x80,0xC0,0xB0,0xF0};//ADS7828 channel format ch0,1,6,7

static const uint8_t ADS7828_ADDR_ARRAY[ADS7828_NUM] = {0x48<<1, 0x49<<1, 0x4A<<1, 0x4B<<1};
//PFS-01B(Right): 1,0,0,1,0,A1,A0 A1=0, A0=0
//PFS-01A(Left) : 1,0,0,1,0,A1,A0 A1=0, A0=1
//PFS-01A(Front): 1,0,0,1,0,A1,A0 A1=1, A0=0
//PFS-01A(Top)  : 1,0,0,1,0,A1,A0 A1=1, A0=1

// sensor CONST
#define MAX_PS_SENSOR_NUM (PS_CHANNEL_NUM + (PCA9547_NUM - 1) * (PS_CHANNEL_NUM - 4))
#define MAX_FS_SENSOR_NUM (FS_CHANNEL_NUM + ADS7828_NUM * ADC_CHANNEL_NUM_ADS)

// Buffer CONST
#define TXBUFF_LENGTH 44
#define SERIAL_PUBLISH_LENGTH (MAX_PS_SENSOR_NUM + MAX_FS_SENSOR_NUM + ACC_CHANNEL_NUM + GYRO_CHANNEL_NUM)

#define SPI_SLAVE_SENSOR_EN 1
#define SPISLAVE_PERIOD 30//長くすると上手く行く場合がある

// I2S CONST
#define MIC_BUFF_SIZE 1024
//#define MIC_PERIOD (20 + 20)//長くすると上手く行く場合がある
#define MIC_PERIOD (20 + 20)//1000 / () [ms]
#define MIC_CHANNEL_NUM 4

// Debug buffer
#define DEBUG_EN 0
#define UPDATE_SINGLE_THREAD 0
#define TIMER_SPISLAVE 0
#define SERIAL_PERIOD 1000

// DMA CONST
#define I2C1_DMA_GYRO 0
#define I2C1_DMA_ACC 1
#define I2C1_DMA_PS 2
#define SPI_SLAVE_DMA 1
#define MIC_DMA 0
#define MIC_TIMER 1

// MAIN SPI FLAG
#define SPI_SLAVE 1
#define SPI_SLAVE_STATENUM 2

// SLAVE MODE
#define PR2_SPI_SLAVE 0
#define UART_SLAVE 1

uint8_t debug_buffer[2048];
uint8_t gyro_buffer[512];
uint8_t acc_buffer[512];
uint8_t adc_buffer[512];
uint8_t i2s_buffer[512];
uint8_t ps_buffer[512];

// acc_flag == 1 -> measure acc
// acc_flag == 0 -> measure gyro
uint8_t acc_flag;

struct sensor_params {
	//buffer
	uint8_t rxbuff[1];
	uint8_t txbuff[TXBUFF_LENGTH];
	uint8_t txbuff_state[SPI_SLAVE_STATENUM][TXBUFF_LENGTH];
//	uint8_t txbuff_state_flatten[TXBUFF_LENGTH * 2];
	int16_t serial_publish_flatten[SERIAL_PUBLISH_LENGTH];

	// read write data
	uint8_t id;

	// write data
	uint8_t imu_select;
	uint8_t board_select;

	// read data
	uint8_t ps[PS_CHANNEL_NUM * 2];
	uint16_t ps_print[PS_CHANNEL_NUM];
	uint16_t ps_print_raw[PS_CHANNEL_NUM];
	uint8_t ps_2d[PCA9547_NUM][PS_CHANNEL_NUM * 2];
	uint16_t ps_print_2d[PCA9547_NUM][PS_CHANNEL_NUM];
	uint16_t ps_print_flatten[MAX_PS_SENSOR_NUM];//PFS-01A:8 * 1, PFS-01B:4 * 4
	uint8_t ps_en[PS_CHANNEL_NUM]; //0x00:disable 0x01:enable
	uint8_t ps_en_2d[PCA9547_NUM][PS_CHANNEL_NUM]; //0x00:disable 0x01:enable
	uint16_t ps_elapsed_time;
	uint8_t ps_dma[2];
	uint8_t gyro[GYRO_CHANNEL_NUM * 2];
	int16_t gyro_print[GYRO_CHANNEL_NUM];
	uint8_t acc[ACC_CHANNEL_NUM * 2];
	int16_t acc_print[ACC_CHANNEL_NUM];
	uint8_t gyro_acc[GYRO_CHANNEL_NUM * 2 + ACC_CHANNEL_NUM * 2];
	uint16_t imu_elapsed_time;
	uint8_t imu_en; //0x00:disable 0x01:enable
	uint8_t imu_dma_en; //0x00:disable 0x01:enable
	uint32_t imu_prev_frame;
	uint32_t imu_count_frame;
	uint8_t adc[ADC_CHANNEL_NUM * 2];
	uint16_t adc_print[ADC_CHANNEL_NUM];
	uint16_t adc_print_raw[ADC_CHANNEL_NUM];
	uint16_t adc_elapsed_time;
	uint8_t adc_ADS_2d[ADS7828_NUM][ADC_CHANNEL_NUM_ADS * 2];
	uint16_t adc_print_ADS_2d[ADS7828_NUM][ADC_CHANNEL_NUM_ADS];
	uint16_t adc_print_flatten[MAX_FS_SENSOR_NUM];//PFS-01A:8 * 1, PFS-01B:4 * 4
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

	uint8_t spi_slave_flag;
	uint8_t slave_mode;
};

extern struct sensor_params sp;

#endif



