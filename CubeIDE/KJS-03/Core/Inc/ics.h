#ifndef __ICS_H
#define __ICS_H

#include "stm32f4xx_hal.h"

// SERVO CMD
#define CMD_POS 0x80
#define CMD_RD 0xa0
#define CMD_WR 0xc0
#define CMD_ID 0xe0

// SERVO SC
#define SC_EEPROM 0x00
#define SC_STRC 0x01
#define SC_SPD 0x02
#define SC_CUR 0x03
#define SC_TMP 0x04
#define SC_TCH 0x05

// SENSOR SC
#define SC_NONE 0x60
#define SC_SW_ADC 0x61
#define SC_PS  0x62
#define SC_GYRO 0x64
#define SC_ACC 0x68
#define SC_ALL 0x6F

// IMU selection
static const uint8_t SELECT_ICM_20600 = 0x00;
static const uint8_t SELECT_ICM_42605 = 0x01;

// Board selection
static const uint8_t SELECT_revA = 0x00;
static const uint8_t SELECT_revB = 0x01;

// ICM-20600 IMU REGISTER and ADDR
static const uint8_t ICM_20600_PWR_MGMT_1 =  0x6B;
static const uint8_t ICM_20600_CONFIG = 0x1A;
static const uint8_t ICM_20600_GYRO_CONFIG = 0x1B;
static const uint8_t ICM_20600_ACCEL_CONFIG = 0x1C;
static const uint8_t ICM_20600_ACCEL_CONFIG2 = 0x1D;
static const uint8_t ICM_20600_GYRO_ADDRESS =  0x43;
static const uint8_t ICM_20600_ACC_ADDRESS =  0x3B;
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
static const uint8_t ICM_42605_GYRO_ADDRESS =  0x25;
static const uint8_t ICM_42605_ACC_ADDRESS =  0x1F;
static const uint8_t ICM_42605_PING_ADDRESS =  0x75;

// ICM-42605 IMU CONFIG VAL
static const uint8_t ICM_42605_GYRO_CONFIG0_VAL =  0x06;//GYRO_FS_SEL = 0: Full scale set to 2000 deg/sec, 0x05: ODR = 2kHz, 0x06: ODR = 1kHz(default)
static const uint8_t ICM_42605_ACCEL_CONFIG0_VAL =  0x06;//ACCEL_FS_SEL = 0: Full scale set to +/-16G, 0x05: ODR = 2kHz, 0x06: ODR = 1kHz(default)
static const uint8_t ICM_42605_GYRO_ACCEL_CONFIG0_VAL = 0x00;//Setting for Bandwidth of LPF(acc,gyro), 0x00: ODR/2=500Hz, 0x11:max(400, ODR)/4=250Hz(defalut)
static const uint8_t ICM_42605_PWR_MGMT0_VAL =  0x0F;//Turn on gyro and acc with Low Noise Mode

// IMU CS PIN
#define IMU_CS_PORT GPIOB
#define IMU_CS_PIN GPIO_PIN_0

// IMU CONST
#define ACC_CHANNEL_NUM 3
#define GYRO_CHANNEL_NUM 3
static const uint8_t IMU_EN = 0x01;
static const uint8_t IMU_NOT_EN = 0x00;
static const uint8_t IMU_WHO_AM_I_20600 = 0x11;
static const uint8_t IMU_WHO_AM_I_42605 = 0x42;

// LED PIN
#define LED_PORT GPIOC
#define LED_PIN GPIO_PIN_13

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
//static const uint8_t PS_CHANNEL_ARRAY[4] = {0x00,0x01,0x06,0x07};//KJS-01 PCA9457 channel format
static const uint8_t PS_CHANNEL_ARRAY_PCA9458[4] = {0x01,0x08,0x10,0x80};//KJS-03-revA PCA9458 channel format ch0,3,4,7
static const uint8_t PS_CHANNEL_ARRAY_PCA9457[4] = {0x00,0x03,0x04,0x07};//KJS-03-revB PCA9457 channel format ch0,3,4,7
#define PS_CHANNEL_NUM 4

// ADC CONST
#define ADC_CHANNEL_NUM 4
//static const uint8_t ADC_CHANNEL_ARRAY[4] = {0,6,1,2};//�ꏊ���m�F���ċL��
static const uint8_t ADC_CHANNEL_ARRAY[4] = {0,2,3,4};//�ꏊ���m�F���ċL��
#define ADC_GETCHANNEL_NUM 4
//#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  4)
//static uint16_t   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];

//COMMUNICATION CONST
#define MAX_TXBUFF_LEN (2 + 2 * ADC_CHANNEL_NUM + 3 * PS_CHANNEL_NUM + 3 * GYRO_CHANNEL_NUM + 3 * ACC_CHANNEL_NUM)

#define USE_SEMIHOSTING 0

//Grove CONST
#define USE_GROVE_I2C 0
#define M5STICKV_MAX_NUM 4
#define M5STICKV_DETECTION_RESULT_LENGTH 254

//TIMER CONST
#define LED_TOGGLE_TIME 100000 //100000[ns] = 100[ms]
//#define UART_TIMEOUT_TIME 1000000 //1000000[us] = 1000[ms]
#define UART_TIMEOUT_TIME 1000000 //2000000[us] = 2000[ms]

//Debug SETTING
#define RX_IT_DEBUG 0
#define IT_FLAG_TRUE 1
#define IT_FLAG_FALSE 0
#define COM_EN_IT_NUM 2

//DMA CONST
#define RX_CIRC_BUF_SZ (128)

struct servo_params {
	// read write data
	uint8_t id;
	uint8_t strc;
	uint8_t spd;

	// write data
	uint16_t pos;
	uint8_t cur_lim;
	uint8_t tmp_lim;
	uint8_t imu_select;
	uint8_t board_select;

	// read data
	uint16_t tch;
	uint8_t cur;
	uint8_t tmp;
	uint8_t ps[PS_CHANNEL_NUM * 2];//0H,0L,1H,1L,4H,4L,7H,7L
	uint16_t ps_print[PS_CHANNEL_NUM];
	uint8_t ps_en[PS_CHANNEL_NUM]; //0x00:disable 0x01:enable
	uint8_t gyro[GYRO_CHANNEL_NUM * 2];
	int16_t gyro_print[GYRO_CHANNEL_NUM];
	uint8_t acc[ACC_CHANNEL_NUM * 2];
	int16_t acc_print[ACC_CHANNEL_NUM];
	uint8_t imu_en; //0x00:disable 0x01:enable
	uint8_t adc[ADC_CHANNEL_NUM * 2];
	uint16_t adc_print[ADC_CHANNEL_NUM];
	uint8_t sw;
	uint8_t detection_result[M5STICKV_DETECTION_RESULT_LENGTH];

	// control
	uint8_t mode;
	uint8_t state;
	uint8_t past_length;
	uint8_t txbuff[MAX_TXBUFF_LEN];
	uint8_t rxbuff[2];//header + sub command
	uint8_t rx_dma_circ_buf[RX_CIRC_BUF_SZ];
	uint16_t rd_idx;
	uint8_t com_en;

	uint16_t it_count;
	uint16_t uart_error_count;
	uint16_t ics_timeout_count;
	uint8_t uart_error[4];
	uint32_t count_frame;//us
	uint32_t prev_frame;//us
	uint32_t check2com_delay;
};

extern struct servo_params sp;
extern uint8_t recv_buf;


#endif
