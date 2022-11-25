/*
 * sensor.c
 *
 *  Created on: 2022/06/01
 *      Author: makabe
 */

#include "main.h"
#include "sensor.h"
#include "cmsis_os.h"

struct sensor_params sp;

void flatten_sensor_val(){
	uint8_t index = 0;
	for(int i = 0; i < PS_CHANNEL_NUM; i++){//1 * 8 = 8
		sp.ps_print_flatten[index] = sp.ps_print_2d[0][i];
		index++;
	}
	switch(sp.board_select){
	case 0:
		for (int i = index; i < MAX_PS_SENSOR_NUM; i++){
			sp.ps_print_flatten[i] = 11111;
		}
		break;
	case 1:
		for(int j=1; j < PCA9547_NUM; j++){//4 * 4 = 16
			for (int i=0; i < (PS_CHANNEL_NUM - 4); i++){
				sp.ps_print_flatten[index] = sp.ps_print_2d[j][i];
				index++;
			}
		}
		break;
	}

	index = 0;
	for(int i = 0; i < FS_CHANNEL_NUM; i++){//1 * 8 = 8
		sp.adc_print_flatten[index] = sp.adc_print[i];
		index++;
	}
	switch(sp.board_select){
	case 0:
		for (int i = index; i < MAX_FS_SENSOR_NUM; i++){
			sp.adc_print_flatten[i] = 22222;
		}
		break;
	case 1:
		for(int j=0; j < ADS7828_NUM; j++){//4 * 4 = 16
			for (int i=0; i < ADC_CHANNEL_NUM_ADS; i++){
				sp.adc_print_flatten[index] = sp.adc_print_ADS_2d[j][i];
				index++;
			}
		}
		break;
	}
}

void txbuff_update(){//max: uint8_t * 44:
	flatten_sensor_val();
	uint8_t index = 0;
	uint16_t check_sum = 0;

	switch(sp.spi_slave_flag){
	case 0:
		for(int i = 0; i < (MAX_FS_SENSOR_NUM / 2); i++){//(24 / 2) * 3 = 36
			sp.txbuff_state[sp.spi_slave_flag][index] = (sp.ps_print_flatten[i] & 0x0ff0) >> 4;
			sp.txbuff_state[sp.spi_slave_flag][index + 1] = ((sp.ps_print_flatten[i] & 0x000f) << 4) | ((sp.adc_print_flatten[i] & 0x0f00) >> 8);
			sp.txbuff_state[sp.spi_slave_flag][index + 2] = (sp.adc_print_flatten[i] & 0x00ff);
			index += 3;
		}//need time for coppy?
		for(int i = 0; i < GYRO_CHANNEL_NUM; i++){//2 * 3 = 6
			sp.txbuff_state[sp.spi_slave_flag][index] = sp.gyro_print[i] >> 8;
			sp.txbuff_state[sp.spi_slave_flag][index + 1] = sp.gyro_print[i] & 0x00ff;
			index += 2;
		}
		break;
	case 1:
		for(int i = (MAX_FS_SENSOR_NUM / 2); i < MAX_FS_SENSOR_NUM; i++){//(24 / 2) * 3 = 36
			sp.txbuff_state[sp.spi_slave_flag][index] = (sp.ps_print_flatten[i] & 0x0ff0) >> 4;
			sp.txbuff_state[sp.spi_slave_flag][index + 1] = ((sp.ps_print_flatten[i] & 0x000f) << 4) | ((sp.adc_print_flatten[i] & 0x0f00) >> 8);
			sp.txbuff_state[sp.spi_slave_flag][index + 2] = (sp.adc_print_flatten[i] & 0x00ff);
			index += 3;
		}//need time for coppy?
		for(int i = 0; i < ACC_CHANNEL_NUM; i++){//2 * 3 = 6
			sp.txbuff_state[sp.spi_slave_flag][index] = sp.acc_print[i] >> 8;
			sp.txbuff_state[sp.spi_slave_flag][index + 1] = sp.acc_print[i] & 0x00ff;
			index += 2;
		}
		break;
	}
	for(int i = 1; i < (MAX_FS_SENSOR_NUM / 2) * 3 + GYRO_CHANNEL_NUM * 2; i += 2){//36 + 6 =42
		check_sum += sp.txbuff_state[sp.spi_slave_flag][i];
	}

	/*for serial publish*/
	if (sp.slave_mode == UART_SLAVE){
		for(int i=0; i < MAX_PS_SENSOR_NUM; i++){ // proximity 24
			sp.serial_publish_flatten[i] = (int16_t)sp.ps_print_flatten[i];
		}
		for(int i=0; i < MAX_FS_SENSOR_NUM; i++){ // force 24
			sp.serial_publish_flatten[i + MAX_PS_SENSOR_NUM] = (int16_t)sp.adc_print_flatten[i];
		}
		for(int i=0; i < ACC_CHANNEL_NUM; i++){ // acc 3
			sp.serial_publish_flatten[i + MAX_PS_SENSOR_NUM + MAX_FS_SENSOR_NUM] = (int16_t)sp.acc_print[i];
		}
		for(int i=0; i < GYRO_CHANNEL_NUM; i++){ // gyro 3
			sp.serial_publish_flatten[i + MAX_PS_SENSOR_NUM + MAX_FS_SENSOR_NUM + ACC_CHANNEL_NUM] = (int16_t)sp.gyro_print[i];
		}
	}


	sp.txbuff_state[sp.spi_slave_flag][TXBUFF_LENGTH - 2] = (((sp.board_select & 0x0f) << 4) | (sp.spi_slave_flag & 0x0f));
	sp.txbuff_state[sp.spi_slave_flag][TXBUFF_LENGTH - 1] = check_sum & 0x00ff;

	sp.spi_slave_flag += 1;
	if(sp.spi_slave_flag == SPI_SLAVE_STATENUM){
		sp.spi_slave_flag = 0;
	}
}

void txbuff_update_old(){//max: uint8_t * 44: 44-(8+6+6+2+16)=6
	uint8_t index = 0;
	for(int i=0; i < GYRO_CHANNEL_NUM; i++){//2*3=6
		sp.txbuff[index] = sp.gyro_print[i] >> 8;
		sp.txbuff[index + 1] = sp.gyro_print[i] & 0x00ff;
		index += 2;
	}
	for(int i=0; i < ACC_CHANNEL_NUM; i++){//2*3=6
		sp.txbuff[index] = sp.acc_print[i] >> 8;
		sp.txbuff[index + 1] = sp.acc_print[i] & 0x00ff;
		index += 2;
	}
	for(int i=0; i < PS_CHANNEL_NUM; i++){//2*1=2
		sp.txbuff[index] = sp.ps_print[i] >> 8;
		sp.txbuff[index + 1] = sp.ps_print[i] & 0x00ff;
		index += 2;
	}
	for(int i=0; i < ADC_CHANNEL_NUM; i++){//2*4=8
		sp.txbuff[index] = sp.adc_print[i] >> 8;
		sp.txbuff[index + 1] = sp.adc_print[i] & 0x00ff;
		index += 2;
	}
	for(int i = 0; i < MIC_CHANNEL_NUM; i++){//4*4=16
		if(sp.i2s_buff_sifted[i * 2] != 0){
			sp.txbuff[index] = (sp.i2s_buff_sifted[i * 2] >> 10) & 0x000000ff;
			sp.txbuff[index + 1] = (sp.i2s_buff_sifted[i * 2] >> 2)& 0x000000ff;
		}else if(sp.i2s_buff_sifted[i * 2 + 1] != 0){
			sp.txbuff[index] = (sp.i2s_buff_sifted[i * 2 + 1] >> 10) & 0x000000ff;
			sp.txbuff[index + 1] = (sp.i2s_buff_sifted[i * 2 + 1] >> 2)& 0x000000ff;
		}else{
			sp.txbuff[index] = (12345 * 4 >> 10) & 0x000000ff;
			sp.txbuff[index + 1] = (12345 * 4 >> 2)& 0x000000ff;
		}
		//sp.txbuff[index] = (12345 >> 8) & 0x000000ff;
		//sp.txbuff[index + 1] = 12345 & 0x000000ff;

		index += 2;
	}
	sp.txbuff[index] = sp.mic_elapsed_time >> 8;
	sp.txbuff[index + 1] = sp.mic_elapsed_time & 0x00ff;
	sp.txbuff[index + 2] = sp.adc_elapsed_time >> 8;
	sp.txbuff[index + 3] = sp.adc_elapsed_time & 0x00ff;
	sp.txbuff[index + 4] = sp.imu_elapsed_time >> 8;
	sp.txbuff[index + 5] = sp.imu_elapsed_time & 0x00ff;
	index += 6;
	//sp.txbuff[index + 6] = sp.ps_elapsed_time >> 8;
	//sp.txbuff[index + 7] = sp.ps_elapsed_time & 0x00ff;
	//index += 8;
	sp.txbuff[index] = sp.error_count >> 8;
	sp.txbuff[index + 1] = sp.error_count & 0x00ff;
	index += 2;
	sp.txbuff[index] = 0;
	sp.txbuff[index + 1] = sp.rxbuff[0];
	index += 2;
	sp.txbuff[index] = sp.rx_counter >> 8;
	sp.txbuff[index + 1] = sp.rx_counter & 0x00ff;
	index += 2;

	/*
	for(int i=0; i < MIC_CHANNEL_NUM; i++){//4*4=16
		sp.txbuff[index] = 0x00;
		sp.txbuff[index + 1] = sp.i2s_buff_sifted[i * 2] >> 16;
		sp.txbuff[index + 2] = (sp.i2s_buff_sifted[i * 2] >> 8) & 0x000000ff;
		sp.txbuff[index + 3] = sp.i2s_buff_sifted[i * 2] & 0x000000ff;
		index += 4;
	}
	*/
	for(int i = index; i < TXBUFF_LENGTH; i++){
		if((sp.txbuff[i] == 0) && (i % 2 == 1)){
			sp.txbuff[i] = i / 2;
		}
	}
}

uint32_t getUs(void) {
  uint32_t usTicks = HAL_RCC_GetSysClockFreq() / 1000000;
  register uint32_t ms, cycle_cnt;
  do {
    ms = HAL_GetTick();
    cycle_cnt = SysTick->VAL;
  } while (ms != HAL_GetTick());
  return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

void delayUs(uint16_t micros) {
	uint32_t start = getUs();
	while (getUs()-start < (uint32_t) micros) {
		asm("nop");
	}
}

void mpuWrite(SPI_HandleTypeDef *hspi, uint8_t address, uint8_t value)
{
	HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &address, 1, 1000);
	HAL_SPI_Transmit(hspi, &value, 1, 1000);
	HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_SET);
}

void mpuRead(SPI_HandleTypeDef *hspi, uint8_t *address, uint8_t *value)
{
	HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &*address, 1, 1000);
	HAL_SPI_Receive(hspi, &*value, 1, 1000);
	HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_SET);
}

void imu_init(SPI_HandleTypeDef *hspi){

	uint8_t t_data;
	uint8_t who = 0x00;
	uint8_t count = 0;

	switch(sp.imu_select){
	case 0:
		t_data = ICM_20600_PING_ADDRESS | 0x80;
		break;
	case 1:
		t_data = ICM_42605_PING_ADDRESS | 0x80;
		break;
	case 2:
		t_data = ICM_42688_PING_ADDRESS | 0x80;
		break;
	}

	while ((who != IMU_WHO_AM_I_20600) && (who != IMU_WHO_AM_I_42605) && (who != IMU_WHO_AM_I_42688) && (count < 20)){
		mpuRead(hspi, &t_data, &who);
		count++;
		HAL_Delay(100);
	}

	if((who == IMU_WHO_AM_I_20600  && sp.imu_select == SELECT_ICM_20600) ||
			(who == IMU_WHO_AM_I_42605  && sp.imu_select == SELECT_ICM_42605) ||
			(who == IMU_WHO_AM_I_42688  && sp.imu_select == SELECT_ICM_42688_SPI)){
		sp.imu_en = IMU_EN;
		HAL_Delay(100);
		switch(sp.imu_select){
		case 0:
			//reset-device
			mpuWrite(hspi,ICM_20600_PWR_MGMT_1,ICM_20600_PWR_MGMT_1_VAL);//important setting
			HAL_Delay(1);

			//init-gyro
			mpuWrite(hspi, ICM_20600_CONFIG, ICM_20600_GYRO_DLPF_CFG); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
			HAL_Delay(1); //very importnat! between gyro and acc
			mpuWrite(hspi, ICM_20600_GYRO_CONFIG, ICM_20600_GYRO_CONFIG_VAL); //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
			HAL_Delay(10); //very importnat! between gyro and acc

			//init-acc
			mpuWrite(hspi, ICM_20600_ACCEL_CONFIG, ICM_20600_ACCEL_CONFIG_VAL); //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
			HAL_Delay(1);
			mpuWrite(hspi, ICM_20600_ACCEL_CONFIG2, ICM_20600_ACC_DLPF_CFG);
			HAL_Delay(10);
			break;
		case 1:
			//set-gyro-scale-and-ODR
			mpuWrite(hspi, ICM_42605_GYRO_CONFIG0, ICM_42605_GYRO_CONFIG0_VAL); //GYRO_FS_SEL = 0: Full scale set to 2000 deg/sec, 1kHz ODR
			HAL_Delay(10); //very importnat! between gyro and acc

			//set-acc-scale-and-ODR
			mpuWrite(hspi, ICM_42605_ACCEL_CONFIG0, ICM_42605_ACCEL_CONFIG0_VAL);//ACCEL_FS_SEL = 0: Full scale set to +/-16G, 1kHz ODR
			HAL_Delay(10); //very importnat! between gyro and acc

			//set-gyro-acc-LPF
			mpuWrite(hspi, ICM_42605_GYRO_ACCEL_CONFIG0, ICM_42605_GYRO_ACCEL_CONFIG0_VAL); //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
			HAL_Delay(10);

			//start gyro and acc
			mpuWrite(hspi, ICM_42605_PWR_MGMT0, ICM_42605_PWR_MGMT0_VAL);//Turn on Gyro and Acc with Low Noise Mode
			HAL_Delay(10);
			break;
		case 2:
			//set-gyro-scale-and-ODR
			mpuWrite(hspi, ICM_42688_GYRO_CONFIG0, ICM_42688_GYRO_CONFIG0_VAL_1khz); //GYRO_FS_SEL = 0: Full scale set to 2000 deg/sec, 1kHz ODR
			HAL_Delay(10); //very importnat! between gyro and acc

			//set-acc-scale-and-ODR
			mpuWrite(hspi, ICM_42688_ACCEL_CONFIG0, ICM_42688_ACCEL_CONFIG0_VAL_1khz);//ACCEL_FS_SEL = 0: Full scale set to +/-16G, 1kHz ODR
			HAL_Delay(10); //very importnat! between gyro and acc

			//set-gyro-acc-LPF
			mpuWrite(hspi, ICM_42688_GYRO_ACCEL_CONFIG0, ICM_42688_GYRO_ACCEL_CONFIG0_VAL); //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
			HAL_Delay(10);

			//start gyro and acc
			mpuWrite(hspi, ICM_42688_PWR_MGMT0, ICM_42688_PWR_MGMT0_VAL);//Turn on Gyro and Acc with Low Noise Mode
			HAL_Delay(10);
			break;
		}

	}else{
		sp.imu_en = IMU_NOT_EN;
	}
}

void imu_update(SPI_HandleTypeDef *hspi1){
	uint8_t gyro_t_data[1];
	uint8_t acc_t_data[1];

	switch(sp.imu_select){
	case 0:
		gyro_t_data[0] = ICM_20600_GYRO_XOUT_H | 0x80;
		acc_t_data[0] = ICM_20600_ACCEL_XOUT_H | 0x80;
		break;
	case 1:
		gyro_t_data[0] = ICM_42605_GYRO_DATA_X1 | 0x80;
		acc_t_data[0] = ICM_42605_ACCEL_DATA_X1 | 0x80;
		break;
	case 2:
		gyro_t_data[0] = ICM_42688_GYRO_DATA_X1 | 0x80;
		acc_t_data[0] = ICM_42688_ACCEL_DATA_X1 | 0x80;
		break;
	}

	if(sp.imu_en == IMU_EN){
		HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(hspi1, gyro_t_data, 1, 10);
		HAL_SPI_Receive(hspi1, sp.gyro, 6, 10);
		HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_SET);

		if((sp.gyro[0] != 0) || (sp.gyro[1] != 0) || (sp.gyro[2] != 0)){
			sp.gyro_print[0] = (int16_t)(sp.gyro[0] << 8 | sp.gyro[1]);
			sp.gyro_print[1] = (int16_t)(sp.gyro[2] << 8 | sp.gyro[3]);
			sp.gyro_print[2] = (int16_t)(sp.gyro[4] << 8 | sp.gyro[5]);
		}

		HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(hspi1, acc_t_data, 1, 10);
		HAL_SPI_Receive(hspi1, sp.acc, 6, 10);
		HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_SET);

		if((sp.acc[0] != 0) || (sp.acc[1] != 0) || (sp.acc[2] != 0)){
			sp.acc_print[0] = (int16_t)(sp.acc[0] << 8 | sp.acc[1]);
			sp.acc_print[1] = (int16_t)(sp.acc[2] << 8 | sp.acc[3]);
			sp.acc_print[2] = (int16_t)(sp.acc[4] << 8 | sp.acc[5]);
		}
	}
}

void acc_update(SPI_HandleTypeDef *hspi) {
	acc_flag = 1;
	uint8_t acc_t_data[1];
	switch(sp.imu_select){
	case 0:
		acc_t_data[0] = ICM_20600_ACCEL_XOUT_H | 0x80;
		break;
	case 1:
		acc_t_data[0] = ICM_42605_ACCEL_DATA_X1 | 0x80;
		break;
	case 2:
		acc_t_data[0] = ICM_42688_ACCEL_DATA_X1 | 0x80;
		break;
	}

	if(sp.imu_en == IMU_EN){
    	HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_RESET);
    	HAL_SPI_Transmit_DMA(hspi, acc_t_data, 1);
	}
}

void gyro_update(SPI_HandleTypeDef *hspi){
	acc_flag = 0;
	uint8_t gyro_t_data[1];

	switch(sp.imu_select){
	case 0:
		gyro_t_data[0] = ICM_20600_GYRO_XOUT_H | 0x80;
		break;
	case 1:
		gyro_t_data[0] = ICM_42605_GYRO_DATA_X1 | 0x80;
		break;
	case 2:
		gyro_t_data[0] = ICM_42688_GYRO_DATA_X1 | 0x80;
		break;
	}

	if(sp.imu_en == IMU_EN){
		HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit_DMA(hspi, gyro_t_data, 1);
	}
}

void adc_init(ADC_HandleTypeDef *hadc){
   if (HAL_ADC_Start(hadc) !=  HAL_OK)
   {
	   Error_Handler();
	}
	for(int i=0; i < ADC_CHANNEL_NUM; i++){
		sp.adc_print[i] = 0;
	}
}

void adc_update(ADC_HandleTypeDef *hadc){
	HAL_ADC_Start(hadc);
	for (int i = 0; i < ADC_CHANNEL_NUM; i++){
		HAL_StatusTypeDef status = HAL_ADC_PollForConversion(hadc, 1);
		if (status == HAL_OK){
			uint16_t value = HAL_ADC_GetValue(hadc);
			sp.adc[i * 2] = value >> 8;
			sp.adc[i * 2 + 1] = value & 0x00ff;
		}
	}
	for(int i=0; i < ADC_CHANNEL_NUM; i++){
		sp.adc_print_raw[i] = (int16_t)(sp.adc[i * 2] << 8 | sp.adc[i * 2 + 1]);
	}
	sp.adc_print[0] = sp.adc_print_raw[3];
	sp.adc_print[1] = sp.adc_print_raw[2];
	sp.adc_print[2] = sp.adc_print_raw[1];
	sp.adc_print[3] = sp.adc_print_raw[0];
	sp.adc_print[4] = sp.adc_print_raw[4];
	sp.adc_print[5] = sp.adc_print_raw[6];
	sp.adc_print[6] = sp.adc_print_raw[5];
	sp.adc_print[7] = sp.adc_print_raw[7];
	sp.adc_print[8] = sp.adc_print_raw[8];
	HAL_ADC_Stop (hadc);
}

void ps_select_channel(I2C_HandleTypeDef *hi2c, uint8_t ch){
	uint8_t tx;
	tx = 0x08 | ch;//PCA9547 format
	HAL_I2C_Master_Transmit(hi2c, PCA9547_ADDR, &tx, 1, HAL_MAX_DELAY);//select i2c channel
}

void ps_select_channel_ADDR(I2C_HandleTypeDef *hi2c, uint8_t ch, uint8_t ADDR){
	uint8_t tx;
	tx = 0x08 | ch;//PCA9547 format
	HAL_I2C_Master_Transmit(hi2c, ADDR, &tx, 1, HAL_MAX_DELAY);//select i2c channel
}

void disable_mux(I2C_HandleTypeDef *hi2c, uint8_t ADDR){
	uint8_t tx;
	tx = 0x00;//PCA9547 format, disable IC
	HAL_I2C_Master_Transmit(hi2c, ADDR, &tx, 1, HAL_MAX_DELAY);//select i2c channel
}

void disable_all_mux(I2C_HandleTypeDef *hi2c){
	for (int i = 0; i < PCA9547_NUM; i++){
		disable_mux(hi2c, PCA9547_ADDR_ARRAY[i]);
	}
}

void ps_init(I2C_HandleTypeDef *hi2c){
	uint8_t init_buff[2];
	uint8_t start_buff[2];
	uint8_t id_buff[2];
	id_buff[0] = 0x00;
	id_buff[1] = 0x00;
	init_buff[0] = 0x00;
	//init_buff[1] = 0x06;//180[mA]
	//init_buff[1] = 0x02;//100[mA]
	init_buff[1] = 0x00;//50[mA]
	//start_buff[0] = 0xce;//1/320,8T
	//start_buff[0] = 0x0e;//1/40,8T
	//start_buff[0] = 0x0c;//1/40,4T
	//start_buff[0] = 0x04;//1/40,2T
	start_buff[0] = 0x00;//1/40,1T
	start_buff[1] = 0x08;//16bit

	switch(sp.board_select){
	case 0:
		for (int i = 0; i < PS_CHANNEL_NUM; i++){
			ps_select_channel(hi2c, PS_CHANNEL_ARRAY_PCA9457[i]);

			HAL_I2C_Mem_Read(hi2c, VCNL4040_ADDR, ID_L, 1, id_buff, 2, HAL_MAX_DELAY);//check sensor ID

			if(id_buff[0] == ID_L_VAL && id_buff[1] == ID_H_VAL){
				sp.ps_en_2d[0][i] = PS_EN;
				HAL_I2C_Mem_Write(hi2c, VCNL4040_ADDR, PS_CONF3, 1, init_buff, 2, HAL_MAX_DELAY);//LED setting
				HAL_I2C_Mem_Write(hi2c, VCNL4040_ADDR, PS_CONF1, 1, start_buff, 2, HAL_MAX_DELAY);//Turn on LED
			}else{
				sp.ps_en_2d[0][i] = PS_NOT_EN;
			}
		}
		break;
	case 1:
		disable_all_mux(hi2c);
		for (int i = 0; i < PCA9547_NUM; i++){
			for (int j = 0; j < PS_CHANNEL_NUM; j++){
				if(PS_CHANNEL_2DARRAY_PCA9457[i][j] < 0x08){
					ps_select_channel_ADDR(hi2c, PS_CHANNEL_2DARRAY_PCA9457[i][j], PCA9547_ADDR_ARRAY[i]);
					HAL_I2C_Mem_Read(hi2c, VCNL4040_ADDR, ID_L, 1, id_buff, 2, 100);//check sensor ID
					if(id_buff[0] == ID_L_VAL && id_buff[1] == ID_H_VAL){
						sp.ps_en_2d[i][j] = PS_EN;
						HAL_I2C_Mem_Write(hi2c, VCNL4040_ADDR, PS_CONF3, 1, init_buff, 2, HAL_MAX_DELAY);//LED setting
						HAL_I2C_Mem_Write(hi2c, VCNL4040_ADDR, PS_CONF1, 1, start_buff, 2, HAL_MAX_DELAY);//Turn on LED
					}else{
						sp.ps_en_2d[i][j] = PS_NOT_EN;
					}
				}else{
					sp.ps_en_2d[i][j] = PS_NOT_EN;
				}
			}
			disable_mux(hi2c,PCA9547_ADDR_ARRAY[i]);
		}
		disable_all_mux(hi2c);
		break;
	}
}

uint8_t ps_data_tmp[2];

void ps_update(I2C_HandleTypeDef *hi2c){
	uint8_t start_buff[2];
	uint8_t stop_buff[2];
	uint8_t data[2];
	uint8_t ps_ret = 0xff;
	start_buff[0] = 0x0e;
	start_buff[1] = 0x08;
	stop_buff[0] = 0x01;
	stop_buff[1] = 0x00;

	switch(sp.board_select){
	case 0:
		for (int i = 0; i < PS_CHANNEL_NUM; i++){
			data[0] = 0x00;
			data[1] = 0x00;

			if(sp.ps_en_2d[0][i] == PS_EN){
				ps_select_channel(hi2c, PS_CHANNEL_ARRAY_PCA9457[i]);

				ps_ret = HAL_I2C_Mem_Read(hi2c, VCNL4040_ADDR, PS_DATA_L, 1, data, 2, HAL_MAX_DELAY);

				if(ps_ret == HAL_OK){
					sp.ps_print_raw[i] = (uint16_t)(data[1] << 8 | data[0]);

					sp.ps_2d[0][i * 2] = data[1];

					sp.ps_2d[0][i * 2 + 1] = data[0];
				}
			}
		}
		break;
	case 1:
		for (int i = 0; i < PCA9547_NUM; i++){
			for (int j = 0; j < PS_CHANNEL_NUM; j++){
				data[0] = 0x00;
				data[1] = 0x00;

				if(sp.ps_en_2d[i][j] == PS_EN){
					ps_select_channel_ADDR(hi2c, PS_CHANNEL_2DARRAY_PCA9457[i][j], PCA9547_ADDR_ARRAY[i]);

					ps_ret = HAL_I2C_Mem_Read(hi2c, VCNL4040_ADDR, PS_DATA_L, 1, data, 2, HAL_MAX_DELAY);

					if(ps_ret == HAL_OK){
						if (i == 0){
							sp.ps_print_raw[j] = (uint16_t)(data[1] << 8 | data[0]);
						}else{
							sp.ps_print_2d[i][j] = (uint16_t)(data[1] << 8 | data[0]);
						}

						sp.ps_2d[i][j * 2] = data[1];

						sp.ps_2d[i][j * 2 + 1] = data[0];
					}
				}
			}
			disable_mux(hi2c,PCA9547_ADDR_ARRAY[i]);
		}
		break;
	}
	sp.ps_print_2d[0][0] = sp.ps_print_raw[3];
	sp.ps_print_2d[0][1] = sp.ps_print_raw[4];
	sp.ps_print_2d[0][2] = sp.ps_print_raw[5];
	sp.ps_print_2d[0][3] = sp.ps_print_raw[6];
	sp.ps_print_2d[0][4] = sp.ps_print_raw[7];
	sp.ps_print_2d[0][5] = sp.ps_print_raw[0];
	sp.ps_print_2d[0][6] = sp.ps_print_raw[1];
	sp.ps_print_2d[0][7] = sp.ps_print_raw[2];
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == I2C1) {
		sp.ps_print[0] = (uint16_t)(ps_data_tmp[1] << 8 | ps_data_tmp[0]);
		sp.ps[0 * 2] = ps_data_tmp[1];
		sp.ps[0 * 2 + 1] = ps_data_tmp[0];
		HAL_Delay(1);
		ps_update(hi2c);
	}
}

void adc_update_ADS7828(I2C_HandleTypeDef *hi2c){
	uint8_t command;
	uint8_t command_PD;
	uint8_t adc_ret;
	//command_PD = 0x00; //power down between ADC conversions
	//command_PD = 0x04; //Internal reference off and ADC on
	//command_PD = 0x08; //Internal reference on and ADC off
	command_PD = 0x0C; //Internal reference on and ADC on

	uint8_t data[2];

	for (int i = 0; i < ADS7828_NUM; i++){
		for (int j = 0; j < ADC_CHANNEL_NUM_ADS; j++){
			data[0] = 0x00;
			data[1] = 0x00;
			command = command_PD | ADC_CHANNEL_ARRAY[j];

			adc_ret = HAL_I2C_Mem_Read(hi2c, ADS7828_ADDR_ARRAY[i], command, 1, data, 2, HAL_MAX_DELAY);
			//adc_ret = HAL_I2C_Mem_Read(hi2c, 0x34, command, 1, data, 2, HAL_MAX_DELAY);

			if(adc_ret == HAL_OK){
				sp.adc_print_ADS_2d[i][j] = (uint16_t)(data[0] << 8 | data[1]);

				sp.adc_ADS_2d[i][j * 2] = data[0];
				sp.adc_ADS_2d[i][j * 2 + 1] = data[1];
			}
		}
	}
}
