/*
 * sensor.c
 *
 *  Created on: 2022/06/01
 *      Author: makabe
 */

#include "sensor.h"
#include "cmsis_os.h"

struct sensor_params sp;

void txbuff_update(){//max: uint8_t * 44: 44-(8+6+6+2+16)=6
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
	HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &address, 1, 1000);
	HAL_SPI_Transmit(hspi, &value, 1, 1000);
	HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
}

void mpuRead(SPI_HandleTypeDef *hspi, uint8_t *address, uint8_t *value)
{
	HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &*address, 1, 1000);
	HAL_SPI_Receive(hspi, &*value, 1, 1000);
	HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
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

void imu_update(SPI_HandleTypeDef *hspi){
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
		HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
		HAL_SPI_Transmit(hspi, gyro_t_data, 1, 10);
		HAL_SPI_Receive(hspi, sp.gyro, 6, 10);
		HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);

		if((sp.gyro[0] != 0) || (sp.gyro[1] != 0) || (sp.gyro[2] != 0)){
			sp.gyro_print[0] = (int16_t)(sp.gyro[0] << 8 | sp.gyro[1]);
			sp.gyro_print[1] = (int16_t)(sp.gyro[2] << 8 | sp.gyro[3]);
			sp.gyro_print[2] = (int16_t)(sp.gyro[4] << 8 | sp.gyro[5]);
		}

		HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
		HAL_SPI_Transmit(hspi, acc_t_data, 1, 10);
		HAL_SPI_Receive(hspi, sp.acc, 6, 10);
		HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);

		if((sp.acc[0] != 0) || (sp.acc[1] != 0) || (sp.acc[2] != 0)){
			sp.acc_print[0] = (int16_t)(sp.acc[0] << 8 | sp.acc[1]);
			sp.acc_print[1] = (int16_t)(sp.acc[2] << 8 | sp.acc[3]);
			sp.acc_print[2] = (int16_t)(sp.acc[4] << 8 | sp.acc[5]);
		}
	}
}

void imu_init_i2c(I2C_HandleTypeDef *hi2c){
	uint8_t id_buff = 0x00;
	uint8_t tx_buff;
	uint8_t count = 0;

	HAL_Delay(500);

	while ((id_buff != IMU_WHO_AM_I_42688) && (count < 20)){
		HAL_I2C_Mem_Read(hi2c, ICM_42688_I2C_ADDR, ICM_42688_PING_ADDRESS, 1, &id_buff, 1, 100);//check sensor ID
		count++;
		HAL_Delay(100);
	}
	HAL_Delay(10);

	if(id_buff == IMU_WHO_AM_I_42688){
		sp.imu_en = IMU_EN;
		//set-gyro-scale-and-ODR
		tx_buff = ICM_42688_GYRO_CONFIG0_VAL_1khz;
		//HAL_I2C_Mem_Write(hi2c, ICM_42688_I2C_ADDR, ICM_42688_GYRO_CONFIG0, 1, &tx_buff, 1, HAL_MAX_DELAY);//GYRO_FS_SEL = 0: Full scale set to 2000 deg/sec, 1kHz ODR
		HAL_I2C_Mem_Write(hi2c, ICM_42688_I2C_ADDR, ICM_42688_GYRO_CONFIG0, 1, &tx_buff, 1, 1000);//GYRO_FS_SEL = 0: Full scale set to 2000 deg/sec, 1kHz ODR
		HAL_Delay(100); //very importnat! between gyro and acc

		//set-acc-scale-and-ODR
		tx_buff = ICM_42688_ACCEL_CONFIG0_VAL_1khz;
		//HAL_I2C_Mem_Write(hi2c, ICM_42688_I2C_ADDR, ICM_42688_ACCEL_CONFIG0, 1, &tx_buff, 1, HAL_MAX_DELAY);//ACCEL_FS_SEL = 0: Full scale set to +/-16G, 1kHz ODR
		HAL_I2C_Mem_Write(hi2c, ICM_42688_I2C_ADDR, ICM_42688_ACCEL_CONFIG0, 1, &tx_buff, 1, 1000);//ACCEL_FS_SEL = 0: Full scale set to +/-16G, 1kHz ODR
		HAL_Delay(100); //very importnat! between gyro and acc

		//set-gyro-acc-LPF
		tx_buff = ICM_42688_GYRO_ACCEL_CONFIG0_VAL;
		//HAL_I2C_Mem_Write(hi2c, ICM_42688_I2C_ADDR, ICM_42688_GYRO_ACCEL_CONFIG0, 1, &tx_buff, 1, HAL_MAX_DELAY);//ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
		HAL_I2C_Mem_Write(hi2c, ICM_42688_I2C_ADDR, ICM_42688_GYRO_ACCEL_CONFIG0, 1, &tx_buff, 1, 1000);//ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
		HAL_Delay(100);

		//start gyro and acc
		tx_buff = ICM_42688_PWR_MGMT0_VAL;
		//HAL_I2C_Mem_Write(hi2c, ICM_42688_I2C_ADDR, ICM_42688_PWR_MGMT0, 1, &tx_buff, 1, HAL_MAX_DELAY);//Turn on Gyro and Acc with Low Noise Mode
		HAL_I2C_Mem_Write(hi2c, ICM_42688_I2C_ADDR, ICM_42688_PWR_MGMT0, 1, &tx_buff, 1, 1000);//Turn on Gyro and Acc with Low Noise Mode
		HAL_Delay(100);
	}else{
		sp.imu_en = IMU_NOT_EN;
	}
}

void imu_update_i2c(I2C_HandleTypeDef *hi2c){
	if(sp.imu_en == IMU_EN){
		//taskENTER_CRITICAL();
		HAL_I2C_Mem_Read(hi2c, ICM_42688_I2C_ADDR, ICM_42688_GYRO_DATA_X1, 1, sp.gyro, 6, 1000);//check sensor ID
		//HAL_I2C_Mem_Read(hi2c, ICM_42688_I2C_ADDR, ICM_42688_GYRO_DATA_X1, 1, sp.gyro, 6, 1);//check sensor ID
		//taskEXIT_CRITICAL();
		if((sp.gyro[0] != 0) || (sp.gyro[1] != 0) || (sp.gyro[2] != 0)){
			sp.gyro_print[0] = (int16_t)(sp.gyro[0] << 8 | sp.gyro[1]);
			sp.gyro_print[1] = (int16_t)(sp.gyro[2] << 8 | sp.gyro[3]);
			sp.gyro_print[2] = (int16_t)(sp.gyro[4] << 8 | sp.gyro[5]);
		}
		HAL_Delay(IMU_GYRO_ACC_DELAY);//important delay

		//taskENTER_CRITICAL();
		HAL_I2C_Mem_Read(hi2c, ICM_42688_I2C_ADDR, ICM_42688_ACCEL_DATA_X1, 1, sp.acc, 6, 1000);//check sensor ID
		//HAL_I2C_Mem_Read(hi2c, ICM_42688_I2C_ADDR, ICM_42688_ACCEL_DATA_X1, 1, sp.acc, 6, 1);//check sensor ID
		//taskEXIT_CRITICAL();
		if((sp.acc[0] != 0) || (sp.acc[1] != 0) || (sp.acc[2] != 0)){
			sp.acc_print[0] = (int16_t)(sp.acc[0] << 8 | sp.acc[1]);
			sp.acc_print[1] = (int16_t)(sp.acc[2] << 8 | sp.acc[3]);
			sp.acc_print[2] = (int16_t)(sp.acc[4] << 8 | sp.acc[5]);
		}
		HAL_Delay(IMU_GYRO_ACC_DELAY);//important delay

		/*
		taskENTER_CRITICAL();
		HAL_I2C_Mem_Read(hi2c, ICM_42688_I2C_ADDR, ICM_42688_GYRO_DATA_X1, 1, sp.gyro_acc, 12, 100);//check sensor ID
		taskEXIT_CRITICAL();
		if((sp.gyro_acc[0] != 0) || (sp.gyro_acc[1] != 0) || (sp.gyro_acc[2] != 0)){
			sp.gyro_print[0] = (int16_t)(sp.gyro_acc[0] << 8 | sp.gyro_acc[1]);
			sp.gyro_print[1] = (int16_t)(sp.gyro_acc[2] << 8 | sp.gyro_acc[3]);
			sp.gyro_print[2] = (int16_t)(sp.gyro_acc[4] << 8 | sp.gyro_acc[5]);
		}
		if((sp.gyro_acc[6] != 0) || (sp.gyro_acc[7] != 0) || (sp.gyro_acc[8] != 0)){
			sp.acc_print[0] = (int16_t)(sp.gyro_acc[6] << 8 | sp.gyro_acc[7]);
			sp.acc_print[1] = (int16_t)(sp.gyro_acc[8] << 8 | sp.gyro_acc[9]);
			sp.acc_print[2] = (int16_t)(sp.gyro_acc[10] << 8 | sp.gyro_acc[11]);
		}
		HAL_Delay(5);//important delay
		*/
	}
}

void imu_update_i2c_DMA_gyro(I2C_HandleTypeDef *hi2c){
	if(sp.imu_en == IMU_EN){
		//sp.imu_prev_frame = getUs();
		sp.i2c1_dma_flag = I2C1_DMA_GYRO;
		//HAL_Delay(50);
		//HAL_I2C_IsDeviceReady(hi2c, ICM_42688_I2C_ADDR, 1, 100);
		HAL_I2C_Mem_Read_DMA(hi2c, ICM_42688_I2C_ADDR, ICM_42688_GYRO_DATA_X1, 1, sp.gyro, 6);
		//HAL_Delay(50);
	}
}

void imu_update_i2c_DMA_acc(I2C_HandleTypeDef *hi2c){
	if(sp.imu_en == IMU_EN){
		sp.i2c1_dma_flag = I2C1_DMA_ACC;
		//HAL_Delay(50);
		HAL_I2C_Mem_Read_DMA(hi2c, ICM_42688_I2C_ADDR, ICM_42688_ACCEL_DATA_X1, 1, sp.acc, 6);
		//HAL_Delay(50);
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
		sp.adc_print[i] = (int16_t)(sp.adc[i * 2] << 8 | sp.adc[i * 2 + 1]);
	}
	HAL_ADC_Stop (hadc);
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

	for (int i = 0; i < PS_CHANNEL_NUM; i++){
		/*
		if(sp.board_select == SELECT_revA){
			ps_select_channel(hi2c, PS_CHANNEL_ARRAY_PCA9458[i]);
		}else if(sp.board_select == SELECT_revB){
			ps_select_channel(hi2c, PS_CHANNEL_ARRAY_PCA9457[i]);
		}*/

		HAL_I2C_Mem_Read(hi2c, VCNL4040_ADDR, ID_L, 1, id_buff, 2, HAL_MAX_DELAY);//check sensor ID

		if(id_buff[0] == ID_L_VAL && id_buff[1] == ID_H_VAL){
			sp.ps_en[i] = PS_EN;
			HAL_I2C_Mem_Write(hi2c, VCNL4040_ADDR, PS_CONF3, 1, init_buff, 2, HAL_MAX_DELAY);//LED setting
			HAL_I2C_Mem_Write(hi2c, VCNL4040_ADDR, PS_CONF1, 1, start_buff, 2, HAL_MAX_DELAY);//Turn on LED
		}else{
			sp.ps_en[i] = PS_NOT_EN;
		}

	}
}

void ps_update(I2C_HandleTypeDef *hi2c){
	uint8_t start_buff[2];
	uint8_t stop_buff[2];
	uint8_t data[2];
	uint8_t ps_ret = 0xff;
	start_buff[0] = 0x0e;
	start_buff[1] = 0x08;
	stop_buff[0] = 0x01;
	stop_buff[1] = 0x00;

	for (int i = 0; i < PS_CHANNEL_NUM; i++){
		data[0] = 0x00;
		data[1] = 0x00;

		if(sp.ps_en[i] == PS_EN){
			/*
			if(sp.board_select == SELECT_revA){
				ps_select_channel(hi2c, PS_CHANNEL_ARRAY_PCA9458[i]);
			}else if(sp.board_select == SELECT_revB){
				ps_select_channel(hi2c, PS_CHANNEL_ARRAY_PCA9457[i]);
			}*/

			//HAL_I2C_Mem_Write(hi2c1, VCNL4040_ADDR, PS_CONF1, 1, start_buff, 2, HAL_MAX_DELAY);//Turn on LED
			//HAL_Delay(10);
			//HAL_I2C_Mem_Read(hi2c1, VCNL4040_ADDR, PS_DATA_L, 1, data, 2, 1);
			//ps_ret = HAL_I2C_Mem_Read(hi2c, VCNL4040_ADDR, PS_DATA_L, 1, data, 2, HAL_MAX_DELAY);
#if !PS_I2C_DMA
			ps_ret = HAL_I2C_Mem_Read(hi2c, VCNL4040_ADDR, PS_DATA_L, 1, data, 2, 1);
			if(ps_ret == HAL_OK){
				sp.ps_print[i] = (uint16_t)(data[1] << 8 | data[0]);

				sp.ps[i * 2] = data[1];

				sp.ps[i * 2 + 1] = data[0];
			}
			HAL_Delay(1);
#else
			sp.i2c1_dma_flag = I2C1_DMA_PS;
			HAL_I2C_Mem_Read_DMA(hi2c, VCNL4040_ADDR, PS_DATA_L, 1, sp.ps_dma, 2);
#endif
		}
		//HAL_I2C_Mem_Write(hi2c1, VCNL4040_ADDR, PS_CONF1, 1, stop_buff, 2, HAL_MAX_DELAY);//Turn off LED
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	switch(sp.i2c1_dma_flag){
	case 0:
		if((sp.gyro[0] != 0) || (sp.gyro[1] != 0) || (sp.gyro[2] != 0)){
			sp.gyro_print[0] = (int16_t)(sp.gyro[0] << 8 | sp.gyro[1]);
			sp.gyro_print[1] = (int16_t)(sp.gyro[2] << 8 | sp.gyro[3]);
			sp.gyro_print[2] = (int16_t)(sp.gyro[4] << 8 | sp.gyro[5]);
		}
		//osDelay(1);
		delayUs(100);
		imu_update_i2c_DMA_gyro(hi2c);
		//imu_update_i2c_DMA_acc(hi2c);
		break;
	case 1:
		if((sp.acc[0] != 0) || (sp.acc[1] != 0) || (sp.acc[2] != 0)){
			sp.acc_print[0] = (int16_t)(sp.acc[0] << 8 | sp.acc[1]);
			sp.acc_print[1] = (int16_t)(sp.acc[2] << 8 | sp.acc[3]);
			sp.acc_print[2] = (int16_t)(sp.acc[4] << 8 | sp.acc[5]);
		}
		imu_update_i2c_DMA_gyro(hi2c);
		break;
	case 2:
		sp.ps_print[0] = (uint16_t)(sp.ps_dma[1] << 8 | sp.ps_dma[0]);
		sp.ps[0 * 2] = sp.ps_dma[1];
		sp.ps[0 * 2 + 1] = sp.ps_dma[0];
		ps_update(hi2c);
		break;
	}
}
