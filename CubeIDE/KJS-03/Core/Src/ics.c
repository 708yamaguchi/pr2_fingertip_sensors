/*
 * ics.c
 *
 *  Created on: 2019/06/27
 *      Author: sugai
 */

#include "ics.h"

struct servo_params sp;
uint8_t recv_buf;
uint32_t current_time;
uint32_t receive_time1;
uint32_t receive_time2;
uint32_t receive_time3;
UART_HandleTypeDef huart6;
uint8_t flagRcved;
//uint8_t getlen();
uint32_t UART_IsEnabledIT_RX();

// dma buffer
//UART_HandleTypeDef *huart_cobs;

uint32_t getUs(void) {
	uint32_t usTicks = HAL_RCC_GetSysClockFreq() / 1000000;
	register uint32_t ms, cycle_cnt;
	do {
		ms = HAL_GetTick();
		cycle_cnt = SysTick->VAL;
	} while (ms != HAL_GetTick());
	return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

uint8_t getlen(uint8_t sc){
	uint8_t len = 2;
	len += ((sc & SC_SW_ADC) == SC_SW_ADC) * 2 * ADC_CHANNEL_NUM;
	len += ((sc & SC_PS) == SC_PS) * 3 * PS_CHANNEL_NUM;
	len += ((sc & SC_GYRO) == SC_GYRO) * 3 * GYRO_CHANNEL_NUM;
	len += ((sc & SC_ACC) == SC_ACC) * 3 * ACC_CHANNEL_NUM;
	if(len > MAX_TXBUFF_LEN){
		len = MAX_TXBUFF_LEN;
	}
	return len;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	uint8_t sensor_flag;
	//uint8_t send_length;
	uint8_t ret;
	//sp.state = 1;
	//flagRcved = IT_FLAG_TRUE;
	if(huart->Instance==USART6){
		//sp.state = 2;
    	if(((recv_buf & 0xE0) == 0xA0) && (recv_buf & 0x1f) == sp.id){
    		//sp.state = 3;
			//ret = HAL_UART_Receive(huart, &(recv_data[1]), 1, HAL_MAX_DELAY);
			ret = HAL_UART_Receive(huart, &sensor_flag, 1, HAL_MAX_DELAY);
			if(ret == HAL_OK){
				if ((sensor_flag & SC_NONE) == SC_NONE){
					sp.mode = sensor_flag;
		    		if(sp.it_count < 4096){
		    			sp.it_count += 1;
		    		}else{
		    			sp.it_count = 0;
		    		}
				}
				sp.txbuff[1] = sensor_flag;
				HAL_HalfDuplex_EnableTransmitter(huart);
				ret = HAL_UART_Transmit(huart, sp.txbuff, getlen(sp.mode), 1);
				HAL_HalfDuplex_EnableReceiver(huart);
				HAL_UART_Receive_IT(huart, &recv_buf, 1);
			}
		}
	}
	//flagRcved = IT_FLAG_FALSE;
}

//#define MAX_TXBUFF_LEN (2 + 2 * ADC_CHANNEL_NUM + 3 * PS_CHANNEL_NUM + 3 * GYRO_CHANNEL_NUM + 3 * ACC_CHANNEL_NUM)

void txbuff_update(){
	uint8_t sensor_flag = sp.mode;
	uint8_t index = 2;
	//sp.txbuff[1] = sensor_flag;
#if RX_IT_DEBUG
	sp.adc_print[0] = sp.it_count;
	sp.adc_print[1] = sp.uart_error_count;
	sp.adc_print[2] = sp.com_en;
	sp.adc_print[3] = 1000 + sp.ics_timeout_count;

	sp.gyro_print[0] = sp.uart_error[0];
	sp.gyro_print[1] = sp.uart_error[1];
	sp.gyro_print[2] = sp.uart_error[2];
	sp.acc_print[0] = sp.uart_error[3];
	sp.acc_print[1] = (uint16_t) (sp.check2com_delay / 1000);//[ms]
#endif
	if((sensor_flag & SC_SW_ADC) == SC_SW_ADC){
		for(int i=0; i < ADC_CHANNEL_NUM; i++){
			sp.txbuff[index] = ((sp.adc_print[i] >> 7 | sp.sw << 5) & 0x7f);
			sp.txbuff[index + 1] = sp.adc_print[i] & 0x007f;
			index += 2;
		}
	}
	if((sensor_flag & SC_PS) == SC_PS){
		for(int i=0; i < PS_CHANNEL_NUM; i++){
			sp.txbuff[index] = (sp.ps_print[i] >> 14) & 0x7f;
			sp.txbuff[index + 1] = (sp.ps_print[i] >> 7) & 0x7f;
			sp.txbuff[index + 2] = sp.ps_print[i] & 0x007f;
			index += 3;
		}
	}
	if((sensor_flag & SC_GYRO) == SC_GYRO){
		for(int i=0; i < GYRO_CHANNEL_NUM; i++){
			sp.txbuff[index] = (sp.gyro_print[i] >> 14) & 0x7f;
			sp.txbuff[index + 1] = (sp.gyro_print[i] >> 7) & 0x7f;
			sp.txbuff[index + 2] = sp.gyro_print[i] & 0x007f;
			index += 3;
		}
	}
	if((sensor_flag & SC_ACC) == SC_ACC){
		for(int i=0; i < ACC_CHANNEL_NUM; i++){
			sp.txbuff[index] = (sp.acc_print[i] >> 14) & 0x7f;
			sp.txbuff[index + 1] = (sp.acc_print[i] >> 7) & 0x7f;
			sp.txbuff[index + 2] = sp.acc_print[i] & 0x007f;
			index += 3;
		}
	}
	for(int i = 0; i < MAX_TXBUFF_LEN; i++) {
		sp.txbuff[i] &= 0x7f;
	}
}

void delayUs(uint16_t micros) {
	uint32_t start = getUs();
	while (getUs()-start < (uint32_t) micros) {
		asm("nop");
	}
}

void mpuWrite(SPI_HandleTypeDef *hspi1, uint8_t address, uint8_t value)
{
	HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi1, &address, 1, 1000);
	HAL_SPI_Transmit(hspi1, &value, 1, 1000);
	HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
}

void mpuRead(SPI_HandleTypeDef *hspi1, uint8_t address, uint8_t value)
{
	HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi1, &address, 1, 1000);
	HAL_SPI_Receive(hspi1, &value, 1, 1000);
	HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
}

void imu_init(SPI_HandleTypeDef *hspi1){

	uint8_t t_data;
	uint8_t who = 0x00;

	switch(sp.imu_select){
	case 0:
		t_data = ICM_20600_PING_ADDRESS | 0x80;
		break;
	case 1:
		t_data = ICM_42605_PING_ADDRESS | 0x80;
		break;
	}

	HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi1, &t_data, 1, 1000);
	HAL_SPI_Receive(hspi1, &who, 1, 1000);
	HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);

	if((who == IMU_WHO_AM_I_20600  && sp.imu_select == SELECT_ICM_20600) || (who == IMU_WHO_AM_I_42605  && sp.imu_select == SELECT_ICM_42605)){
		sp.imu_en = IMU_EN;
		switch(sp.imu_select){
		case 0:
			//reset-device
			mpuWrite(hspi1,ICM_20600_PWR_MGMT_1,ICM_20600_PWR_MGMT_1_VAL);//important setting
			HAL_Delay(1);

			//init-gyro
			mpuWrite(hspi1, ICM_20600_CONFIG, ICM_20600_GYRO_DLPF_CFG); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
			//mpuWrite(hspi1, 0x1A, 0x00); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
			HAL_Delay(1); //very importnat! between gyro and acc
			mpuWrite(hspi1, ICM_20600_GYRO_CONFIG, ICM_20600_GYRO_CONFIG_VAL); //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
			HAL_Delay(10); //very importnat! between gyro and acc

			//init-acc
			mpuWrite(hspi1, ICM_20600_ACCEL_CONFIG, ICM_20600_ACCEL_CONFIG_VAL); //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
			HAL_Delay(1);
			mpuWrite(hspi1, ICM_20600_ACCEL_CONFIG2, ICM_20600_ACC_DLPF_CFG);
			HAL_Delay(10);
			break;
		case 1:
			//set-gyro-scale-and-ODR
			mpuWrite(hspi1, ICM_42605_GYRO_CONFIG0, ICM_42605_GYRO_CONFIG0_VAL); //GYRO_FS_SEL = 0: Full scale set to 2000 deg/sec, 1kHz ODR
			HAL_Delay(10); //very importnat! between gyro and acc

			//set-acc-scale-and-ODR
			mpuWrite(hspi1, ICM_42605_ACCEL_CONFIG0, ICM_42605_ACCEL_CONFIG0_VAL);//ACCEL_FS_SEL = 0: Full scale set to +/-16G, 1kHz ODR
			HAL_Delay(10); //very importnat! between gyro and acc

			//set-gyro-acc-LPF
			mpuWrite(hspi1, ICM_42605_GYRO_ACCEL_CONFIG0, ICM_42605_GYRO_ACCEL_CONFIG0_VAL); //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
			HAL_Delay(10);

			//start gyro and acc
			mpuWrite(hspi1, ICM_42605_PWR_MGMT0, ICM_42605_PWR_MGMT0_VAL);//Turn on Gyro and Acc with Low Noise Mode
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
		gyro_t_data[0] = ICM_20600_GYRO_ADDRESS | 0x80;
		acc_t_data[0] = ICM_20600_ACC_ADDRESS | 0x80;
		break;
	case 1:
		gyro_t_data[0] = ICM_42605_GYRO_ADDRESS | 0x80;
		acc_t_data[0] = ICM_42605_ACC_ADDRESS | 0x80;
		break;
	}

	if(sp.imu_en == IMU_EN){
		HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
		HAL_SPI_Transmit(hspi1, gyro_t_data, 1, 10);
		HAL_SPI_Receive(hspi1, sp.gyro, 6, 10);
		HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);

		if((sp.gyro[0] != 0) || (sp.gyro[1] != 0) || (sp.gyro[2] != 0)){
			sp.gyro_print[0] = (int16_t)(sp.gyro[0] << 8 | sp.gyro[1]);
			sp.gyro_print[1] = (int16_t)(sp.gyro[2] << 8 | sp.gyro[3]);
			sp.gyro_print[2] = (int16_t)(sp.gyro[4] << 8 | sp.gyro[5]);
		}

		HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
		HAL_SPI_Transmit(hspi1, acc_t_data, 1, 10);
		HAL_SPI_Receive(hspi1, sp.acc, 6, 10);
		HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);

		if((sp.acc[0] != 0) || (sp.acc[1] != 0) || (sp.acc[2] != 0)){
			sp.acc_print[0] = (int16_t)(sp.acc[0] << 8 | sp.acc[1]);
			sp.acc_print[1] = (int16_t)(sp.acc[2] << 8 | sp.acc[3]);
			sp.acc_print[2] = (int16_t)(sp.acc[4] << 8 | sp.acc[5]);
		}
	}
}

void ps_select_channel(I2C_HandleTypeDef *hi2c, uint8_t ch){
	uint8_t tx;
	if(sp.board_select == SELECT_revA){
		tx = ch;//PCA9548 format
		HAL_I2C_Master_Transmit(hi2c, PCA9548_ADDR, &tx, 1, HAL_MAX_DELAY);//select i2c channel
	}else if(sp.board_select == SELECT_revB){
		tx = 0x08 | ch;//PCA9547 format
		HAL_I2C_Master_Transmit(hi2c, PCA9547_ADDR, &tx, 1, HAL_MAX_DELAY);//select i2c channel
	}

}

void ps_init(I2C_HandleTypeDef *hi2c){
	uint8_t init_buff[2];
	uint8_t start_buff[2];
	uint8_t id_buff[2];
	uint8_t ret1 = 0xff;
	uint8_t ret2 = 0xff;
	id_buff[0] = 0x00;
	id_buff[1] = 0x00;
	init_buff[0] = 0x00;
	//init_buff[1] = 0x06;//180[mA]
	//init_buff[1] = 0x02;//100[mA]
	init_buff[1] = 0x00;//50[mA]
	start_buff[0] = 0xce;//1/320,8T
	//start_buff[0] = 0x0e;//1/40,8T
	//start_buff[0] = 0x0c;//1/40,4T
	//start_buff[0] = 0x04;//1/40,2T
	//start_buff[0] = 0x00;//1/40,1T
	start_buff[1] = 0x08;//16bit

	for (int i = 0; i < PS_CHANNEL_NUM; i++){
		if(sp.board_select == SELECT_revA){
			ps_select_channel(hi2c, PS_CHANNEL_ARRAY_PCA9458[i]);
		}else if(sp.board_select == SELECT_revB){
			ps_select_channel(hi2c, PS_CHANNEL_ARRAY_PCA9457[i]);
		}

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
			if(sp.board_select == SELECT_revA){
				ps_select_channel(hi2c, PS_CHANNEL_ARRAY_PCA9458[i]);
			}else if(sp.board_select == SELECT_revB){
				ps_select_channel(hi2c, PS_CHANNEL_ARRAY_PCA9457[i]);
			}

			//HAL_I2C_Mem_Write(hi2c1, VCNL4040_ADDR, PS_CONF1, 1, start_buff, 2, HAL_MAX_DELAY);//Turn on LED
			//HAL_Delay(10);
			//HAL_I2C_Mem_Read(hi2c1, VCNL4040_ADDR, PS_DATA_L, 1, data, 2, 1);
			ps_ret = HAL_I2C_Mem_Read(hi2c, VCNL4040_ADDR, PS_DATA_L, 1, data, 2, HAL_MAX_DELAY);

			if(ps_ret == HAL_OK){
				sp.ps_print[i] = (uint16_t)(data[1] << 8 | data[0]);

				sp.ps[i * 2] = data[1];

				sp.ps[i * 2 + 1] = data[0];
			}
		}
		//HAL_I2C_Mem_Write(hi2c1, VCNL4040_ADDR, PS_CONF1, 1, stop_buff, 2, HAL_MAX_DELAY);//Turn off LED
	}
}

void ps_update_single(I2C_HandleTypeDef *hi2c1){
	uint8_t start_buff[2];
	uint8_t stop_buff[2];
	uint8_t data[2];
	start_buff[0] = 0x0e;
	start_buff[1] = 0x08;
	stop_buff[0] = 0x01;
	stop_buff[1] = 0x00;

	data[0] = 0x00;

	data[1] = 0x00;

	HAL_I2C_Mem_Write(hi2c1, VCNL4040_ADDR, PS_CONF1, 1, start_buff, 2, HAL_MAX_DELAY);

	HAL_Delay(5);

	HAL_I2C_Mem_Read(hi2c1, VCNL4040_ADDR, PS_DATA_L, 1, data, 2, HAL_MAX_DELAY);

	sp.ps_print[0] = (uint16_t)(data[1] << 8 | data[0]);

	sp.ps[0] = data[1];

	sp.ps[1] = data[0];

	HAL_I2C_Mem_Write(hi2c1, VCNL4040_ADDR, PS_CONF1, 1, stop_buff, 2, HAL_MAX_DELAY);
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

void sw_update(){
	sp.sw = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15);
}

void groove_i2c_update(I2C_HandleTypeDef *hi2c1){
	uint8_t i2c_state_ = 0;
	uint8_t n_results_ = 0;
	uint8_t ADDR_CTRL_REG = 0x00;
	uint8_t ADDR_OBJ_DATA_LEN = 0x01;
	uint8_t ADDR_OBJ_DATA = 0x02;
	uint8_t m5_addr = 0x24;

	uint8_t detect_type;
	uint16_t box_x;
	uint16_t box_y;
	uint16_t box_w;
	uint16_t box_h;

	ps_select_channel(hi2c1, 0x80);//ch7 = 0x80

	while (hi2c1->State == HAL_BUSY) {}
	HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(hi2c1, (uint16_t)(m5_addr<<1), 2, 50);

	if (result != HAL_OK) {
		i2c_state_ = 0;
		n_results_ = 0;
		return;
	}

	i2c_state_ = 1;
	//HAL_I2C_Mem_Read(hi2c1, VCNL4040_ADDR, PS_DATA_L, 1, data, 2, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(hi2c1, m5_addr << 1, ADDR_OBJ_DATA_LEN, I2C_MEMADD_SIZE_8BIT, &n_results_, 1, HAL_MAX_DELAY);
	//readFrom(ADDR_OBJ_DATA_LEN, m5_addr, &n_results_, 1, HAL_MAX_DELAY, &hi2c1);

	i2c_state_ = 1;

	if (n_results_ == 0) {
		return;
	}
	if (n_results_ > M5STICKV_DETECTION_RESULT_LENGTH) {
		n_results_ = M5STICKV_DETECTION_RESULT_LENGTH;
	}
	HAL_I2C_Mem_Read(hi2c1, m5_addr << 1, ADDR_OBJ_DATA, I2C_MEMADD_SIZE_8BIT, sp.detection_result, n_results_, HAL_MAX_DELAY);
	detect_type = sp.detection_result[0];
	box_x = sp.detection_result[2] << 8 | sp.detection_result[1];
	box_y = sp.detection_result[4] << 8 | sp.detection_result[3];
	box_w = sp.detection_result[6] << 8 | sp.detection_result[5];
	box_h = sp.detection_result[8] << 8 | sp.detection_result[7];
	i2c_state_ = 1;
	//readFrom(ADDR_OBJ_DATA, m5_addr, sp.detection_result, n_results_, HAL_MAX_DELAY, &hi2c1);
}

void readFrom(uint8_t address, uint8_t i2c_address, uint8_t* byte_array, uint8_t n_bytes, uint16_t timeout, I2C_HandleTypeDef *hi2c1) {
	while (hi2c1->State == HAL_BUSY) {}
	HAL_I2C_Mem_Read(hi2c1, i2c_address << 1, address, I2C_MEMADD_SIZE_8BIT, byte_array, n_bytes, timeout);
}

uint32_t UART_IsEnabledIT_RX(UART_HandleTypeDef *huart){
#ifdef USART_CR1_FIFOEN
	return ((READ_BIT(huart->Instance->CR1, USART_CR1_RXNEIE_RXFNEIE) == (USART_CR1_RXNEIE_RXFNEIE)) ? 1U : 0U);
#else
	return ((READ_BIT(huart->Instance->CR1, USART_CR1_RXNEIE) == (USART_CR1_RXNEIE)) ? 1U : 0U);
#endif
}

void ics_init(UART_HandleTypeDef *huart){
	memset(sp.txbuff, 0, sizeof(sp.txbuff));
	sp.txbuff[0] = 0x20 | sp.id;
	//__HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
}

void HAL_UART_RxCpltCallback_old(UART_HandleTypeDef *huart){
	volatile uint8_t data_length;
	uint8_t recv_data[2];
	uint8_t send_length = 0;
	uint8_t send_data[40] = {};//1+1+2*4+3*4+3*3+3*3 = 40
	uint8_t ret;
	sp.state = 1;
	flagRcved = IT_FLAG_TRUE;

	if(huart->Instance==USART6){
		sp.state = 2;
		//receive_time2 = getUs();

    	if(((recv_buf & 0xE0) == 0xA0) && (recv_buf & 0x1f) == sp.id){
    		//CLEAR_BIT(huart->Instance->CR1, USART_CR1_RXNEIE);
    		//__disable_irq();
    		//huart->RxState=HAL_UART_STATE_READY;

    		//receive_time3 = getUs();
    		data_length = 1;
    		sp.state = 3;
			recv_data[0] = recv_buf;
			recv_data[1] = 0xee;
			//recv_data[2] = 0xee;
			//recv_data[3] = 0xee;
			//recv_data[4] = 0xee;
			//ret = HAL_UART_Receive(huart, &(recv_data[1]), data_length, HAL_MAX_DELAY);
			ret = HAL_UART_Receive(huart, &(recv_data[1]), data_length, 1);
#if USE_SEMIHOSTING
			if(ret != HAL_OK){
				printf("rx error\r\n");
			}
			printf("%d\r\n",recv_data[1]);
#endif
			if(ret == HAL_OK){
				HAL_HalfDuplex_EnableTransmitter(huart);

#if RX_IT_DEBUG
				if(sp.it_count < 4096){
					sp.it_count += 1;
				}
				sp.adc_print[0] = sp.it_count;
				sp.adc_print[1] = sp.uart_error_count;
				sp.adc_print[2] = 100;

				//sp.gyro_print[0] = huart6.Lock;
				//sp.gyro_print[1] = huart6.gState;
				//sp.gyro_print[2] = huart6.RxState;
				//sp.acc_print[0] = huart6.ErrorCode & 0x000000ff;
				sp.gyro_print[0] = sp.uart_error[0];
				sp.gyro_print[1] = sp.uart_error[1];
				sp.gyro_print[2] = sp.uart_error[2];
				sp.acc_print[0] = sp.uart_error[3];
#endif
				send_data[0] = recv_data[0] & 0x7f;

				send_data[1] = recv_data[1];
				uint8_t sensor_flag = recv_data[1];
				if ((sensor_flag & SC_NONE) == SC_NONE){
					sp.mode = sensor_flag;
				}
				uint8_t index = 2;
				if((sensor_flag & SC_SW_ADC) == SC_SW_ADC){
					for(int i=0; i < ADC_CHANNEL_NUM; i++){
						send_data[index] = ((sp.adc_print[i] >> 7 | sp.sw << 5) & 0x7f);
						send_data[index + 1] = sp.adc_print[i] & 0x007f;
						index += 2;
					}
				}
				if((sensor_flag & SC_PS) == SC_PS){
					for(int i=0; i < PS_CHANNEL_NUM; i++){
						send_data[index] = (sp.ps_print[i] >> 14) & 0x7f;
						send_data[index + 1] = (sp.ps_print[i] >> 7) & 0x7f;
						send_data[index + 2] = sp.ps_print[i] & 0x007f;
						index += 3;
					}
				}
				if((sensor_flag & SC_GYRO) == SC_GYRO){
					for(int i=0; i < GYRO_CHANNEL_NUM; i++){
						send_data[index] = (sp.gyro_print[i] >> 14) & 0x7f;
						send_data[index + 1] = (sp.gyro_print[i] >> 7) & 0x7f;
						send_data[index + 2] = sp.gyro_print[i] & 0x007f;
						index += 3;
					}
				}
				if((sensor_flag & SC_ACC) == SC_ACC){
					for(int i=0; i < ACC_CHANNEL_NUM; i++){
						send_data[index] = (sp.acc_print[i] >> 14) & 0x7f;
						send_data[index + 1] = (sp.acc_print[i] >> 7) & 0x7f;
						send_data[index + 2] = sp.acc_print[i] & 0x007f;
						index += 3;
					}
				}
				send_length = index;
				//sp.past_length = send_length;

				for(int i = 0; i < send_length; i++) {
					send_data[i] = send_data[i] & 0x7f;
				}
				//HAL_HalfDuplex_EnableTransmitter(huart);
				ret = HAL_UART_Transmit(huart, send_data, send_length,HAL_MAX_DELAY);
				HAL_HalfDuplex_EnableReceiver(huart);

				//__enable_irq();
				//HAL_UART_Receive_IT(&huart6, &recv_buf, 1);
				//if((UART_IsEnabledIT_RX(huart) == 0)){
					  //HAL_UART_Receive_IT(&huart6, &recv_buf, 1);
				//}
				//HAL_UART_Receive_IT(&huart6, &recv_buf, 1);
				//receive_time1 = getUs();
				//HAL_Delay(100);
#if USE_SEMIHOSTING
			if(ret != HAL_OK){
				printf("tx error\r\n");
			}
#endif
			}
		}
	}
	flagRcved = IT_FLAG_FALSE;
}

