/*
 * sensor.c
 *
 *  Created on: 2022/06/01
 *      Author: makabe
 */

#include "sensor.h"
// #include "cmsis_os.h"

struct sensor_params sp;

void txbuff_update(){//max: uint8_t * 44: 44-(8+6+6+2+16)=6
	uint8_t index = 0;
	for(int i=0; i < MIC_CHANNEL_NUM; i++){//4*4=16
		sp.txbuff[index] = (sp.i2s_buff_sifted[i * 2] >> 10) & 0x000000ff;
		sp.txbuff[index + 1] = (sp.i2s_buff_sifted[i * 2] >> 2)& 0x000000ff;
		index += 2;
	}
	for(int i=0; i < ADC_CHANNEL_NUM; i++){//2*4=8
		sp.txbuff[index] = sp.adc_print[i] >> 8;
		sp.txbuff[index + 1] = sp.adc_print[i] & 0x00ff;
		index += 2;
	}
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

void delayUs(uint16_t micros) {
	uint32_t start = getUs();
	while (getUs()-start < (uint32_t) micros) {
		asm("nop");
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
	start_buff[0] = 0xce;//1/320,8T
	//start_buff[0] = 0x0e;//1/40,8T
	//start_buff[0] = 0x0c;//1/40,4T
	//start_buff[0] = 0x04;//1/40,2T
	//start_buff[0] = 0x00;//1/40,1T
	start_buff[1] = 0x08;//16bit

	HAL_I2C_Mem_Read(hi2c, VCNL4040_ADDR, ID_L, 1, id_buff, 2, HAL_MAX_DELAY);//check sensor ID
	// HAL_I2C_Mem_Read_DMA(hi2c, VCNL4040_ADDR, ID_L, 1, id_buff, 2);//check sensor ID

	if(id_buff[0] == ID_L_VAL && id_buff[1] == ID_H_VAL){
		sp.ps_en[0] = PS_EN;
		HAL_I2C_Mem_Write(hi2c, VCNL4040_ADDR, PS_CONF3, 1, init_buff, 2, HAL_MAX_DELAY);//LED setting
		// HAL_I2C_Mem_Write_DMA(hi2c, VCNL4040_ADDR, PS_CONF3, 1, init_buff, 2);//LED setting
		HAL_I2C_Mem_Write(hi2c, VCNL4040_ADDR, PS_CONF1, 1, start_buff, 2, HAL_MAX_DELAY);//Turn on LED
	}else{
		sp.ps_en[0] = PS_NOT_EN;
	}
}

void ps_update(I2C_HandleTypeDef *hi2c){
	uint8_t start_buff[2];
	uint8_t stop_buff[2];
	uint8_t data[2];
	start_buff[0] = 0x0e;
	start_buff[1] = 0x08;
	stop_buff[0] = 0x01;
	stop_buff[1] = 0x00;

	data[0] = 0x00;
	data[1] = 0x00;

	if(sp.ps_en[0] == PS_EN){
		//HAL_I2C_Mem_Read(hi2c1, VCNL4040_ADDR, PS_DATA_L, 1, data, 2, 1);
		int a = 0;
		// HAL_I2C_Mem_Read(hi2c, VCNL4040_ADDR, PS_DATA_L, 1, data, 2, HAL_MAX_DELAY);
		// HAL_I2C_Mem_Read(hi2c, VCNL4040_ADDR, PS_DATA_L, 1, sp.ps, 2, HAL_MAX_DELAY);
		//HAL_I2C_Mem_Read_DMA(hi2c, VCNL4040_ADDR, PS_DATA_L, 1, data, 2);
		HAL_I2C_Mem_Read_DMA(hi2c, VCNL4040_ADDR, PS_DATA_L, 1, sp.ps_dma, 2);
	}
}
