/*
 * wt61pc.c
 *
 *  Created on: Jul 12, 2023
 *      Author: Kadek Wirawan Suryajaya
 *
 *  Driver for DFRobot Serial 6-Axis Accelerometer
 *	Essentially WT61PC
 *	WT61PC only gives acceleration, angle position, angle velocity, and temperature
 *
 *	Protocol:
 *	https://drive.google.com/file/d/1xrfK9bAEncgFQYjvT_c6vwSEH0ZhzaUZ/view
 */

#include "wt61pc.h"
#include "string.h"

static uint8_t WT61PC_checksum_calc();
static int16_t to_i16(uint8_t dataL, uint8_t dataH);

WT61PC_t imu;
static WIT_TX_Command tx;
uint8_t frame_unlock[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};

/**
 * @brief Start UART Peripheral
 * 		  Call before loop
 * 		  Set rx to DMA normal
 * 		  Enable global interrupt
 *
 * @param huart
 */
void WT61PC_init(UART_HandleTypeDef* huart){
	HAL_UART_Receive_DMA(huart, &imu.rx_byte, 1);

	imu.huart = huart;
}

/**
 * @brief Callback everytime a byte is received
 *		  Call in HAL_UART_RxCpltCallback
 *
 * @param huart
 */
void WT61PC_rx_callback(UART_HandleTypeDef* huart){
	if(huart->Instance != imu.huart->Instance) return;

	HAL_UART_Receive_DMA(huart, &imu.rx_byte, 1);

	if(imu.rx_ptr == 11){	//frame complete
		imu.rx_ptr = 0;

		//checksum calculation wrong
		if(imu.rx_frame[10] != WT61PC_checksum_calc()) return;

		if(imu.rx_frame[1] == 0x51){	//acceleration
			imu.acceleration.x = to_i16(imu.rx_frame[2], imu.rx_frame[3]) * 16.0 / 32768;
			imu.acceleration.y = to_i16(imu.rx_frame[4], imu.rx_frame[5]) * 16.0 / 32768;
			imu.acceleration.z = to_i16(imu.rx_frame[6], imu.rx_frame[7]) * 16.0 / 32768;

			imu.temperature = to_i16(imu.rx_frame[8], imu.rx_frame[9]) / 100.0;
		}
		else if(imu.rx_frame[1] == 0x52){		//angular velocity
			imu.angle_vel.x = to_i16(imu.rx_frame[2], imu.rx_frame[3]) * 2000.0 / 32768;
			imu.angle_vel.y = to_i16(imu.rx_frame[4], imu.rx_frame[5]) * 2000.0 / 32768;
			imu.angle_vel.z = to_i16(imu.rx_frame[6], imu.rx_frame[7]) * 2000.0 / 32768;

			imu.voltage = to_i16(imu.rx_frame[8], imu.rx_frame[9]) / 100.0;

		}
		else if(imu.rx_frame[1] == 0x53){		//angular position
			imu.angle_pos.x = to_i16(imu.rx_frame[2], imu.rx_frame[3]) * 180.0 / 32768;
			imu.angle_pos.y = to_i16(imu.rx_frame[4], imu.rx_frame[5]) * 180.0 / 32768;
			imu.angle_pos.z = to_i16(imu.rx_frame[6], imu.rx_frame[7]) * 180.0 / 32768;

			imu.version = to_i16(imu.rx_frame[8], imu.rx_frame[9]);

			static uint32_t prev_tick = 0;
			imu.T_ms = HAL_GetTick() - prev_tick;
			prev_tick = HAL_GetTick();
		}
		else if(imu.rx_frame[1] == 0x54){		//magnetic field
			imu.magnetic_field.x = to_i16(imu.rx_frame[2], imu.rx_frame[3]);
			imu.magnetic_field.y = to_i16(imu.rx_frame[4], imu.rx_frame[5]);
			imu.magnetic_field.z = to_i16(imu.rx_frame[6], imu.rx_frame[7]);

			imu.temperature = to_i16(imu.rx_frame[8], imu.rx_frame[9]) / 100.0;
		}
		else if(imu.rx_frame[1] == 0x5F){		//read value
			memcpy(tx.register_data, imu.rx_frame, 11);
		}


		imu.timestamp = HAL_GetTick();
	}

	//data unsync
	if(imu.rx_ptr == 0 && imu.rx_byte != 0x55) return;

	imu.rx_frame[imu.rx_ptr] = imu.rx_byte;
	imu.rx_ptr++;
}

/**
 * @brief Routine to check IMU serial connection
 * 		  Call periodically, ideally in main while(1)
 */
void WT61PC_rx_restart(){
	static int last_restart = 0;

	if(HAL_GetTick() < imu.timestamp + 100) return;
	if(HAL_GetTick() < last_restart + 100) return;

	last_restart = HAL_GetTick();

	imu.rx_ptr = 0;
	HAL_UART_Receive_DMA(&huart1, &imu.rx_byte, 1);
}

/**
 * @brief Checksum calculation
 *
 * @return
 */
static uint8_t WT61PC_checksum_calc(){
	uint8_t checksum_res = 0;
	for(int i = 0; i < 10; i++){
		checksum_res += imu.rx_frame[i];
	}

	return checksum_res;
}

/**
 * @brief Convert 2 byte into 16 bit integer
 *
 * @param dataL
 * @param dataH
 * @return
 */
static int16_t to_i16(uint8_t dataL, uint8_t dataH){
	int16_t data = ((dataH << 8) | dataL);

	return data;
}

void WT61PC_tx_routine(){
	if(tx.ctr == 1){
		HAL_UART_Transmit_DMA(imu.huart, frame_unlock, 5);
		tx.timestamp = HAL_GetTick();
		tx.ctr = 2;
	}

	if(tx.ctr == 2 && HAL_GetTick() > tx.timestamp + 10){
		HAL_UART_Transmit_DMA(imu.huart, tx.tx_frame, 5);
		tx.timestamp = HAL_GetTick();
		tx.ctr = 3;
	}

	if(tx.ctr == 3 && HAL_GetTick() > tx.timestamp + 10){
		tx.ctr = 0;
	}
}

HAL_StatusTypeDef WIT_save(WIT_SAVE_Cmd cmd){
	if(tx.ctr != 0) return HAL_BUSY;

	tx.ctr = 1;
	tx.tx_frame[0] = 0xFF;
	tx.tx_frame[1] = 0xAA;
	tx.tx_frame[2] = WIT_REG_SAVE;
	tx.tx_frame[3] = (cmd >> 0) & 0xFF;
	tx.tx_frame[4] = (cmd >> 8) & 0xFF;

	return HAL_OK;
}

HAL_StatusTypeDef WIT_calibrate(WIT_CALSW_Cmd cmd){
	if(tx.ctr != 0) return HAL_BUSY;

	tx.ctr = 1;
	tx.tx_frame[0] = 0xFF;
	tx.tx_frame[1] = 0xAA;
	tx.tx_frame[2] = WIT_REG_CALSW;
	tx.tx_frame[3] = (cmd >> 0) & 0xFF;
	tx.tx_frame[4] = (cmd >> 8) & 0xFF;

	return HAL_OK;
}

HAL_StatusTypeDef WIT_set_output_content(uint8_t out_content){
	if(tx.ctr != 0) return HAL_BUSY;

	tx.ctr = 1;
	tx.tx_frame[0] = 0xFF;
	tx.tx_frame[1] = 0xAA;
	tx.tx_frame[2] = WIT_REG_RSW;
	tx.tx_frame[3] = (out_content >> 0) & 0xFF;
	tx.tx_frame[4] = (out_content >> 8) & 0xFF;

	return HAL_OK;
}

HAL_StatusTypeDef WIT_set_output_rate(WIT_RRATE_Cmd cmd){
	if(tx.ctr != 0) return HAL_BUSY;

	tx.ctr = 1;
	tx.tx_frame[0] = 0xFF;
	tx.tx_frame[1] = 0xAA;
	tx.tx_frame[2] = WIT_REG_RRATE;
	tx.tx_frame[3] = (cmd >> 0) & 0xFF;
	tx.tx_frame[4] = (cmd >> 8) & 0xFF;

	return HAL_OK;
}

HAL_StatusTypeDef WIT_set_baud_rate(WIT_BAUD_Cmd cmd){
	if(tx.ctr != 0) return HAL_BUSY;

	tx.ctr = 1;
	tx.tx_frame[0] = 0xFF;
	tx.tx_frame[1] = 0xAA;
	tx.tx_frame[2] = WIT_REG_BAUD;
	tx.tx_frame[3] = (cmd >> 0) & 0xFF;
	tx.tx_frame[4] = (cmd >> 8) & 0xFF;

	return HAL_OK;
}

//change stm32 baud rate
void UART_change_baud_rate(uint32_t baud){
	HAL_UART_Abort(imu.huart);
	HAL_UART_DeInit(imu.huart);
	imu.huart->Init.BaudRate = baud;
	HAL_UART_Init(imu.huart);

	HAL_UART_Receive_DMA(imu.huart, &imu.rx_byte, 1);
}

/* testing functions on debug mode*/
typedef struct{
	uint8_t send_save;
	uint16_t save_cmd;

	uint8_t send_calibrate;
	uint16_t calibrate_cmd;

	uint8_t send_out_content;
	uint16_t out_content_cmd;

	uint8_t send_out_rate;
	uint16_t out_rate_cmd;

	uint8_t send_baud_rate;
	uint32_t baud_rate_cmd;

	uint8_t change_uart_baud;
	uint32_t uart_buad;
} test_t;
test_t test;

void WIT_TestFunction(){
	if(test.send_save){
		test.send_save = 0;
		WIT_save((WIT_SAVE_Cmd) test.save_cmd);
	}

	if(test.send_calibrate){
		test.send_calibrate = 0;
		WIT_calibrate((WIT_CALSW_Cmd) test.calibrate_cmd);
	}

	if(test.send_out_content){
		test.send_out_content = 0;
		WIT_set_output_content((WIT_RSW_Cmd) test.out_content_cmd);
	}

	if(test.send_out_rate){
		test.send_out_rate = 0;
		WIT_set_output_rate((WIT_RRATE_Cmd) test.out_rate_cmd);
	}

	if(test.send_baud_rate){
		test.send_baud_rate = 0;
		WIT_set_baud_rate((WIT_BAUD_Cmd) test.baud_rate_cmd);
	}

	if(test.change_uart_baud){
		test.change_uart_baud = 0;
		UART_change_baud_rate(test.uart_buad);
	}
}
