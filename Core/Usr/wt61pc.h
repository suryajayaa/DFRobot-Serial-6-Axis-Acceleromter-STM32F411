/*
 * wt61pc.h
 *
 *  Created on: Jul 12, 2023
 *      Author: Surya
 */

#ifndef USR_WT61PC_H_
#define USR_WT61PC_H_

#include "main.h"
#include "usart.h"

typedef struct{
	float roll_x;
	float pitch_y;
	float yaw_z;
} ang_pos_t;

typedef struct{
	float x;
	float y;
	float z;
} WT61PC_axis_t;

typedef struct{
	UART_HandleTypeDef* huart;
	uint8_t rx_frame[11];
	uint8_t rx_ptr;
	uint8_t rx_byte;

	WT61PC_axis_t acceleration;
	WT61PC_axis_t angle_pos;
	WT61PC_axis_t angle_vel;
	WT61PC_axis_t magnetic_field;

	float temperature;
	float voltage;
	uint16_t version;

	uint32_t timestamp;
} WT61PC_t;

extern WT61PC_t imu;

void WT61PC_init(UART_HandleTypeDef* huart);
void WT61PC_rx_callback(UART_HandleTypeDef* huart);
void WT61PC_rx_restart();

#endif /* USR_WT61PC_H_ */
