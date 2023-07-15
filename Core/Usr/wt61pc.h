/*
 * wt61pc.h
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
	uint16_t T_ms;
} WT61PC_t;

typedef enum{
	WIT_REG_SAVE = 0x00,	//save or restart or reset
	WIT_REG_CALSW = 0x01,	//calibration mode
	WIT_REG_RSW = 0x02,		//output content
	WIT_REG_RRATE = 0x03,	//output rate
	WIT_REG_BAUD = 0x04 	//baud rate
} WIT_RegisterName;

typedef enum{
	WIT_SAVE_SAVE = 0x0000,
	WIT_SAVE_RESTART = 0x00FF,
	WIT_SAVE_FACTORY_RESET = 0x0001
} WIT_SAVE_Cmd;

typedef enum{	//calibrate
	WIT_CALSW_NORMAL = 0x0000,
	WIT_CALSW_AUTO_ACC_CALIBRATION = 0x0001,
	WIT_CALSW_HEIGHT_RESET = 0x0003,
	WIT_CALSW_HEADING_ZERO = 0x0004,
	WIT_CALSW_MAG_FIELD_CALIBRATION_SPHERICAL = 0x0007,
	WIT_CALSW_SET_ANGLE_REF = 0x0008,
	WIT_CALSW_MAG_FIELD_CALIBRATION_DUAL_PLANE = 0x0009
} WIT_CALSW_Cmd;

typedef enum{	//output content (multiple content = or every bit)
	WIT_RSW_TIME_BIT = 0,	//on addr 0x50
	WIT_RSW_ACC_BIT = 1,	//on addr 0x51
	WIT_RSW_GYRO_BIT = 2,	//on addr 0x52
	WIT_RSW_ANGLE_BIT = 3,	//on addr 0x53
	WIT_RSW_MAG_BIT = 4,	//on addr 0x54
	WIT_RSW_PORT_BIT = 5,	//on addr 0x55
	WIT_RSW_PRESS_BIT = 6,	//on addr 0x56
	WIT_RSW_GPS_BIT = 7,	//on addr 0x57
	WIT_RSW_VELO_BIT = 8,	//on addr 0x58
	WIT_RSW_QUARTER_BIT = 9,//on addr 0x59
	WIT_RSW_GSA_BIT = 10	//on addr 0x5A
} WIT_RSW_Cmd;

typedef enum{
	WIT_RRATE_02HZ = 0x0001,	//output rate 0.2 Hz
	WIT_RRATE_05HZ = 0x0002,	//output rate 0.5 Hz
	WIT_RRATE_1HZ = 0x0003,	//output rate 1 Hz
	WIT_RRATE_2HZ = 0x0004,	//output rate 2 Hz
	WIT_RRATE_5HZ = 0x0005,	//output rate 5 Hz
	WIT_RRATE_10HZ = 0x0006,	//output rate 10 Hz
	WIT_RRATE_20HZ = 0x0007,	//output rate 20 Hz
	WIT_RRATE_50HZ = 0x0008,	//output rate 50 Hz
	WIT_RRATE_100HZ = 0x0009,	//output rate 100 Hz
	WIT_RRATE_200HZ = 0x000B,	//output rate 200 Hz
	WIT_RRATE_SINGLE_RETURN = 0x000C,	//output rate ?
	WIT_RRATE_NO_RETURN = 0x000D,	//output rate ?
} WIT_RRATE_Cmd;

typedef enum{
	WIT_BAUD_4800 = 0x0001,
	WIT_BAUD_9600 = 0x0002,
	WIT_BAUD_19200 = 0x0003,
	WIT_BAUD_38400 = 0x0004,
	WIT_BAUD_57600 = 0x0005,
	WIT_BAUD_115200 = 0x0006,
	WIT_BAUD_230400 = 0x0007,
	WIT_BAUD_460800 = 0x0008,	//not all imu supported
	WIT_BAUD_921600 = 0x0009,	//not all imu supported
} WIT_BAUD_Cmd;

typedef struct{
	uint8_t tx_frame[5];
	uint8_t ctr;
	uint32_t timestamp;

	uint8_t register_data[11];
} WIT_TX_Command;

extern WT61PC_t imu;

void WT61PC_init(UART_HandleTypeDef* huart);
void WT61PC_rx_callback(UART_HandleTypeDef* huart);
void WT61PC_rx_restart();
void WT61PC_tx_routine();

HAL_StatusTypeDef WIT_save(WIT_SAVE_Cmd cmd);
HAL_StatusTypeDef WIT_calibrate(WIT_CALSW_Cmd cmd);
HAL_StatusTypeDef WIT_set_output_content(uint8_t out_content);
HAL_StatusTypeDef WIT_set_output_rate(WIT_RRATE_Cmd cmd);
HAL_StatusTypeDef WIT_set_baud_rate(WIT_BAUD_Cmd cmd);

void UART_change_baud_rate(uint32_t baud);
void WIT_TestFunction();

#endif /* USR_WT61PC_H_ */
