/*
 * Registers_handler.h
 *
 *  Created on: Jun 21, 2024
 *      Author: chipi
 */

#ifndef INC_REGISTERS_HANDLER_H_
#define INC_REGISTERS_HANDLER_H_

#include "main.h"
#include "modbusDevice.h"
#include "modbusSlave.h"
#include "stm32f3xx_hal.h"
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

void Registers_handler( uint8_t* rxFrame,uint16_t* data_reg, uint16_t* rcv_data_reg);
void Modbus_parsing(uint8_t* rxFrame);

#endif /* INC_REGISTERS_HANDLER_H_ */
