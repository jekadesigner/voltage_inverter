

#ifndef INC_REGISTERS_HANDLER_H_
#define INC_REGISTERS_HANDLER_H_

#include "main.h"
#include "modbusDevice.h"
#include "modbusSlave.h"
#include "stm32g0xx_hal.h"
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
//extern TIM_HandleTypeDef htim15;
extern uint8_t dicreteInputs;
extern uint8_t coils;
void Registers_handler( uint8_t* rxFrame,uint16_t* data_reg, uint16_t* rcv_data_reg,uint16_t Size);
void Modbus_parsing(uint8_t* rxFrame);
void RestartModbusReception(uint8_t* rxFrame);
#endif /* INC_REGISTERS_HANDLER_H_ */
