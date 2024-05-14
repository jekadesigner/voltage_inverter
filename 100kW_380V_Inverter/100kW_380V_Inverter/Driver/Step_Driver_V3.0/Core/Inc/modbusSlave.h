 /*******************************
  *      modbusSlave.h
  *
  *  Created on: 1 feb. 2023
  *   Author: JorgeMaker
 *******************************/

#ifndef INC_MODBUSSLAVE_H_
#define INC_MODBUSSLAVE_H_

#include "modbusDevice.h"

#define NUM_OF_HOLDING_REGS 30
#define LED_3_ON    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1,GPIO_PIN_SET);
#define LED_3_OFF   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1,GPIO_PIN_RESET);


modbusResult sendModbusException(UART_HandleTypeDef* huart,modbusExceptionCode exceptionCode);

modbusResult handleReadHoldingRegs (UART_HandleTypeDef* huart,uint16_t* holdingRegisterValues);
modbusResult handleReadInputRegs (UART_HandleTypeDef* huart,uint16_t* inputRegisterValues);
modbusResult handleReadCoils (UART_HandleTypeDef* huart,uint8_t* coilValues);
modbusResult handleReadDiscreteInputs (UART_HandleTypeDef* huart,uint8_t* dicreteInputValues);
modbusResult handleWriteSingleHandlingRegister (UART_HandleTypeDef* huart,uint16_t* holdingRegisterValues);
modbusResult handleWriteMulyipleHandlingRegister (UART_HandleTypeDef* huart,uint16_t* holdingRegisterValues);
modbusResult handleWriteSingleCoil (UART_HandleTypeDef* huart,uint8_t* coilValues);
modbusResult hadleWriteMultipleCoils(UART_HandleTypeDef* huart,uint8_t* coilValues);

void extractFromByte(uint8_t byte, uint8_t bitToExtract,uint8_t* outPutResuls );
void decodeCoilsFromRXBuffer(uint8_t numerOfBytes,uint8_t numberOfCoils,uint8_t* bytesSource, uint8_t* outPutResuls);

#endif /* INC_MODBUSSLAVE_H_ */
