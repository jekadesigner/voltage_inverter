/*******************************
 *      modbusDevice.h
 *
 *  Created on: 1 feb. 2023
 *   Author: JorgeMaker
*******************************/

#ifndef INC_MODBUSMASTER_H__OLD_
#define INC_MODBUSMASTER_H__OLD_

#include "main.h"
#include <string.h>

#define PROCCESING_PAUSE 45
#define DELAY_PRECCESING_FLAG

typedef enum {
	READ_COILs = 		0x01,
	READ_DISC_INPUTs =	0x02,
	READ_HOLDING_REGs =	0x03,
	READ_INPUT_REGs = 	0x04,
	WRITE_SINGLE_COIL = 0x05,
	WRITE_SINGLE_REG = 	0x06,
	WRITE_MULT_COILs = 	0x0F, // 15
	WRITE_HOLDING_REGs =0x10  // 16

} modbusOpCode;


typedef enum {
	ILEGAL_FUCTION 	=   	0x01,
	ILLEGAL_DATA_ADDRESS=   0x02,
	ILLEGAL_DATA_VALUE = 	0x03
} modbusExceptionCode;

typedef enum {
	MODBUS_OK 	=  	0x01,
	MODBUS_ERROR =  0x00
} modbusResult;


typedef enum{
	ON  = 0x01,
	OFF = 0x00
}binaryDataValue;

typedef enum{
	True  = 0x01,
	False = 0x00
}BooleanValue;

uint16_t calcCRC16ModBus (uint8_t *buffer, uint8_t u8length);
void extractBinaryReceivedData(uint8_t len, uint8_t *receivedData,uint8_t *extractedBits);
uint8_t encodeBitsIntoByte (uint8_t *bitToEncode, uint8_t len);
uint8_t encodeCoils(uint8_t* coilValues, uint8_t coilsNum, uint8_t* output_array);
void copyFrom_S_to_E_UpTo8Values(uint8_t* origin, uint8_t* destination, uint8_t start,  uint8_t end);
modbusResult sendModBusRequest(UART_HandleTypeDef* huart,uint8_t* frame,uint8_t len);

#endif /* INC_MODBUSMASTER_H__OLD_ */
