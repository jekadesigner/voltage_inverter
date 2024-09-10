/*******************************
 *      modbusSlave.c
 *
 *  Created on: 1 feb. 2023
 *   Author: JorgeMaker
*******************************/

#include "main.h"
#include "string.h"
#include "modbusDevice.h"
#include "modbusSlave.h"

extern uint8_t rxFrame[255];
extern uint8_t txFrame[255];

extern uint8_t SLAVE_ID;

modbusResult handleReadHoldingRegs(UART_HandleTypeDef* huart,uint16_t* holdingRegisterValues){

	uint16_t startingReg = (rxFrame[2]<<8) | rxFrame[3];
	uint16_t numOfregs = (rxFrame[4]<<8) | rxFrame[5];

	startingReg=startingReg-30001;

	uint16_t endAddr = startingReg+numOfregs-1;



	if((numOfregs<0)|(numOfregs >125)){
		sendModbusException(huart,ILLEGAL_DATA_VALUE);
		return MODBUS_ERROR;
	}
	if(endAddr > NUM_OF_HOLDING_REGS){
		sendModbusException(huart,ILLEGAL_DATA_ADDRESS);
		return MODBUS_ERROR;
	}
	//  | SLAVE_ID | FUNC_CODE | BYTE_COUNT |    DATA     |    CRC    |
	//  |  1 byte  |   1 byte  |    1 byte  |   N*2 Bytes |   2 bytes |

	txFrame[0] = SLAVE_ID;
	txFrame[1] = rxFrame[1];
	txFrame[2] = numOfregs *2;

	int byteCursor=3;

	for(int i = 0; i<numOfregs;i++){
		txFrame[byteCursor++] =(holdingRegisterValues[startingReg]>>8) & 0xff; // Higher byte
		txFrame[byteCursor++] =(holdingRegisterValues[startingReg]) & 0xff;    // Lower  byte
		startingReg++;
	}



	sendModBusRequest(huart,txFrame,byteCursor);
	return 1;

}

modbusResult handleReadInputRegs(UART_HandleTypeDef* huart,uint16_t* inputRegisterValues){

	uint16_t startingReg = (rxFrame[2]<<8) | rxFrame[3];
	uint16_t numOfregs = (rxFrame[4]<<8) | rxFrame[5];

	uint16_t endAddr = startingReg+numOfregs-1;

	if((numOfregs<0)|(numOfregs >125)){
		sendModbusException(huart,ILLEGAL_DATA_VALUE);
		return MODBUS_ERROR;
	}
	if(endAddr > NUM_OF_HOLDING_REGS){
		sendModbusException(huart,ILLEGAL_DATA_ADDRESS);
		return MODBUS_ERROR;
	}
	//  | SLAVE_ID | FUNC_CODE | BYTE_COUNT |    DATA     |    CRC    |
	//  |  1 byte  |   1 byte  |    1 byte  |   N*2 Bytes |   2 bytes |

	txFrame[0] = SLAVE_ID;
	txFrame[1] = rxFrame[1];
	txFrame[2] = numOfregs *2;

	int byteCursor=3;

	for(int i = 0; i<numOfregs;i++){
		txFrame[byteCursor++] =(inputRegisterValues[startingReg]>>8) & 0xff; // Higher byte
		txFrame[byteCursor++] =(inputRegisterValues[startingReg]) & 0xff;    // Lower  byte
		startingReg++;
	}
	sendModBusRequest(huart,txFrame,byteCursor);
	return 1;
}

modbusResult handleReadCoils (UART_HandleTypeDef* huart,uint8_t* coilValues){

	uint16_t startAddr = ((rxFrame[2]<<8)|rxFrame[3]);  // start Coil Address

	uint16_t numCoils = ((rxFrame[4]<<8)|rxFrame[5]);    // number to coils master has requested
	if ((numCoils<1)||(numCoils>2000))  				 // maximum no. of coils as per the PDF
	{
		sendModbusException (huart,ILLEGAL_DATA_VALUE);  // send an exception
		return MODBUS_ERROR;
	}

	uint16_t endingAddr = startAddr+numCoils-1;  // Last coils address
	if (endingAddr>199)  // end coil can not be more than 199 as we only have record of 200 (0-199) coils in total
	{
		sendModbusException(huart,ILLEGAL_DATA_ADDRESS);   // send an exception
		return MODBUS_ERROR;
	}
	memset (txFrame, '\0', 256);

	txFrame[0] = SLAVE_ID;  							// Slave ID
	txFrame[1] = rxFrame[1];  							// Function code
	txFrame[2] = (numCoils/8) + ((numCoils%8)>0 ? 1:0);	// Byte count

	encodeCoils(coilValues,numCoils,txFrame+3);
	sendModBusRequest(huart,txFrame, txFrame[2]+3);
	return 1;

}

modbusResult handleReadDiscreteInputs (UART_HandleTypeDef* huart,uint8_t* dicreteInputValues){

	uint16_t startAddr = ((rxFrame[2]<<8)|rxFrame[3]);  // start Coil Address

	uint16_t numOfDiscInputs = ((rxFrame[4]<<8)|rxFrame[5]);   	// number to coils master has requested
	if ((numOfDiscInputs<1)||(numOfDiscInputs>2000))  			// maximum no. of coils as per the PDF
	{
		sendModbusException (huart,ILLEGAL_DATA_VALUE);  		// send an exception
		return MODBUS_ERROR;
	}

	uint16_t endingAddr = startAddr+numOfDiscInputs-1; 			 // Last coils address
	if (endingAddr>199)  										 // end coil can not be more than 199
	{
		sendModbusException(huart,ILLEGAL_DATA_ADDRESS);   // send an exception
		return MODBUS_ERROR;
	}
	memset (txFrame, '\0', 256);

	txFrame[0] = SLAVE_ID;  							// slave ID
	txFrame[1] = rxFrame[1];  							// function code
	txFrame[2] = (numOfDiscInputs/8) + ((numOfDiscInputs%8)>0 ? 1:0);	// Byte count

     txFrame[3]=1;
//	decodeCoilsFromRXBuffer(dicreteInputValues,numOfDiscInputs,txFrame+3);
	//encodeCoils(dicreteInputValues,numOfDiscInputs,txFrame+3);
	sendModBusRequest(huart,txFrame, txFrame[2]+3);
	return 1;

}

modbusResult handleWriteSingleHandlingRegister(UART_HandleTypeDef* huart,uint16_t* holdingRegisterValues){

	uint16_t startingAddr = ((rxFrame[2]<<8)|rxFrame[3]);  // start Register Address

	if (startingAddr> NUM_OF_HOLDING_REGS)  //  The Register Address can not be more than 49 as
		               	   	   	   	   	   	//  we only have record of XX Registers in total
	{
		sendModbusException(huart,ILLEGAL_DATA_ADDRESS);   // send an exception
		return MODBUS_ERROR;
	}
	holdingRegisterValues[startingAddr] = (rxFrame[4]<<8)|rxFrame[5];

	txFrame[0] = SLAVE_ID;     // Slave ID
	txFrame[1] = rxFrame[1];   // Function code

	txFrame[2] = rxFrame[2];   // Start Addr HIGH Byte
	txFrame[3] = rxFrame[3];   // Start Addr LOW Byte

	txFrame[4] = rxFrame[4];   // Reg Data HIGH Byte
	txFrame[5] = rxFrame[5];   // Reg Data LOW  Byte

	sendModBusRequest(huart,txFrame, 6);
	return 1;

}

modbusResult handleWriteMulyipleHandlingRegister (UART_HandleTypeDef* huart,uint16_t* holdingRegisterValues){

	uint16_t staringtAddr = ((rxFrame[2]<<8)| rxFrame[3]);

	uint16_t numRegs = ((rxFrame[4]<<8) | rxFrame[5]);   // number to registers master has requested

	if ((numRegs<1)||(numRegs>123))  // maximum no. of Registers as per Modbus Specification
	{
		sendModbusException(huart,ILLEGAL_DATA_VALUE);  // send an exception
		return MODBUS_ERROR;
	}

	uint16_t endAddr = staringtAddr + numRegs - 1;  // end Register
	if (endAddr> NUM_OF_HOLDING_REGS)  	// end Register can not be more than NUM_OF_HOLDING_REGS as
										// we only have record of NUM_OF_HOLDING_REGS Registers in total
	{
		sendModbusException(huart,ILLEGAL_DATA_ADDRESS);   // send an exception
		return MODBUS_ERROR;
	}

	int indx = 7;  // we need to keep track of index in rxFrame

	for (int regCorsor=0; regCorsor<numRegs; regCorsor++){

		holdingRegisterValues[staringtAddr++] = (rxFrame[indx++]<<8)|rxFrame[indx++];


	}
	//   | SLAVE_ID | FUNCTION_CODE | Start Addr |  num of Regs |   CRC   |
	//   | 1 BYTE   |     1 BYTE    |  2 BYTE    |    2 BYTES   | 2 BYTES |

	txFrame[0] = SLAVE_ID;     // Slave ID
	txFrame[1] = rxFrame[1];   // Function code

	txFrame[2] = rxFrame[2];   // Start Addr HIGH Byte
	txFrame[3] = rxFrame[3];   // Start Addr LOW Byte

	txFrame[4] = rxFrame[4];   // Num of Regs HIGH Byte
	txFrame[5] = rxFrame[5];   // Num of Regs LOW Byte

	sendModBusRequest(huart,txFrame, 6);  // send data... CRC will be calculated in the function itself
	return 1;   // success

}

modbusResult handleWriteSingleCoil (UART_HandleTypeDef* huart,uint8_t* coilValues){

	uint16_t staringtAddr = ((rxFrame[2]<<8) | rxFrame[3]);  // start Coil Address

	if (staringtAddr>199)  // The Coil Address can not be more than 199 as we only have record of 200 Coils in total
	{
		sendModbusException(huart,ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}
	/* The next 2 bytes in the RxData determines the state of the coil
	 * A value of FF 00 hex requests the coil to be ON.
	 * A value of 00 00 requests it to be OFF.
	 * All other values are illegal and will not affect the coil.
	 */
	if ((rxFrame[4] == 0xFF) && (rxFrame[5] == 0x00)){
		coilValues[staringtAddr] |= ON;
	}

	else if ((rxFrame[4] == 0x00) && (rxFrame[5] == 0x00)){
		coilValues[staringtAddr] &=  OFF;
	}
	txFrame[0] = SLAVE_ID;     // Slave ID
	txFrame[1] = rxFrame[1];   // Function code

	txFrame[2] = rxFrame[2];   // Start Addr HIGH Byte
	txFrame[3] = rxFrame[3];   // Start Addr LOW Byte

	txFrame[4] = rxFrame[4];   // Coil Data HIGH Byte
	txFrame[5] = rxFrame[5];   // Coil Data LOW  Byte

	sendModBusRequest(huart,txFrame, 6);  // send data... CRC will be calculated in the function itself

	return 1;

}

void extractFromByte(uint8_t byte, uint8_t bitToExtract,uint8_t* outPutResuls ){

	for(int bitCursor=0; (bitCursor <8) & (bitCursor<=bitToExtract); bitCursor++){
		uint8_t decodedValue =  (byte >> bitCursor) & 1;
		outPutResuls[bitCursor] = decodedValue;
	}
}

void decodeCoilsFromRXBuffer(uint8_t numerOfBytes,uint8_t numberOfCoils,uint8_t* bytesSource, uint8_t* outPutResuls){

	uint8_t remainingBits = numberOfCoils;
	uint8_t bitsToExtract =0;
	for(uint8_t byteConter =0;byteConter<numerOfBytes;byteConter++){
		if (remainingBits-8 >=0){
			bitsToExtract =8;
		}
		else{
			bitsToExtract = remainingBits;
		}
		uint8_t currentByte = bytesSource[byteConter];
		extractFromByte(currentByte, bitsToExtract,outPutResuls+byteConter*8);
		remainingBits= remainingBits-8;
	}
}

modbusResult hadleWriteMultipleCoils(UART_HandleTypeDef* huart,uint8_t* coilValues){

	uint16_t staringtAddr = ((rxFrame[2]<<8) | rxFrame[3]);  // start coil Address
	uint16_t numCoils = 	((rxFrame[4]<<8) | rxFrame[5]);   // Number of coils

	uint8_t byteCount = rxFrame[6];

	decodeCoilsFromRXBuffer(byteCount, numCoils,rxFrame+7, coilValues+(uint8_t)staringtAddr );

	txFrame[0] = SLAVE_ID;     // Slave ID
	txFrame[1] = rxFrame[1];   // Function code
	txFrame[2] = rxFrame[2];   // Start Addr HIGH Byte
	txFrame[3] = rxFrame[3];   // Start Addr LOW Byte
	txFrame[4] = rxFrame[4];   // Num of coils HIGH Byte
	txFrame[5] = rxFrame[5];   // Num of coils LOW  Byte

	sendModBusRequest(huart,txFrame, 6);
	return MODBUS_OK;
}

modbusResult sendModbusException(UART_HandleTypeDef* huart,modbusExceptionCode exceptionCode){

	// | SLAVE_ID | FUNCTION_CODE | Exception code | CRC     |
	// | 1 BYTE   |  1 BYTE       |    1 BYTE      | 2 BYTES |

	txFrame[0] = rxFrame[0];       		// Slave ID
	txFrame[1] = rxFrame[1] | 0x80;  	// Adding 1 to the MSB of the function code
	txFrame[2] =  exceptionCode;   		// Load the Exception code

	return 	sendModBusRequest(huart,txFrame, 3);  // send Data... CRC will be calculated in the function
}
