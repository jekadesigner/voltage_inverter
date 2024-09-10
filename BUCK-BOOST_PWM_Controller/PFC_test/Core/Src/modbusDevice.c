/*******************************
 *      modbusDevice.c
 *
 *  Created on: 1 feb. 2023
 *   Author: JorgeMaker
*******************************/

#include "modbusDevice.h"


// Compute the MODBUS RTU CRC
uint16_t calcCRC16ModBus(uint8_t *buffer, uint8_t u8length) {
	unsigned int temp, temp2, flag;
	temp = 0xFFFF;
	for (unsigned char i = 0; i < u8length; i++) {
		temp = temp ^ buffer[i];
		for (unsigned char j = 1; j <= 8; j++) {
			flag = temp & 0x0001;
			temp >>= 1;
			if (flag)
				temp ^= 0xA001;
		}
	}
	// Reverse byte order.
	temp2 = temp >> 8;
	temp = (temp << 8) | temp2;
	temp &= 0xFFFF;
	// the returned value is already swapped
	// crcLo byte is first & crcHi byte is last
	return temp;

}

/**
 * @brief extractBinaryReceivedData is a function that takes an array of received data and extracts the individual bits from each byte,
 * storing them in another array.
 *
 * @param len: the length of the receivedData array
 * @param receivedData: the array containing the data to be extracted
 * @param extractedBits: the array where the extracted bits will be stored
 */

void extractBinaryReceivedData(uint8_t len, uint8_t *receivedData, uint8_t *extractedBits) {
	uint8_t rxDataCursor = 0;
	for (uint8_t byteCursor = 0; byteCursor < len; byteCursor++) {
		// Obtain each received byte containing
		uint8_t currentByte = receivedData[byteCursor];
		for (int i = 0; i < 8; i++) {
			extractedBits[rxDataCursor] = (currentByte & (1 << i)) >> i;
			rxDataCursor++;
		}
	}
}

modbusResult sendModBusRequest(UART_HandleTypeDef* huart, uint8_t* frame,uint8_t len) {

    uint16_t crc = calcCRC16ModBus(frame, len);
    frame[len+1] = crc & 0xFF;       // CRC LOW
    frame[len] = (crc >> 8) & 0xFF;  // CRC HIGH

    TX_2;
  //  HAL_UART_Transmit_DMA(huart,  frame, len+2);

    if (HAL_UART_Transmit_DMA(huart,  frame, len+2) != HAL_OK) {

	        RX_2;

	        Error_Handler();
	    }

	 else{
		  LED_1_OFF;
	  }

}
