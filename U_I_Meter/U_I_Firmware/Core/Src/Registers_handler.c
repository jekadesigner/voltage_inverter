/*
 * Registers_handler.c
 *
 *  Created on: Jun 21, 2024
 *      Author: chipi
 */

#include "Registers_handler.h"
//#include "I2C_display.h"
//#include "I2C_interface.h"
#define RX_BUFFER_SIZE 64
//extern RX_BUFFER_SIZE;
extern uint8_t SLAVE_ID;

void RestartModbusReception(uint8_t* rxFrame) {
    // Очищаем буфер
    for (uint16_t i = 0; i < 64; i++) {
        rxFrame[i] = 0;
    }

    RX_2; // Сброс RX состояния (определите макрос, если он отсутствует)
    Reset_USART1(); // Сброс UART (вызов вашей функции)

    // Перезапуск приёма через DMA
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxFrame, RX_BUFFER_SIZE) != HAL_OK) {
        // Если ошибка при запуске приёма, можно добавить дополнительную обработку
       // LED_1_ON; // Например, индикация ошибки
    }

    // Включение прерывания IDLE
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

void Registers_handler(uint8_t* rxFrame, uint16_t* data_reg, uint16_t* rcv_data_reg,uint16_t Size){


	if(rxFrame[0] != SLAVE_ID){   for(uint16_t i=0; i<64;i++){rxFrame[i]=0;}
	     RX_2;
	    // LED_1_OFF;
	     	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxFrame, RX_BUFFER_SIZE);
			__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
		return;}


	if (Size < 4) {
	        // Ошибка: неверный ID устройства или слишком короткий пакет
	        sendError(&huart1, 0x03, 0x02); // Код ошибки 0x02: ошибка длины пакета

	        return;
	    }


	  // Расчет CRC для пакета (исключая последние 2 байта CRC)
	    uint16_t receivedCRC = (rxFrame[Size - 1]) | (rxFrame[Size - 2]<<8);
	    uint16_t calculatedCRC = calcCRC16ModBus(rxFrame, Size - 2);

	    // Проверка CRC
	    if (receivedCRC != calculatedCRC) {
	        // Ошибка: неверный CRC
	        sendError(&huart1, 0x03, 0x03); // Код ошибки 0x03: нарушение данных

	        return;
	    }


                       uint8_t opCode = rxFrame[1];

				  			  switch (opCode) {
				  			   case READ_COILs:
				  				 handleReadCoils (&huart1,coils);
				  			   break;

				  			   case   READ_DISC_INPUTs:
				  				 handleReadDiscreteInputs (&huart1, dicreteInputs);
							   break;

				  			   case  READ_HOLDING_REGs:

				  			 	handleReadHoldingRegs(&huart1,data_reg);
				  			   break;

				  			   case  READ_INPUT_REGs:
				  				handleReadInputRegs (&huart1,data_reg);

				  			   break;

				  			   case WRITE_SINGLE_REG:

				  			    handleWriteMulyipleHandlingRegister(&huart1,rcv_data_reg);

                               break;

				  			   case WRITE_HOLDING_REGs:

                                 handleWriteMulyipleHandlingRegister(&huart1,rcv_data_reg);

                               break;


				  			   default:


				  				 sendError(&huart1, opCode, 0x01); // Код ошибки 0x01: недопустимый код функции
				  			   break;
				  			     }



                          }

void Modbus_parsing(uint8_t* rxFrame){


    //  lcd_set_cursor(1, 0);
  	 // sprintf(int_to_str, "%02X,%02X,%01X,%01X,%01X,%01X,%04X", rxFrame[0],rxFrame[1],rxFrame[2],rxFrame[3],rxFrame[4] ,rxFrame[5],((rxFrame[6]<<8)|rxFrame[7]));
  	 // lcd_write_string(int_to_str);


}
