/*
 * Registers_handler.c
 *
 *  Created on: Jun 21, 2024
 *      Author: chipi
 */

#include "Registers_handler.h"
#include "I2C_display.h"
#include "I2C_interface.h"

extern RX_BUFFER_SIZE;
extern uint8_t SLAVE_ID;

void Registers_handler(uint8_t* rxFrame, uint16_t* data_reg, uint16_t* rcv_data_reg){

			 if (rxFrame[0] == SLAVE_ID) {
                       uint8_t opCode = rxFrame[1];

				  			  switch (opCode) {

				  			   case   READ_DISC_INPUTs:

							   break;


				  			   case  READ_HOLDING_REGs:

				  			 	handleReadHoldingRegs(&huart2,data_reg);
				  			   break;

				  			   case READ_COILs:

				  			   break;

				  			   case  READ_INPUT_REGs:

				  			   break;


				  			   case WRITE_SINGLE_REG:

				  			    handleWriteMulyipleHandlingRegister (&huart2,rcv_data_reg);

                               break;

				  			   case WRITE_HOLDING_REGs:

                                 handleWriteMulyipleHandlingRegister (&huart2,rcv_data_reg);

                               break;


				  			   default:


				  			   break;
				  			     }

				  			 }
			                  else{
			                            for(uint16_t i=0; i<64;i++){rxFrame[i]=0;}
			 				  			    	     RX_2;
			 				  			  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rxFrame,64 );
			 				  			    			    // Включение прерывания IDLE
			 				  			   __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);}

                          }

void Modbus_parsing(uint8_t* rxFrame){


      lcd_set_cursor(1, 0);
  	  sprintf(int_to_str, "%02X,%02X,%01X,%01X,%01X,%01X,%04X", rxFrame[0],rxFrame[1],rxFrame[2],rxFrame[3],rxFrame[4] ,rxFrame[5],((rxFrame[6]<<8)|rxFrame[7]));
  	  lcd_write_string(int_to_str);



}
