/*
 * I2C_interface.h
 *
 *  Created on: Jun 13, 2024
 *      Author: chipi
 */

#ifndef INC_I2C_INTERFACE_H_
#define INC_I2C_INTERFACE_H_


#include "main.h"

#define IND_ADDRESS 0x25
#define BUTTONS_ADDRESS  0x24  // Адрес второй микросхемы для чтения

#define LCD_OFF IND_WritePin(0,1);
#define LCD_ON IND_WritePin(0,0);
#define IND_1_ON IND_WritePin(1,1);
#define IND_1_OFF IND_WritePin(1,0);
#define IND_2_ON IND_WritePin(2,1);
#define IND_2_OFF IND_WritePin(2,0);
#define IND_3_ON IND_WritePin(3,1);
#define IND_3_OFF IND_WritePin(3,0);
#define IND_4_ON IND_WritePin(4,1);
#define IND_4_OFF IND_WritePin(4,0);
#define IND_5_ON IND_WritePin(5,1);
#define IND_5_OFF IND_WritePin(5,0);
#define IND_6_ON IND_WritePin(6,1);
#define IND_6_OFF IND_WritePin(6,0);
#define IND_7_ON IND_WritePin(7,1);
#define IND_7_OFF IND_WritePin(7,0);


extern I2C_HandleTypeDef hi2c1;


void IND_WritePin(uint8_t ind, uint8_t state);
void ReadButtons(uint8_t *buttons);
void Get_Buttons_States(uint8_t *buttons_states);

#endif /* INC_I2C_INTERFACE_H_ */
