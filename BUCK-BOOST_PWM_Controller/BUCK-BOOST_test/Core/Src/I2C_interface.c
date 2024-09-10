/*
 * I2C_interface.c
 *
 *  Created on: Jun 13, 2024
 *      Author: chipi
 */


#include "I2C_interface.h"
extern I2C_HandleTypeDef hi2c1;

static uint8_t ind_data = 0xFF;  // По умолчанию все порты высокие (pull-up)


void IND_WritePin(uint8_t ind, uint8_t state) {
    if (ind > 7) return;  // Пины PCF8574 от 0 до 7

    if (state) {

        ind_data &= ~(1 << ind);  // Сбросить бит
    } else {

        ind_data |= (1 << ind);  // Установить бит
    }

    HAL_I2C_Master_Transmit(&hi2c1, IND_ADDRESS << 1, &ind_data, 1, HAL_MAX_DELAY);
}



void ReadButtons(uint8_t *buttons) {
    HAL_I2C_Master_Receive(&hi2c1, BUTTONS_ADDRESS<< 1, buttons, 1, HAL_MAX_DELAY);

}


void Get_Buttons_States(uint8_t *buttons_states) {
    uint8_t buttons;
    ReadButtons(&buttons);
    for (uint8_t pin = 0; pin < 8; pin++) {
    	buttons_states[pin] = !((buttons >> pin) & 0x01);

    }
}
