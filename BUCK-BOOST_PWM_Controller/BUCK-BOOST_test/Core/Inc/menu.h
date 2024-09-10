/*
 * menu.h
 *
 *  Created on: Jul 10, 2024
 *      Author: chipi
 */

#ifndef INC_MENU_H_
#define INC_MENU_H_

#include "I2C_display.h"

extern uint16_t adc1_data[4];
extern uint16_t adc2_data[3];
extern uint32_t pwm_buck;
extern uint16_t ADC_1_MAX;
extern uint16_t ADC_2_MAX;
extern uint16_t ADC_3_MAX;

extern uint16_t ADC_1_MID;
extern uint16_t ADC_2_MID;
extern uint16_t ADC_3_MID;

extern uint16_t ADC_1_MIN;
extern uint16_t ADC_2_MIN;
extern uint16_t ADC_3_MIN;





// Определение состояний меню
typedef enum {

	MENU_MAIN_VIEW,
	MENU_SUPPLY_VIEW,
	MENU_CONNECT_VIEW,
	MENU_ALARM_VIEW,
	MENU_STATE_COUNT
} MenuState;

// Структура для хранения состояния меню
typedef struct {
    MenuState currentState;
    uint8_t currentOption;
} Menu;

void Menu_Init(Menu *menu);
void Menu_HandleButtonPress(Menu *menu, uint8_t button);
void Menu_UpdateDisplay(Menu *menu);

#endif /* INC_MENU_H_ */
