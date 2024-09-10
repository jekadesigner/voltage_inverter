/*
 * ADC_1.h
 *
 *  Created on: Jun 20, 2024
 *      Author: chipi
 */

#ifndef INC_ADC_1_H_
#define INC_ADC_1_H_


#include "stm32f3xx_hal.h"

// Экспортируемый дескриптор ADC
extern ADC_HandleTypeDef hadc1;

// Функции для инициализации и использования ADC1

void Configure_ADC_Channel(uint32_t channel);
uint32_t Read_ADC_Value(uint32_t channel);
void Read_ADC_Values(uint32_t* values, uint32_t size);

#endif /* INC_ADC_1_H_ */
