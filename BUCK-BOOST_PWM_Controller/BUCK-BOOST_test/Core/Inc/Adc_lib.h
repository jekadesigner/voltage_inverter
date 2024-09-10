#ifndef ADC_LIB_H
#define ADC_LIB_H

#include "stm32f3xx.h"

// Функция инициализации ADC1
void ADC1_Init(void);

// Функция инициализации ADC2
void ADC2_Init(void);

// Функция для запуска преобразования и чтения данных ADC1
void ADC1_StartConversion(uint16_t *data_buffer);

// Функция для запуска преобразования и чтения данных ADC2
void ADC2_StartConversion(uint16_t *data_buffer);

#endif // ADC_LIB_H
