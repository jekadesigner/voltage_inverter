/*
 * ADC_1.c
 *
 *  Created on: Jun 20, 2024
 *      Author: chipi
 */


#include "ADC_1.h"

//ADC_HandleTypeDef hadc1;
/*
void MX_ADC1_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    // Инициализация глобальных параметров ADC
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;  // Одиночный канал
    hadc1.Init.ContinuousConvMode = DISABLE;  // Одиночное преобразование
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }
}
*/

void Configure_ADC_Channel(uint32_t channel) {

    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;  // Выберите время выборки в зависимости от ваших требований

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

uint32_t Read_ADC_Value(uint32_t channel) {
    Configure_ADC_Channel(channel);

    // Запуск преобразования
    HAL_ADC_Start(&hadc1);

    // Ожидание завершения преобразования
   // if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
  //      Error_Handler();
  //  }

    HAL_ADC_PollForConversion(&hadc1, 10);
    // Чтение значения
    uint32_t value = HAL_ADC_GetValue(&hadc1);

    // Остановка ADC
    HAL_ADC_Stop(&hadc1);

    return value;
}


void Read_ADC_Values(uint32_t* values, uint32_t size) {
    if (size < 6) {
        // Недостаточно места в массиве для хранения всех значений
        Error_Handler();
        return;
    }

    // Заполнение массива значениями с каналов 1, 2, 3, 4, температурного сенсора и Vrefint
    values[0] = Read_ADC_Value(ADC_CHANNEL_1);
    values[1] = Read_ADC_Value(ADC_CHANNEL_2);
    values[2] = Read_ADC_Value(ADC_CHANNEL_3);
    values[3] = Read_ADC_Value(ADC_CHANNEL_4);
    values[4] = Read_ADC_Value(ADC_CHANNEL_TEMPSENSOR);
    values[5] = Read_ADC_Value(ADC_CHANNEL_VREFINT);



}



