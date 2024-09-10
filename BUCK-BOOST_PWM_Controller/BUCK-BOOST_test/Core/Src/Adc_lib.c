#include "adc_lib.h"
#include "main.h"

// Определение значения SMP для 7.5 тактов (011 в двоичном формате)
#define ADC_SAMPLING_TIME_7_5 0x3
#define ADC_SAMPLING_TIME_61_5 0x5
#define ADC_SAMPLING_TIME_181_5 0x6
#define ADC_SAMPLING_TIME_601_5 0x7

void ADC1_Init(void) {


    // Включение тактирования для ADC1
    RCC->AHBENR |= RCC_AHBENR_ADC12EN;

    // Сброс флагов ADC
    ADC1->ISR |= ADC_ISR_ADRDY;

    ADC12_COMMON->CCR &= ~ADC_CCR_CKMODE; // Очистка старых значений
    ADC12_COMMON->CCR |= (0x1 << ADC_CCR_CKMODE_Pos); // Установка CKMODE на HCLK/1

    // Настройка последовательного преобразования каналов для ADC1
    ADC1->SQR1 = (1 << ADC_SQR1_SQ1_Pos) | // Первый канал 1
                 (2 << ADC_SQR1_SQ2_Pos) | // Второй канал 2
                 (3 << ADC_SQR1_SQ3_Pos) | // Третий канал 3
                 (4 << ADC_SQR1_SQ4_Pos);  // Четвертый канал 4

    ADC1->SQR1 |= (4 << ADC_SQR1_L_Pos); // Указываем, что 4 преобразования

    // Настройка времени выборки для каналов 1-4
       ADC1->SMPR1 &= ~((0x7 << ADC_SMPR1_SMP1_Pos) | // Очистка старых значений
                        (0x7 << ADC_SMPR1_SMP2_Pos) |
                        (0x7 << ADC_SMPR1_SMP3_Pos) |
                        (0x7 << ADC_SMPR1_SMP4_Pos));  // Каналы 1-4


       ADC1->SMPR1 |= (ADC_SAMPLING_TIME_601_5 << ADC_SMPR1_SMP1_Pos) | // Канал 1
                      (ADC_SAMPLING_TIME_601_5 << ADC_SMPR1_SMP2_Pos) | // Канал 2
                      (ADC_SAMPLING_TIME_601_5 << ADC_SMPR1_SMP3_Pos) | // Канал 3
                      (ADC_SAMPLING_TIME_601_5 << ADC_SMPR1_SMP4_Pos);  // Канал 4

       // Запуск калибровки ADC1
       ADC1->CR &= ~ADC_CR_ADEN; // Отключить ADC перед калибровкой
       ADC1->CR |= ADC_CR_ADCAL; // Запуск калибровки
       while (ADC1->CR & ADC_CR_ADCAL); // Ожидание завершения калибровки

       // Включение ADC1
       ADC1->CR |= ADC_CR_ADEN;
       while (!(ADC1->ISR & ADC_ISR_ADRDY)); // Ожидание готовности ADC1


}

void ADC2_Init(void) {
    // Включение тактирования для ADC2
    RCC->AHBENR |= RCC_AHBENR_ADC12EN;

    // Сброс флагов ADC

    ADC2->ISR |= ADC_ISR_ADRDY;


    ADC12_COMMON->CCR &= ~ADC_CCR_CKMODE; // Очистка старых значений
    ADC12_COMMON->CCR |= (0x1 << ADC_CCR_CKMODE_Pos); // Установка CKMODE на HCLK/1

    // Настройка последовательного преобразования каналов для ADC2
    ADC2->SQR1 = (1 << ADC_SQR1_SQ1_Pos) | // Первый канал 1
                 (2 << ADC_SQR1_SQ2_Pos) | // Второй канал 2
                 (3 << ADC_SQR1_SQ3_Pos);  // Третий канал 3

    ADC2->SQR1 |= (3 << ADC_SQR1_L_Pos); // Указываем, что 3 преобразования


    // Настройка времени выборки для каналов 1-3
       ADC2->SMPR1 &= ~((0x7 << ADC_SMPR1_SMP1_Pos) | // Очистка старых значений
                        (0x7 << ADC_SMPR1_SMP2_Pos) |
                        (0x7 << ADC_SMPR1_SMP3_Pos));  // Каналы 1-3

       ADC2->SMPR1 |= (ADC_SAMPLING_TIME_601_5 << ADC_SMPR1_SMP1_Pos) | // Канал 1
                      (ADC_SAMPLING_TIME_601_5 << ADC_SMPR1_SMP2_Pos) | // Канал 2
                      (ADC_SAMPLING_TIME_601_5 << ADC_SMPR1_SMP3_Pos);  // Канал 3

       // Запуск калибровки ADC2
       ADC2->CR &= ~ADC_CR_ADEN; // Отключить ADC перед калибровкой
       ADC2->CR |= ADC_CR_ADCAL; // Запуск калибровки
       while (ADC2->CR & ADC_CR_ADCAL); // Ожидание завершения калибровки

       // Включение ADC2
       ADC2->CR |= ADC_CR_ADEN;
       while (!(ADC2->ISR & ADC_ISR_ADRDY)); // Ожидание готовности ADC2

}

void ADC1_StartConversion(uint16_t *data_buffer) {
	 ADC1->ISR |= ADC_ISR_EOC | ADC_ISR_EOS| ADC_ISR_OVR; // Сброс флагов завершения преобразования
    // Запуск преобразования для ADC1
    ADC1->CR |= ADC_CR_ADSTART;

    uint32_t timeout;

    while (!(ADC1->ISR & ADC_ISR_EOS)) { // Ожидаем завершения последовательности
        for (int i = 0; i < 4; i++) {
            timeout = 100000; // Тайм-аут для ожидания

            while (!(ADC1->ISR & ADC_ISR_EOC)) { // Ожидаем завершения преобразования
                if (--timeout == 0) {
                    if (ADC1->ISR & ADC_ISR_OVR) { // Проверка на переполнение
                    	 LED_6_ON;
                    }

                    return;
                }
            }

            data_buffer[i] = ADC1->DR; // Сохраняем данные в буфер
        }
    }

    ADC1->ISR |= ADC_ISR_EOS; // Сброс флага завершения последовательности
}


void ADC2_StartConversion(uint16_t *data_buffer) {
	 ADC2->ISR |= ADC_ISR_EOC | ADC_ISR_EOS| ADC_ISR_OVR; // Сброс флагов завершения преобразования
    // Запуск преобразования для ADC2
    ADC2->CR |= ADC_CR_ADSTART;
    uint32_t timeout;

       while (!(ADC2->ISR & ADC_ISR_EOS)) { // Ожидаем завершения последовательности
           for (int i = 0; i < 3; i++) {
               timeout = 100000; // Тайм-аут для ожидания

               while (!(ADC2->ISR & ADC_ISR_EOC)) { // Ожидаем завершения преобразования
                   if (--timeout == 0) {
                       if (ADC2->ISR & ADC_ISR_OVR) { // Проверка на переполнение
                       	 LED_6_ON;
                       }

                       return;
                   }
               }

               data_buffer[i] = ADC2->DR; // Сохраняем данные в буфер
           }
       }

       ADC2->ISR |= ADC_ISR_EOS; // Сброс флага завершения последовательности
   }

