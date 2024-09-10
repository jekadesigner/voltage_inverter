


#include "menu.h"
#include "I2C_display.h"

// Функция инициализации меню
void Menu_Init(Menu *menu) {
    menu->currentState = MENU_MAIN_VIEW;
    menu->currentOption = 0;
    Menu_UpdateDisplay(menu);
}

// Функция обработки нажатий кнопок
void Menu_HandleButtonPress(Menu *menu, uint8_t button) {


    switch (button) {
        case 1: // Вверх по меню
        	 menu->currentState--;
            if ((menu->currentState <=0)||(menu->currentState>=MENU_STATE_COUNT)) {

                menu->currentState = MENU_MAIN_VIEW;
            }
            break;
        case 2: // Вниз по меню
            menu->currentState++;
            if (menu->currentState >= MENU_STATE_COUNT) {
                menu->currentState = MENU_MAIN_VIEW;
            }
            break;
        case 3: // Инкрементирование
                   if (menu->currentState == MENU_MAIN_VIEW) {
                       pwm_buck += 10;
                       if ((pwm_buck > 2000)||(pwm_buck <=0 )) {
                           pwm_buck =2000;
                       }
                   } else {
                       menu->currentOption++;
                   }
                   break;
               case 4: // Декрементирование
                   if (menu->currentState == MENU_MAIN_VIEW) {
                       pwm_buck -= 10;
                       if ((pwm_buck <0)||(pwm_buck>2000)) {
                           pwm_buck = 0;
                       }
                   } else {
                       menu->currentOption--;
                   }
                   break;
               default:
                   break;
    }

}

// Функция обновления дисплея
void Menu_UpdateDisplay(Menu *menu) {
   // static uint8_t previousState = 255;  // Инициализируем значение, которое не совпадает с MENU_STATE_xxx

    // Проверяем, изменилось ли текущее состояние меню
  //  if (menu->currentState != previousState) {
     //   lcd_clear();  // Очищаем экран только если состояние изменилось
        switch (menu->currentState) {
            case MENU_MAIN_VIEW:
                lcd_set_cursor(1, 0);
                lcd_write_string("Main Menu           ");

              //  lcd_set_cursor(2, 0);
              // 	sprintf(int_to_str, "1-%04d,2-%04d,3-%04d", 0,0,0 );
              // 	lcd_write_string(int_to_str);
               	/*
               	lcd_set_cursor(3, 0);
            	sprintf(int_to_str,"1-%04d,2-%04d,3-%04d", adc_buffer_1[3],adc_buffer_1[4],adc_buffer_1[5]);
            	lcd_write_string(int_to_str);

            	lcd_set_cursor(4, 0);
            	sprintf(int_to_str,"1-%04d,2-%04d,3-%04d", adc_buffer_1[0],adc_buffer_1[1],adc_buffer_1[2]);
            	lcd_write_string(int_to_str);


            	 lcd_set_cursor(2, 0);
            	 sprintf(int_to_str, "1-%04d,2-%04d,3-%04d", ADC_1_MAX, ADC_1_MIN, ADC_1_MID );
            	 lcd_write_string(int_to_str);

				lcd_set_cursor(3, 0);
				sprintf(int_to_str,"1-%04d,2-%04d,3-%04d", ADC_2_MAX,ADC_2_MIN, ADC_2_MID);
				lcd_write_string(int_to_str);

				lcd_set_cursor(4, 0);
				sprintf(int_to_str,"1-%04d,2-%04d,3-%04d", ADC_3_MAX,ADC_3_MIN, ADC_2_MID);
				lcd_write_string(int_to_str);
   */
                break;
            case MENU_SUPPLY_VIEW:
                lcd_set_cursor(1, 0);
                lcd_write_string("Menu Supply         ");

                break;
            case MENU_CONNECT_VIEW:
                lcd_set_cursor(1, 0);
                lcd_write_string("Menu Connection     ");

                break;
            case MENU_ALARM_VIEW:
                lcd_set_cursor(1, 0);
                lcd_write_string("Menu Alarm          ");

                break;
            default:
                break;
        }

        // Обновляем предыдущее состояние
      //  previousState = menu->currentState;
  //  }
}
