/*
 * I2C_display.h
 *
 *  Created on: Jun 12, 2024
 *      Author: chipi
 */

#ifndef INC_I2C_DISPLAY_H_
#define INC_I2C_DISPLAY_H_

#include "main.h"

extern I2C_HandleTypeDef hi2c1;
extern char *text;
extern char int_to_str[10];




//////////////////////////////////////////////////////////////////////////////////////////////

#define LCD_ADDR (0x27 << 1)       // адрес дисплея, сдвинутый на 1 бит влево (HAL работает с I2C-адресами, сдвинутыми на 1 бит влево)

#define PIN_RS    (1 << 0)         // если на ножке 0, данные воспринимаются как команда, если 1 - как символы для вывода
#define PIN_EN    (1 << 2)         // бит, по изменению сост. которого считывается информация
#define BACKLIGHT (1 << 3)         // управление подсветкой

#define LCD_DELAY_MS 5             // пауза перед высвечиванием символа

//////////////////////////////////////////////////////////////////////////////////////////////

#define I2C_ADDR 0x27 // I2C address of the PCF8574
#define RS_BIT 0 // Register select bit
#define EN_BIT 2 // Enable bit
#define BL_BIT 3 // Backlight bit
#define D4_BIT 4 // Data 4 bit
#define D5_BIT 5 // Data 5 bit
#define D6_BIT 6 // Data 6 bit
#define D7_BIT 7 // Data 7 bit

#define LCD_ROWS 4 // Number of rows on the LCD
#define LCD_COLS 20 // Number of columns on the LCD

#define N 20

#define I2C_SLAVE_ADDRESS   0x2D
/////////////////////////////////////////////////////////////


extern I2C_HandleTypeDef hi2c1;
extern char *text;
extern char int_to_str[10];

void I2C_send(uint8_t data, uint8_t flags);
void LCD_SendString(char *str);
void lcd_write_nibble(uint8_t nibble, uint8_t rs);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_init();
void lcd_write_string(char *str);
void lcd_set_cursor(uint8_t row, uint8_t column);
void lcd_clear(void);
void lcd_backlight(uint8_t state);





#endif /* INC_I2C_DISPLAY_H_ */
