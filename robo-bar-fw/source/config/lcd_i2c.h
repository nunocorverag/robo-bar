#ifndef LCD_I2C_H
#define LCD_I2C_H

#include <stdint.h>

void LCD_init(void);
void LCD_clear(void);
void LCD_setCursor(uint8_t row, uint8_t col);
void LCD_print(const char *str);
void LCD_command(uint8_t cmd);
void LCD_data(uint8_t data);
void delayMs(int n);

#endif /* LCD_I2C_H */