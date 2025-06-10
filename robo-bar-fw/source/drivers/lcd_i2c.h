#ifndef LCD_I2C_H_
#define LCD_I2C_H_

void i2c_init(void);
void lcd_init(void);
void lcd_clear(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_print(const char *str);
void Delay(volatile unsigned int);

#endif /* LCD_I2C_H_ */
