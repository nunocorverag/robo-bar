/* lcd_i2c.c - Controlador para pantalla LCD 16x2 con interfaz I2C */

#include "lcd_i2c.h"
#include "fsl_i2c.h"  // O tu librería de I2C
#include <MKL25Z4.h>
#include <string.h>
#include <stdio.h>

#define LCD_I2C_ADDR       (0x27 << 1)
#define LCD_BACKLIGHT      0x08
#define LCD_ENABLE         0x04
#define LCD_RS             0x01

extern void delayMs(int n); // Implementación en otro archivo o en este

static void I2C_SendByte(uint8_t data) {
    I2C1->C1 |= I2C_C1_TX_MASK | I2C_C1_MST_MASK; // Transmit + Master
    I2C1->D = LCD_I2C_ADDR;
    while (!(I2C1->S & I2C_S_IICIF_MASK));
    I2C1->S |= I2C_S_IICIF_MASK;

    I2C1->D = data;
    while (!(I2C1->S & I2C_S_IICIF_MASK));
    I2C1->S |= I2C_S_IICIF_MASK;

    I2C1->C1 &= ~I2C_C1_MST_MASK; // STOP
}

static void LCD_WriteNibble(uint8_t nibble, uint8_t control) {
    uint8_t data = nibble | LCD_BACKLIGHT | control;
    I2C_SendByte(data);
    I2C_SendByte(data | LCD_ENABLE);
    delayMs(1);
    I2C_SendByte(data & ~LCD_ENABLE);
}

void LCD_command(uint8_t cmd) {
    LCD_WriteNibble(cmd & 0xF0, 0);
    LCD_WriteNibble((cmd << 4) & 0xF0, 0);
}

void LCD_data(uint8_t data) {
    LCD_WriteNibble(data & 0xF0, LCD_RS);
    LCD_WriteNibble((data << 4) & 0xF0, LCD_RS);
}

void LCD_clear(void) {
    LCD_command(0x01);
    delayMs(2);
}

void LCD_setCursor(uint8_t row, uint8_t col) {
    uint8_t offsets[] = {0x00, 0x40};
    LCD_command(0x80 | (offsets[row] + col));
}

void LCD_print(const char *str) {
    while (*str) {
        LCD_data(*str++);
    }
}

void LCD_init(void) {
    delayMs(50);
    LCD_command(0x33);
    LCD_command(0x32);
    LCD_command(0x28);
    LCD_command(0x0C);
    LCD_command(0x06);
    LCD_command(0x01);
    delayMs(2);
}
