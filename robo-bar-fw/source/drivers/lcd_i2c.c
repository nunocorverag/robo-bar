#include "MKL25Z4.h"
#include "lcd_i2c.h"

// I2C por PTC10=SDA y PTC11=SCL
#define I2C_TRAN    I2C1->C1 |= I2C_C1_TX_MASK
#define I2C_REC     I2C1->C1 &= ~I2C_C1_TX_MASK
#define I2C_M_START I2C1->C1 |= I2C_C1_MST_MASK
#define I2C_M_STOP  I2C1->C1 &= ~I2C_C1_MST_MASK
#define I2C_WAIT    while(!(I2C1->S & I2C_S_IICIF_MASK)); I2C1->S |= I2C_S_IICIF_MASK

#define LCD_ADDR        0x27
#define LCD_ADDR_W      (LCD_ADDR << 1)
#define LCD_BL          0x08
#define LCD_EN          0x04
#define LCD_RS          0x01

#define CMD_CLEAR       0x01
#define CMD_HOME        0x02
#define CMD_ENTRY_MODE  0x06
#define CMD_DISP_ON     0x0C
#define CMD_FUNC_SET    0x28

void Delay(volatile unsigned int time) {
    while (time--) for (volatile int i = 0; i < 1000; i++);
}

void i2c_init(void) {
    SIM->SCGC4 |= SIM_SCGC4_I2C1_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    PORTC->PCR[10] = PORT_PCR_MUX(2);
    PORTC->PCR[11] = PORT_PCR_MUX(2);

    I2C1->F = (I2C_F_ICR(0x1F)|I2C_F_MULT(0));
    I2C1->C1 = I2C_C1_IICEN_MASK;
    I2C1->S |= I2C_S_IICIF_MASK;
}

static void lcd_write_byte(uint8_t data, uint8_t mode) {
    uint8_t hi = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;
    uint8_t rs = mode ? LCD_RS : 0;

    I2C_TRAN; I2C_M_START;
    I2C1->D = LCD_ADDR_W; I2C_WAIT;
    I2C1->D = hi | LCD_BL | rs; I2C_WAIT;
    I2C1->D = hi | LCD_BL | LCD_EN | rs; I2C_WAIT;
    Delay(1);
    I2C1->D = hi | LCD_BL | rs; I2C_WAIT;

    I2C1->D = lo | LCD_BL | rs; I2C_WAIT;
    I2C1->D = lo | LCD_BL | LCD_EN | rs; I2C_WAIT;
    Delay(1);
    I2C1->D = lo | LCD_BL | rs; I2C_WAIT;

    I2C_M_STOP;
    Delay(1);
}

void lcd_init(void) {
    Delay(50);
    for (int i = 0; i < 3; i++) {
        I2C_TRAN; I2C_M_START;
        I2C1->D = LCD_ADDR_W; I2C_WAIT;
        I2C1->D = 0x30 | LCD_BL; I2C_WAIT;
        I2C1->D = 0x30 | LCD_BL | LCD_EN; I2C_WAIT;
        Delay(1);
        I2C1->D = 0x30 | LCD_BL; I2C_WAIT;
        I2C_M_STOP;
        Delay(i == 2 ? 5 : 1);
    }
    I2C_TRAN; I2C_M_START;
    I2C1->D = LCD_ADDR_W; I2C_WAIT;
    I2C1->D = 0x20 | LCD_BL; I2C_WAIT;
    I2C1->D = 0x20 | LCD_BL | LCD_EN; I2C_WAIT;
    Delay(1);
    I2C1->D = 0x20 | LCD_BL; I2C_WAIT;
    I2C_M_STOP;
    Delay(1);

    lcd_write_byte(CMD_FUNC_SET, 0);
    lcd_write_byte(CMD_DISP_ON, 0);
    lcd_write_byte(CMD_CLEAR, 0);
    Delay(2);
    lcd_write_byte(CMD_ENTRY_MODE, 0);
}

void lcd_clear(void) {
    lcd_write_byte(CMD_CLEAR, 0);
    Delay(2);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t offsets[] = {0x00, 0x40};
    lcd_write_byte(0x80 | (offsets[row] + col), 0);
}

void lcd_print(const char *s) {
    while (*s) lcd_write_byte(*s++, 1);
}
