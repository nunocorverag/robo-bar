#include "MKL25Z4.h"
#include "lcd_i2c.h"
#include "FreeRTOS.h"
#include "task.h"

// Pines I2C: PTC10 = SDA, PTC11 = SCL
#define I2C_TRAN        I2C1->C1 |= I2C_C1_TX_MASK
#define I2C_REC         I2C1->C1 &= ~I2C_C1_TX_MASK
#define I2C_M_START     I2C1->C1 |= I2C_C1_MST_MASK
#define I2C_M_STOP      I2C1->C1 &= ~I2C_C1_MST_MASK
#define I2C_WAIT        while (!(I2C1->S & I2C_S_IICIF_MASK)); I2C1->S |= I2C_S_IICIF_MASK
#define I2C_ACK_CHECK   (I2C1->S & I2C_S_RXAK_MASK)

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

static bool i2c_initialized = false;
static bool lcd_initialized = false;

static void lcd_delay_ms(uint32_t ms) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
        volatile uint32_t delay = ms * 1000;
        while (delay--);
    }
}

static void lcd_delay_us(uint32_t us) {
    volatile uint32_t delay = us * 10;
    while (delay--);
}

void i2c_init(void) {
    if (i2c_initialized) return;

    SIM->SCGC4 |= SIM_SCGC4_I2C1_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

    PORTC->PCR[10] = PORT_PCR_MUX(2); // SDA
    PORTC->PCR[11] = PORT_PCR_MUX(2); // SCL

    I2C1->F = (I2C_F_ICR(0x1F) | I2C_F_MULT(0));
    I2C1->C1 = I2C_C1_IICEN_MASK;
    I2C1->S |= I2C_S_IICIF_MASK;

    i2c_initialized = true;
}

static bool i2c_start_transaction(uint8_t addr) {
    int timeout = 1000;
    while ((I2C1->S & I2C_S_BUSY_MASK) && timeout--) lcd_delay_us(1);
    if (timeout <= 0) return false;

    I2C_TRAN;
    I2C_M_START;
    I2C1->D = addr;
    I2C_WAIT;

    if (I2C_ACK_CHECK) {
        I2C_M_STOP;
        return false;
    }
    return true;
}

static bool i2c_write_byte(uint8_t data) {
    I2C1->D = data;
    I2C_WAIT;
    return !I2C_ACK_CHECK;
}

static void i2c_stop_transaction(void) {
    I2C_M_STOP;
    lcd_delay_us(100);
}

static bool lcd_write_nibble(uint8_t nibble, uint8_t rs) {
    uint8_t data = nibble | LCD_BL | rs;
    if (!i2c_write_byte(data)) return false;
    lcd_delay_us(1);
    if (!i2c_write_byte(data | LCD_EN)) return false;
    lcd_delay_us(1);
    if (!i2c_write_byte(data)) return false;
    lcd_delay_us(50);
    return true;
}

static bool lcd_write_byte(uint8_t data, uint8_t mode) {
    if (!lcd_initialized) return false;
    uint8_t hi = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;
    uint8_t rs = mode ? LCD_RS : 0;

    if (!i2c_start_transaction(LCD_ADDR_W)) return false;
    bool success = lcd_write_nibble(hi, rs) && lcd_write_nibble(lo, rs);
    i2c_stop_transaction();

    if (data == CMD_CLEAR || data == CMD_HOME)
        lcd_delay_ms(2);
    else
        lcd_delay_us(40);

    return success;
}

bool lcd_init(void) {
    if (lcd_initialized) return true;
    i2c_init();
    lcd_delay_ms(50);

    for (int i = 0; i < 3; i++) {
        if (!i2c_start_transaction(LCD_ADDR_W)) continue;
        if (!lcd_write_nibble(0x30, 0)) {
            i2c_stop_transaction();
            continue;
        }
        i2c_stop_transaction();
        lcd_delay_ms(i == 0 ? 5 : 1);
    }

    if (!i2c_start_transaction(LCD_ADDR_W)) return false;
    if (!lcd_write_nibble(0x20, 0)) {
        i2c_stop_transaction();
        return false;
    }
    i2c_stop_transaction();
    lcd_delay_ms(1);

    lcd_initialized = true;

    if (!lcd_write_byte(CMD_FUNC_SET, 0)) return false;
    if (!lcd_write_byte(CMD_DISP_ON, 0)) return false;
    if (!lcd_write_byte(CMD_CLEAR, 0)) return false;
    if (!lcd_write_byte(CMD_ENTRY_MODE, 0)) return false;

    return true;
}

bool lcd_clear(void) {
    return lcd_write_byte(CMD_CLEAR, 0);
}

bool lcd_set_cursor(uint8_t row, uint8_t col) {
    if (row > 1 || col > 15) return false;
    uint8_t offsets[] = {0x00, 0x40};
    return lcd_write_byte(0x80 | (offsets[row] + col), 0);
}

bool lcd_print(const char *s) {
    if (!s) return false;
    while (*s) {
        if (!lcd_write_byte(*s++, 1)) return false;
    }
    return true;
}

bool lcd_is_initialized(void) {
    return lcd_initialized;
}

void lcd_reset(void) {
    lcd_initialized = false;
    i2c_initialized = false;
}
