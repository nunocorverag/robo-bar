#include "MKL25Z4.h"
#include "lcd_i2c.h"

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

// Delays simples sin SysTick
static void lcd_delay_ms(uint32_t ms) {
    volatile uint32_t delay = ms * (SystemCoreClock / 1000 / 4);
    while (delay--);
}

static void lcd_delay_us(uint32_t us) {
    volatile uint32_t delay = us * (SystemCoreClock / 1000000 / 4);
    while (delay--);
}

void i2c_init(void) {
    if (i2c_initialized) return;

    // Habilitar clocks
    SIM->SCGC4 |= SIM_SCGC4_I2C1_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

    // Configurar pines
    PORTC->PCR[10] = PORT_PCR_MUX(2); // SDA
    PORTC->PCR[11] = PORT_PCR_MUX(2); // SCL

    // Configurar I2C - frecuencia ~100kHz
    I2C1->F = (I2C_F_ICR(0x1F) | I2C_F_MULT(0));
    I2C1->C1 = I2C_C1_IICEN_MASK;
    I2C1->S |= I2C_S_IICIF_MASK;

    i2c_initialized = true;
}

static bool i2c_start_transaction(uint8_t addr) {
    // Esperar a que el bus esté libre (con timeout simple)
    int timeout = 10000;
    while ((I2C1->S & I2C_S_BUSY_MASK) && timeout--) {
        // Pequeña espera
    }
    
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
        lcd_delay_us(50);

    return success;
}

bool lcd_init(void) {
    if (lcd_initialized) return true;
    
    i2c_init();
    lcd_delay_ms(100); // Dar tiempo al LCD para inicializar
    
    // Intentar comunicación básica primero
    bool communication_ok = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        if (i2c_start_transaction(LCD_ADDR_W)) {
            i2c_write_byte(0x00); // Escribir 0 para probar comunicación
            i2c_stop_transaction();
            communication_ok = true;
            break;
        }
        lcd_delay_ms(10);
    }
    
    if (!communication_ok) {
        return false; // No se pudo establecer comunicación I2C
    }
    
    // Secuencia de inicialización LCD en modo 4-bit
    for (int i = 0; i < 3; i++) {
        if (i2c_start_transaction(LCD_ADDR_W)) {
            lcd_write_nibble(0x30, 0);
            i2c_stop_transaction();
        }
        lcd_delay_ms(i == 0 ? 5 : 2);
    }

    // Cambiar a modo 4-bit
    if (i2c_start_transaction(LCD_ADDR_W)) {
        lcd_write_nibble(0x20, 0);
        i2c_stop_transaction();
    }
    lcd_delay_ms(2);

    lcd_initialized = true;

    // Configurar LCD
    lcd_write_byte(CMD_FUNC_SET, 0);    // 4-bit, 2 líneas
    lcd_write_byte(CMD_DISP_ON, 0);     // Display ON
    lcd_write_byte(CMD_CLEAR, 0);       // Limpiar
    lcd_write_byte(CMD_ENTRY_MODE, 0);  // Modo de entrada

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

// Función de prueba para verificar comunicación I2C
bool lcd_test_communication(void) {
    if (!i2c_initialized) i2c_init();
    
    // Intentar comunicación simple
    if (i2c_start_transaction(LCD_ADDR_W)) {
        bool success = i2c_write_byte(0x00);
        i2c_stop_transaction();
        return success;
    }
    return false;
}