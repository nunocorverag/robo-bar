#ifndef LCD_I2C_H
#define LCD_I2C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * API Functions
 ******************************************************************************/

/**
 * @brief Inicializa el módulo I2C y el LCD
 * @return true si la inicialización fue exitosa, false en caso contrario
 */
bool lcd_init(void);

/**
 * @brief Limpia completamente la pantalla LCD
 * @return true si el comando fue exitoso, false en caso contrario
 */
bool lcd_clear(void);

/**
 * @brief Establece la posición del cursor
 * @param row Fila (0-1 para LCD 16x2)
 * @param col Columna (0-15 para LCD 16x2)
 * @return true si el comando fue exitoso, false en caso contrario
 */
bool lcd_set_cursor(uint8_t row, uint8_t col);

/**
 * @brief Imprime una cadena de texto en el LCD
 * @param s Puntero a la cadena de texto (terminada en null)
 * @return true si el comando fue exitoso, false en caso contrario
 */
bool lcd_print(const char *s);

/**
 * @brief Verifica si el LCD está inicializado
 * @return true si está inicializado, false en caso contrario
 */
bool lcd_is_initialized(void);

/**
 * @brief Reinicia el estado del LCD (para recovery de errores)
 */
void lcd_reset(void);

/**
 * @brief Inicializa el módulo I2C (llamada automáticamente por lcd_init)
 */
void i2c_init(void);

#ifdef __cplusplus
}
#endif

#endif /* LCD_I2C_H */