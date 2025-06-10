/* keypad_config.h - Interfaz del teclado 4x4 para Robo-Bar */

#ifndef KEYPAD_CONFIG_H
#define KEYPAD_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Devuelve el carácter correspondiente a la tecla presionada
 * @return Carácter ASCII ('0'-'9', 'A'-'D', '*', '#'), o 0 si no hay tecla
 */
char Keypad_GetKeyChar(void);

/**
 * @brief Espera hasta que una tecla sea presionada y devuelve el carácter
 * @return Carácter de la tecla presionada
 */
char Keypad_WaitForKey(void);

#ifdef __cplusplus
}
#endif

#endif /* KEYPAD_CONFIG_H */
