/* keypad_config.c - Keypad interface functions for Robo-Bar */

#include "gpio_config.h"
#include <stdint.h>
#include <stdbool.h>
#include "fsl_gpio.h"

#define KEYPAD_NO_KEY 0xFF

/* Traduce código (0-15) a tecla real */
static const char keypad_lookup[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

char Keypad_GetKeyChar(void) {
    uint8_t key = gpio_keypad_scan();
    if (key == KEYPAD_NO_KEY || key > 15) return 0;
    return keypad_lookup[key / 4][key % 4];
}

char Keypad_WaitForKey(void) {
    char key = 0;
    while (!key) {
        key = Keypad_GetKeyChar();
    }
    return key;
}
