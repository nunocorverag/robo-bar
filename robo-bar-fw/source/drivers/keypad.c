#include "MKL25Z4.h"
#include "keypad.h"

void keypad_init(void) {
    // Habilitar reloj para PORTB
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

    // Configurar filas (PTB0-PTB3) como GPIO outputs
    PORTB->PCR[0] = PORT_PCR_MUX(1); // PTB0 como GPIO
    PORTB->PCR[1] = PORT_PCR_MUX(1); // PTB1
    PORTB->PCR[2] = PORT_PCR_MUX(1); // PTB2
    PORTB->PCR[3] = PORT_PCR_MUX(1); // PTB3

    GPIOB->PDDR |= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);  // Set output
    GPIOB->PDOR |= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);  // Inicialmente en alto

    // Configurar columnas (PTB8-PTB11) como GPIO inputs con pull-up
    PORTB->PCR[8] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;  // Pull-up
    PORTB->PCR[9] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB->PCR[10] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB->PCR[11] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

    GPIOB->PDDR &= ~((1 << 8) | (1 << 9) | (1 << 10) | (1 << 11)); // Inputs
}

char keypad_getkey(void) {
    const char keys[4][4] = {
        {'1', '2', '3', 'A'},
        {'4', '5', '6', 'B'},
        {'7', '8', '9', 'C'},
        {'*', '0', '#', 'D'}
    };

    static char last_key = 0;

    for (int row = 0; row < 4; row++) {
        // Poner la fila actual en bajo y las demás en alto
        GPIOB->PDOR = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3); // Todas filas en alto
        GPIOB->PDOR &= ~(1 << row); // fila actual en bajo

        for (volatile int d = 0; d < 1000; d++); // debounce corto

        for (int col = 0; col < 4; col++) {
            if (!(GPIOB->PDIR & (1 << (col + 8)))) { // columnas PTB8-11
                char current_key = keys[row][col];
                if (current_key != last_key) {
                    last_key = current_key;
                    return current_key; // Nueva tecla detectada
                }
                return 0; // Ya fue detectada antes
            }
        }
    }

    last_key = 0; // No se detectó tecla
    return 0;
}
