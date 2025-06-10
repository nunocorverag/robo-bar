#include "MKL25Z4.h"
#include "keypad.h"

// PTC0–3 = columnas (inputs con pull-up), PTC4–7 = filas (outputs)

void keypad_init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

    PORTC->PCR[0] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTC->PCR[1] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTC->PCR[2] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTC->PCR[3] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

    PORTC->PCR[4] = PORT_PCR_MUX(1);
    PORTC->PCR[5] = PORT_PCR_MUX(1);
    PORTC->PCR[6] = PORT_PCR_MUX(1);
    PORTC->PCR[7] = PORT_PCR_MUX(1);

    GPIOC->PDDR = (1 << 4)|(1 << 5)|(1 << 6)|(1 << 7);
    GPIOC->PDOR = (1 << 4)|(1 << 5)|(1 << 6)|(1 << 7);
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
        GPIOC->PDOR = ~(1 << (row + 4));
        for (volatile int d = 0; d < 1000; d++); // Debounce corto

        for (int col = 0; col < 4; col++) {
            if (!(GPIOC->PDIR & (1 << col))) {
                char current_key = keys[row][col];
                if (current_key != last_key) {
                    last_key = current_key;
                    return current_key; // Se detecta una nueva tecla
                }
                return 0; // Ya fue detectada
            }
        }
        GPIOC->PDOR = (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7);
    }

    last_key = 0; // Si no se detectó nada, se reinicia el estado
    return 0;
}
