#include "MKL25Z4.h"
#include <stdint.h>

#define RELAY_PIN 31 // PTE31
#define RELAY_PORT_CLK SIM_SCGC5_PORTE_MASK

// Configuración PWM para servo
#define PWM_MOD 20833
#define SERVO_MIN_PULSE 1041    // 0° - Válvula cerrada
#define SERVO_MAX_PULSE 4167    // 180° - Válvula abierta
#define SERVO_90_PULSE 2604     // 90° - Posición intermedia

void RELAY_Init(void) {
    SIM->SCGC5 |= RELAY_PORT_CLK;
    PORTE->PCR[RELAY_PIN] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
    GPIOE->PDDR |= (1U << RELAY_PIN);
    GPIOE->PCOR = (1U << RELAY_PIN); // Relé apagado al inicio (lógica inversa)
}

void RELAY_On(void) {
    GPIOE->PSOR = (1U << RELAY_PIN); // Relé ON (lógica inversa - HIGH activa)
}

void RELAY_Off(void) {
    GPIOE->PCOR = (1U << RELAY_PIN); // Relé OFF (lógica inversa - LOW desactiva)
}

void delay_ms(uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms * 8000U; i++) __asm("nop");
}

void servo5_init(void) {
    // Habilitar relojes para puerto D y TPM0
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // TPM usa MCGFLLCLK (48 MHz)

    // Configurar PTD0 como TPM0_CH0 para servo 5
    PORTD->PCR[0] = PORT_PCR_MUX(4); // PTD0 → TPM0_CH0

    // Configurar TPM0 para CH0 (servo 5)
    TPM0->SC = 0; // Detener TPM0
    TPM0->MOD = PWM_MOD; // Período de 20ms
    TPM0->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // PWM modo edge-aligned
    TPM0->CONTROLS[0].CnV = SERVO_MIN_PULSE; // Iniciar en 0° (válvula cerrada)
    TPM0->SC = TPM_SC_PS(4) | TPM_SC_CMOD(1); // Prescaler /16, iniciar TPM
}

void servo5_set_angle(uint16_t angle) {
    uint16_t pulse;

    if (angle == 0) {
        pulse = SERVO_MIN_PULSE;    // 0° - Válvula cerrada
    } else if (angle == 90) {
        pulse = SERVO_90_PULSE;     // 90° - Posición intermedia
    } else if (angle == 180) {
        pulse = SERVO_MAX_PULSE;    // 180° - Válvula abierta
    } else {
        // Interpolación lineal para ángulos intermedios
        pulse = SERVO_MIN_PULSE + ((SERVO_MAX_PULSE - SERVO_MIN_PULSE) * angle) / 180;
    }

    TPM0->CONTROLS[0].CnV = pulse;
}

void ServirBebida(void) {
    // Abrir válvula completamente (servo 5 en PTD0)
    servo5_set_angle(180);
    delay_ms(150); // Tiempo de dispensado

    // Cerrar válvula completamente
    servo5_set_angle(0);
}

void TestServo5(void) {
    // Test del servo 5 únicamente
    servo5_set_angle(0);    // Cerrar válvula
    delay_ms(500);

    servo5_set_angle(90);   // Posición intermedia
    delay_ms(500);

    servo5_set_angle(180);  // Abrir válvula
    delay_ms(500);

    servo5_set_angle(0);    // Cerrar válvula
    delay_ms(500);
}

int main(void) {
    RELAY_Init();
    servo5_init(); // Inicializar solo servo 5

    // TEST DEL SERVO 5 - Ejecuta solo una vez al inicio
    //TestServo5();

    // Pausa larga para observar el test antes de continuar
    //delay_ms(2000);

    while (1) {
        RELAY_On();     // Mezclar (relé activado)
        delay_ms(200);  // Tiempo de mezclado
        RELAY_Off();    // Detener mezclado

        TestServo5();
        //ServirBebida(); // Servir bebida usando servo 5
        delay_ms(300);  // Espera entre ciclos
    }
}
