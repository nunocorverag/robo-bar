#include "MKL25Z4.h"
#include "servo_control.h"

#define SERVO_MOD 20833  // 50 Hz PWM (48 MHz / 16 / 20833 = ~60 Hz)
#define SERVO_PSC 4      // Prescaler = 16

// TPM y canales para cada servo: [0]–PTD4, [1]–PTA12, [2]–PTA13, [3]–PTD5, [4]–PTD0
static TPM_Type* servo_tpms[5] = { TPM0, TPM1, TPM1, TPM0, TPM0 };
static uint8_t servo_channels[5] = { 4, 0, 1, 5, 0 };

void servo_control_init(void) {
    // Relojes para puertos y TPMs
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTD_MASK;
    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK | SIM_SCGC6_TPM1_MASK;

    // Configuración de pines como salidas PWM
    PORTD->PCR[4] = PORT_PCR_MUX(4);  // PTD4 → TPM0_CH4 (Servo 0)
    PORTA->PCR[12] = PORT_PCR_MUX(3); // PTA12 → TPM1_CH0 (Servo 1)
    PORTA->PCR[13] = PORT_PCR_MUX(3); // PTA13 → TPM1_CH1 (Servo 2)
    PORTD->PCR[5] = PORT_PCR_MUX(4);  // PTD5 → TPM0_CH5 (Servo 3)
    PORTD->PCR[0] = PORT_PCR_MUX(4);  // PTD0 → TPM0_CH0 (Servo 4 - nuevo)

    // TPM0 configuración general
    TPM0->SC = 0;
    TPM0->MOD = SERVO_MOD;
    TPM0->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // Servo 4
    TPM0->CONTROLS[4].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // Servo 0
    TPM0->CONTROLS[5].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // Servo 3
    TPM0->SC = TPM_SC_PS(SERVO_PSC) | TPM_SC_CMOD(1);

    // TPM1 configuración general
    TPM1->SC = 0;
    TPM1->MOD = SERVO_MOD;
    TPM1->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // Servo 1
    TPM1->CONTROLS[1].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // Servo 2
    TPM1->SC = TPM_SC_PS(SERVO_PSC) | TPM_SC_CMOD(1);

    // Posición inicial: 90°
    for (int i = 0; i < 5; i++) {
        servo_control_set_angle(i, 90);
    }
}

void servo_control_set_angle(uint8_t servo_id, uint8_t angle) {
    if (servo_id >= 5) return;
    uint16_t pulse = 1041 + ((angle * (2083 - 1041)) / 180); // 1ms–2ms mapeado
    servo_tpms[servo_id]->CONTROLS[servo_channels[servo_id]].CnV = pulse;
}
