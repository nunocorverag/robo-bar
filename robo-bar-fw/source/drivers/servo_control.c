#include "MKL25Z4.h"
#include "servo_control.h"

#define SERVO_MOD 20833  // 50 Hz PWM
#define SERVO_PSC 4      // Prescaler = 16

// Servos conectados a: PTD4 (TPM0_CH4), PTA12 (TPM1_CH0), PTA13 (TPM1_CH1), PTA5 (TPM0_CH2)
static TPM_Type* servo_tpms[4] = { TPM0, TPM1, TPM1, TPM0 };
static uint8_t   servo_channels[4] = { 4, 0, 1, 2 };

void servo_control_init(void) {
    // Clocks para GPIO y TPMs
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTD_MASK;
    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK | SIM_SCGC6_TPM1_MASK;

    // Mux para PWM (ver hoja de datos)
    PORTD->PCR[4]  = PORT_PCR_MUX(4); // PTD4 -> TPM0_CH4
    PORTA->PCR[12] = PORT_PCR_MUX(3); // PTA12 -> TPM1_CH0
    PORTA->PCR[13] = PORT_PCR_MUX(3); // PTA13 -> TPM1_CH1
    PORTA->PCR[5]  = PORT_PCR_MUX(3); // PTA5  -> TPM0_CH2

    // TPM0
    TPM0->SC = 0;
    TPM0->MOD = SERVO_MOD;
    TPM0->SC = TPM_SC_PS(SERVO_PSC) | TPM_SC_CMOD(1);

    // TPM1
    TPM1->SC = 0;
    TPM1->MOD = SERVO_MOD;
    TPM1->SC = TPM_SC_PS(SERVO_PSC) | TPM_SC_CMOD(1);

    // Canales PWM (MSB + ELSB = Edge-aligned PWM)
    TPM0->CONTROLS[2].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // CH2
    TPM0->CONTROLS[4].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // CH4
    TPM1->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // CH0
    TPM1->CONTROLS[1].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // CH1

    // Ángulo inicial
    for (int i = 0; i < 4; i++) {
        servo_control_set_angle(i, 90);
    }
}

void servo_control_set_angle(uint8_t servo_id, uint8_t angle) {
    if (servo_id >= 4) return;
    uint16_t pulse = 1041 + ((angle * (2083 - 1041)) / 180);
    servo_tpms[servo_id]->CONTROLS[servo_channels[servo_id]].CnV = pulse;
}
