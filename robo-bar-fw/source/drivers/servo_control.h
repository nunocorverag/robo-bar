#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Alias legibles para los servos conectados a los pines específicos
#define SERVO_1 0  // PTD4 → TPM0_CH4
#define SERVO_2 1  // PTA12 → TPM1_CH0
#define SERVO_3 2  // PTA13 → TPM1_CH1
#define SERVO_4 3  // PTD5 → TPM0_CH5
#define SERVO_5 4  // PTD0 → TPM0_CH0

#define SERVO_COUNT 5

/**
 * Inicializa todos los servos (TPM, pines y PWM).
 * Configura TPM0 y TPM1 con 5 servos en los canales indicados.
 */
void servo_control_init(void);

/**
 * Establece el ángulo de un servo entre 0° y 180°.
 * @param servo_id ID del servo (0 a 4) o usa SERVO_X
 * @param angle Ángulo deseado (0 a 180 grados)
 */
void servo_control_set_angle(uint8_t servo_id, uint8_t angle);

#ifdef __cplusplus
}
#endif

#endif // SERVO_CONTROL_H
