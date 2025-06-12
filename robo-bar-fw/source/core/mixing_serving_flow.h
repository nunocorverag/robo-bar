#ifndef MIXING_SERVING_FLOW_H
#define MIXING_SERVING_FLOW_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Inicializa el relé del motor mezclador (PTE31).
 */
void RELAY_Init(void);

/**
 * Enciende el relé (mezclador activo).
 */
void RELAY_On(void);

/**
 * Apaga el relé (mezclador inactivo).
 */
void RELAY_Off(void);

/**
 * Retardo en milisegundos (bloqueante).
 */
void delay_ms(uint32_t ms);

/**
 * Activa el servo 5 (en PTD6) para servir la bebida.
 */
void ServirBebida(void);

#ifdef __cplusplus
}
#endif

#endif // MIXING_SERVING_FLOW_H
