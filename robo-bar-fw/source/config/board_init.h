#ifndef BOARD_INIT_H
#define BOARD_INIT_H

#include "MKL25Z4.h"
#include "fsl_clock.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_uart.h"

#include "gpio_config.h"
#include "system_config.h"
#include "pwm_config.h"

/* Declaraciones públicas de inicialización */
void BOARD_InitClocks(void);
void BOARD_InitGPIO(void);
void BOARD_InitPWM(void);
void BOARD_InitUART(void);
void BOARD_InitI2C(void);
void BOARD_InitAll(void);

#endif // BOARD_INIT_H
