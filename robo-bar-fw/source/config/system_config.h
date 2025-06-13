/*
 * system_config.h
 *
 * System-wide configuration definitions for Robo-Bar project
 * FRDM-KL25Z Development Board
 * LIMPIADO: Referencias a H-Bridge removidas
 */

#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * System Information
 ******************************************************************************/
#define SYSTEM_NAME                 "Robo-Bar System"
#define SYSTEM_VERSION_MAJOR        1
#define SYSTEM_VERSION_MINOR        0
#define SYSTEM_VERSION_PATCH        0

/*******************************************************************************
 * Hardware Configuration
 ******************************************************************************/
/* FRDM-KL25Z Specifications */
#define SYSTEM_CORE_CLOCK_HZ        48000000U    /* 48 MHz */
#define SYSTEM_BUS_CLOCK_HZ         24000000U    /* 24 MHz */
#define SYSTEM_FLASH_SIZE_KB        128U         /* 128 KB */
#define SYSTEM_RAM_SIZE_KB          16U          /* 16 KB */

/*******************************************************************************
 * System Components Configuration
 ******************************************************************************/
/* Liquid Systems */
#define WATER_LEVEL_SENSORS_COUNT   3U           /* Number of level sensors - REDUCED TO 3 */
#define MAX_DISPENSE_TIME_MS        5000U        /* Maximum dispense time */

/* User Interface */
#define KEYPAD_ROWS                 4U           /* 4x4 matrix keypad */
#define KEYPAD_COLS                 4U

/* Relay Control System */
#define PUMP_RELAY_COUNT           3U           /* Number of ingredient pump relays */
#define DISPENSER_RELAY_COUNT      1U           /* Number of dispenser relays */
#define MOTOR_RELAY_COUNT          1U           /* Number of motor relays (mixing) */
#define TOTAL_RELAY_COUNT          5U           /* Total relays (3 pumps + 1 dispenser + 1 motor) */
#define MAX_PUMP_TIME_MS           5000U        /* Maximum pump activation time */

/*******************************************************************************
 * Debug and Monitoring Configuration
 ******************************************************************************/
#define DEBUG_UART_BAUDRATE         115200U      /* Debug UART baud rate */
#define DEBUG_BUFFER_SIZE           256U         /* Debug message buffer size */

/*******************************************************************************
 * Flow Control Configuration
 ******************************************************************************/
#define DEFAULT_MIXING_TIME_MS      5000U        /* Default mixing time */
#define DEFAULT_SERVING_TIME_MS     5000U        /* Default serving time */
#define DEFAULT_PAUSE_TIME_MS       1000U        /* Default pause time */

#endif /* SYSTEM_CONFIG_H */