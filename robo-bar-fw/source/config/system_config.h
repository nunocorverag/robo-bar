/*
 * system_config.h
 *
 * System-wide configuration definitions for Robo-Bar project
 * FRDM-KL25Z Development Board
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
/* Liquid Dispensing System */
#define WATER_LEVEL_SENSORS_COUNT   3U           /* Number of level sensors - REDUCED TO 3 */
#define MAX_DISPENSE_TIME_MS        5000U        /* Maximum dispense time */

/* User Interface */
#define KEYPAD_ROWS                 4U           /* 4x4 matrix keypad */
#define KEYPAD_COLS                 4U

/* Motor Control */
#define MIXING_MOTOR_COUNT          1U

/* Liquid Dispensing System - Relay-based */
#define PUMP_RELAY_COUNT           3U           /* Number of ingredient pump relays */
#define DISPENSER_RELAY_COUNT      1U           /* Number of dispenser relays */
#define TOTAL_RELAY_COUNT          4U           /* Total relays (3 pumps + 1 dispenser) */
#define MAX_PUMP_TIME_MS           5000U        /* Maximum pump activation time */
#define WATER_LEVEL_SENSORS_COUNT  3U           /* Number of level sensors */

/*******************************************************************************
 * Debug and Monitoring Configuration
 ******************************************************************************/
#define DEBUG_UART_BAUDRATE         115200U      /* Debug UART baud rate */
#define DEBUG_BUFFER_SIZE           256U         /* Debug message buffer size */

#endif /* SYSTEM_CONFIG_H */