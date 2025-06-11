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
#define SERVO_COUNT                 4U           /* Number of servo motors */
#define WATER_LEVEL_SENSORS_COUNT   4U           /* Number of level sensors */
#define MAX_DISPENSE_TIME_MS        5000U        /* Maximum dispense time */

/* User Interface */
#define KEYPAD_ROWS                 4U           /* 4x4 matrix keypad */
#define KEYPAD_COLS                 4U

/* Motor Control */
#define MIXING_MOTOR_COUNT          1U
#define DIRECTION_SERVO_COUNT       1U           /* 360° servo */

/* Timing Configuration */
#define SERVO_PWM_FREQUENCY_HZ      50U          /* Standard servo PWM frequency */
#define SERVO_PWM_PERIOD_US         20000U       /* 20ms period */
#define SERVO_PULSE_MIN_US          1000U        /* 1ms minimum pulse */
#define SERVO_PULSE_MAX_US          2000U        /* 2ms maximum pulse */
#define SERVO_PULSE_NEUTRAL_US      1500U        /* 1.5ms neutral pulse */

/*******************************************************************************
 * Debug and Monitoring Configuration
 ******************************************************************************/
#define DEBUG_UART_BAUDRATE         115200U      /* Debug UART baud rate */
#define DEBUG_BUFFER_SIZE           256U         /* Debug message buffer size */

#endif /* SYSTEM_CONFIG_H */