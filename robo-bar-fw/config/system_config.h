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
#define LCD_ROWS                    2U           /* 16x2 LCD display */
#define LCD_COLS                    16U

/* Motor Control */
#define CONVEYOR_MOTOR_COUNT        1U           /* H-bridge motors */
#define MIXING_MOTOR_COUNT          1U
#define DIRECTION_SERVO_COUNT       1U           /* 360° servo */

/* Timing Configuration */
#define SERVO_PWM_FREQUENCY_HZ      50U          /* Standard servo PWM frequency */
#define SERVO_PWM_PERIOD_US         20000U       /* 20ms period */
#define SERVO_PULSE_MIN_US          1000U        /* 1ms minimum pulse */
#define SERVO_PULSE_MAX_US          2000U        /* 2ms maximum pulse */
#define SERVO_PULSE_NEUTRAL_US      1500U        /* 1.5ms neutral pulse */

/* System Limits */
#define MAX_RECIPE_INGREDIENTS      6U           /* Maximum ingredients per recipe */
#define MAX_RECIPE_NAME_LENGTH      20U          /* Maximum recipe name length */
#define MAX_QUEUE_SIZE              10U          /* Maximum order queue size */

/*******************************************************************************
 * Debug and Monitoring Configuration
 ******************************************************************************/
#define DEBUG_UART_BAUDRATE         115200U      /* Debug UART baud rate */
#define DEBUG_BUFFER_SIZE           256U         /* Debug message buffer size */

/* System monitoring intervals */
#define SYSTEM_MONITOR_INTERVAL_MS  2000U        /* System health check interval */
#define DEBUG_PRINT_INTERVAL_MS     5000U        /* Debug message interval */
#define LED_BLINK_INTERVAL_MS       1000U        /* LED status blink interval */

/* Error handling */
#define MAX_ERROR_COUNT             5U           /* Maximum consecutive errors */
#define ERROR_RECOVERY_DELAY_MS     1000U        /* Delay before error recovery */

/*******************************************************************************
 * Safety Configuration
 ******************************************************************************/
#define SAFETY_TIMEOUT_MS           10000U       /* General safety timeout */
#define EMERGENCY_STOP_TIMEOUT_MS   500U         /* Emergency stop response time */
#define SENSOR_DEBOUNCE_MS          50U          /* Sensor debounce time */

/* Operational limits */
#define MAX_CONTINUOUS_RUN_TIME_MS  30000U       /* Maximum continuous operation */
#define COOL_DOWN_TIME_MS           5000U        /* Cool down period */

/*******************************************************************************
 * Memory Configuration
 ******************************************************************************/
#define RECIPE_STORAGE_SIZE         512U         /* Bytes for recipe storage */
#define LOG_BUFFER_SIZE             1024U        /* Bytes for system logs */
#define TEMP_BUFFER_SIZE            128U         /* Temporary buffer size */

/*******************************************************************************
 * System States
 ******************************************************************************/
typedef enum {
    SYSTEM_STATE_INIT = 0,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_PREPARING,
    SYSTEM_STATE_DISPENSING,
    SYSTEM_STATE_MIXING,
    SYSTEM_STATE_SERVING,
    SYSTEM_STATE_CLEANING,
    SYSTEM_STATE_ERROR,
    SYSTEM_STATE_MAINTENANCE
} system_state_t;

/*******************************************************************************
 * Error Codes
 ******************************************************************************/
typedef enum {
    SYSTEM_ERROR_NONE = 0,
    SYSTEM_ERROR_HARDWARE_INIT,
    SYSTEM_ERROR_SENSOR_FAILURE,
    SYSTEM_ERROR_SERVO_TIMEOUT,
    SYSTEM_ERROR_EMPTY_CONTAINER,
    SYSTEM_ERROR_COMMUNICATION,
    SYSTEM_ERROR_MEMORY,
    SYSTEM_ERROR_UNKNOWN
} system_error_t;

/*******************************************************************************
 * Feature Flags
 ******************************************************************************/
/* Enable/disable system features */
#define FEATURE_LCD_DISPLAY         1            /* Enable LCD display */
#define FEATURE_KEYPAD_INPUT        1            /* Enable keypad input */
#define FEATURE_SERVO_CONTROL       1            /* Enable servo control */
#define FEATURE_SENSOR_MONITORING   1            /* Enable sensor monitoring */
#define FEATURE_UART_DEBUG          1            /* Enable UART debugging */
#define FEATURE_ERROR_RECOVERY      1            /* Enable error recovery */
#define FEATURE_SYSTEM_MONITORING   1            /* Enable system monitoring */

/*******************************************************************************
 * Macros
 ******************************************************************************/
#define ARRAY_SIZE(x)               (sizeof(x) / sizeof((x)[0]))
#define MIN(a, b)                   ((a) < (b) ? (a) : (b))
#define MAX(a, b)                   ((a) > (b) ? (a) : (b))
#define CLAMP(val, min, max)        (MIN(MAX(val, min), max))

/* Convert milliseconds to FreeRTOS ticks */
#define MS_TO_TICKS(ms)             ((ms) / portTICK_PERIOD_MS)

/* Status check macros */
#define IS_VALID_SERVO_ID(id)       ((id) < SERVO_COUNT)
#define IS_VALID_SENSOR_ID(id)      ((id) < WATER_LEVEL_SENSORS_COUNT)

/*******************************************************************************
 * Global Variables (extern declarations)
 ******************************************************************************/
extern volatile system_state_t g_system_state;
extern volatile system_error_t g_last_error;
extern volatile uint32_t g_system_uptime_seconds;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
void system_config_init(void);
const char* system_get_version_string(void);
const char* system_state_to_string(system_state_t state);
const char* system_error_to_string(system_error_t error);
bool system_is_feature_enabled(uint32_t feature_flag);

#endif /* SYSTEM_CONFIG_H */