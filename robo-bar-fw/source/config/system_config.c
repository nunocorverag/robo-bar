/*
 * system_config.c
 * 
 * System configuration implementation for Robo-Bar project
 * FRDM-KL25Z Development Board
 */

#include "system_config.h"
#include <stdio.h>
#include <string.h>

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
volatile system_state_t g_system_state = SYSTEM_STATE_INIT;
volatile system_error_t g_last_error = SYSTEM_ERROR_NONE;
volatile uint32_t g_system_uptime_seconds = 0;

/*******************************************************************************
 * Private Variables
 ******************************************************************************/
static char version_string[32];

/*******************************************************************************
 * String lookup tables
 ******************************************************************************/
static const char* system_state_strings[] = {
    "INIT",
    "IDLE",
    "PREPARING",
    "DISPENSING",
    "MIXING",
    "SERVING",
    "CLEANING",
    "ERROR",
    "MAINTENANCE"
};

static const char* system_error_strings[] = {
    "NONE",
    "HARDWARE_INIT",
    "SENSOR_FAILURE",
    "SERVO_TIMEOUT",
    "EMPTY_CONTAINER",
    "COMMUNICATION",
    "MEMORY",
    "UNKNOWN"
};

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/*!
 * @brief Initialize system configuration
 */
void system_config_init(void)
{
    /* Initialize global variables */
    g_system_state = SYSTEM_STATE_INIT;
    g_last_error = SYSTEM_ERROR_NONE;
    g_system_uptime_seconds = 0;
    
    /* Build version string */
    snprintf(version_string, sizeof(version_string), 
             "v%d.%d.%d", 
             SYSTEM_VERSION_MAJOR, 
             SYSTEM_VERSION_MINOR, 
             SYSTEM_VERSION_PATCH);
}

/*!
 * @brief Get system version string
 * @return Pointer to version string
 */
const char* system_get_version_string(void)
{
    return version_string;
}

/*!
 * @brief Convert system state enum to string
 * @param state System state to convert
 * @return Pointer to state string
 */
const char* system_state_to_string(system_state_t state)
{
    if (state < ARRAY_SIZE(system_state_strings))
    {
        return system_state_strings[state];
    }
    return "INVALID";
}

/*!
 * @brief Convert system error enum to string
 * @param error System error to convert
 * @return Pointer to error string
 */
const char* system_error_to_string(system_error_t error)
{
    if (error < ARRAY_SIZE(system_error_strings))
    {
        return system_error_strings[error];
    }
    return "INVALID";
}

/*!
 * @brief Check if a feature is enabled
 * @param feature_flag Feature flag to check
 * @return true if feature is enabled, false otherwise
 */
bool system_is_feature_enabled(uint32_t feature_flag)
{
    /* This could be extended to support runtime feature toggling */
    return (feature_flag != 0);
}