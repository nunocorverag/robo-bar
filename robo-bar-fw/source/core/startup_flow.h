/*
 * startup_flow.h
 * 
 * System Startup Flow Controller
 * FRDM-KL25Z Development Board
 */

#ifndef STARTUP_FLOW_H
#define STARTUP_FLOW_H

#include <stdint.h>
#include <stdbool.h>
#include "../config/system_config.h"

/*******************************************************************************
 * Startup Flow States
 ******************************************************************************/
typedef enum {
    STARTUP_STATE_INIT = 0,
    STARTUP_STATE_INITIALIZING,
    STARTUP_STATE_CHECKING_SENSORS,
    STARTUP_STATE_SENSOR_ERROR,
    STARTUP_STATE_MODE_SELECTION,
    STARTUP_STATE_REFILL_MODE,
    STARTUP_STATE_WAITING_REFILL,
    STARTUP_STATE_RECHECK_SENSORS,
    STARTUP_STATE_READY,
    STARTUP_STATE_ERROR,
    STARTUP_STATE_COMPLETE
} startup_flow_state_t;

/*******************************************************************************
 * Operation Modes
 ******************************************************************************/
typedef enum {
    OPERATION_MODE_NORMAL = 1,
    OPERATION_MODE_REFILL = 2
} operation_mode_t;

/*******************************************************************************
 * Sensor Status Structure
 ******************************************************************************/
typedef struct {
    bool sensor_1_ok;      /* Liquid 1 level OK */
    bool sensor_2_ok;      /* Liquid 2 level OK */
    bool sensor_3_ok;      /* Liquid 3 level OK */
    bool sensor_4_ok;      /* Liquid 4 level OK */
    uint8_t low_sensors;   /* Bitmask of low sensors */
    bool can_operate;      /* true if system can operate */
} sensor_status_t;

/*******************************************************************************
 * Startup Flow Control Structure
 ******************************************************************************/
typedef struct {
    startup_flow_state_t current_state;
    startup_flow_state_t previous_state;
    operation_mode_t selected_mode;
    sensor_status_t sensor_status;
    uint32_t state_enter_time;
    uint32_t total_startup_time;
    bool startup_complete;
    bool error_occurred;
} startup_flow_t;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

/**
 * @brief Initialize startup flow controller
 * @return true if successful, false otherwise
 */
bool StartupFlow_Init(void);

/**
 * @brief Execute main startup flow (blocking)
 * This function will run the complete startup sequence
 * @return true if startup successful, false if error
 */
bool StartupFlow_Execute(void);


/**
 * @brief Run startup flow as FreeRTOS task
 * @param pvParameters Task parameters (unused)
 */
void StartupFlow_Task(void *pvParameters);

/**
 * @brief Get current startup flow state
 * @return Current state
 */
startup_flow_state_t StartupFlow_GetState(void);

/**
 * @brief Get current operation mode
 * @return Selected operation mode
 */
operation_mode_t StartupFlow_GetMode(void);

/**
 * @brief Check if startup is complete
 * @return true if startup complete, false otherwise
 */
bool StartupFlow_IsComplete(void);

/**
 * @brief Get sensor status
 * @return Pointer to sensor status structure
 */
const sensor_status_t* StartupFlow_GetSensorStatus(void);

/**
 * @brief Force restart of startup flow
 */
void StartupFlow_Restart(void);

/*******************************************************************************
 * Internal Functions (can be used for testing)
 ******************************************************************************/

/**
 * @brief Show initialization screen
 */
void StartupFlow_ShowInitScreen(void);

/**
 * @brief Check all sensors and update status
 * @return true if all sensors OK, false if any sensor low
 */
bool StartupFlow_CheckSensors(void);

/**
 * @brief Display sensor error screen
 * @param sensor_status Pointer to sensor status
 */
void StartupFlow_ShowSensorError(const sensor_status_t* sensor_status);

/**
 * @brief Show mode selection screen and wait for user input
 * @return Selected operation mode
 */
operation_mode_t StartupFlow_SelectMode(void);

/**
 * @brief Show refill instructions and wait for completion
 */
void StartupFlow_ShowRefillInstructions(void);

/**
 * @brief Show ready screen and wait for start command
 */
void StartupFlow_ShowReadyScreen(void);

/**
 * @brief Show error screen
 * @param error_msg Error message to display
 */
void StartupFlow_ShowError(const char* error_msg);

/**
 * @brief Convert startup state to string
 * @param state Startup state
 * @return String representation of state
 */
const char* StartupFlow_StateToString(startup_flow_state_t state);

#endif /* STARTUP_FLOW_H */
