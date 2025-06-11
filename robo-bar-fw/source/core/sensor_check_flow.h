/*
 * sensor_check_flow.h
 * 
 * Sensor Check and Mode Selection Flow Controller
 * FRDM-KL25Z Development Board
 */

#ifndef SENSOR_CHECK_FLOW_H
#define SENSOR_CHECK_FLOW_H

#include <stdint.h>
#include <stdbool.h>
#include "../config/system_config.h"
#include "../shared/system_types.h"

/*******************************************************************************
 * Sensor Check Flow States
 ******************************************************************************/

typedef enum {
    SENSOR_CHECK_STATE_INIT = 0,
    SENSOR_CHECK_STATE_CHECKING_SENSORS,
    SENSOR_CHECK_STATE_SENSOR_ERROR,
    SENSOR_CHECK_STATE_MODE_SELECTION,
    SENSOR_CHECK_STATE_REFILL_MODE,
    SENSOR_CHECK_STATE_WAITING_REFILL,
    SENSOR_CHECK_STATE_RECHECK_SENSORS,
    SENSOR_CHECK_STATE_READY,
    SENSOR_CHECK_STATE_ERROR,
    SENSOR_CHECK_STATE_COMPLETE
} sensor_check_flow_state_t;

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
 * Sensor Check Flow Control Structure
 ******************************************************************************/
typedef struct {
    sensor_check_flow_state_t current_state;
    operation_mode_t selected_mode;
    sensor_status_t sensor_status;
    bool hardware_initialized;  // Agregado para compatibilidad con startup_flow
} sensor_check_flow_t;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

/**
 * @brief Run sensor check flow controller (similar a StartupFlow_Run)
 * @return operation_status_t - OP_IN_PROGRESS, OP_COMPLETED, or OP_ERROR
 */
operation_status_t SensorCheckFlow_Run(void);

/**
 * @brief Initialize sensor check flow controller
 * @return true if successful, false otherwise
 */
bool SensorCheckFlow_Init(void);

/**
 * @brief Execute one step of sensor check flow (similar a StartupFlow_Execute)
 * @return operation_status_t - OP_IN_PROGRESS, OP_COMPLETED, or OP_ERROR
 */
operation_status_t SensorCheckFlow_Execute(void);

// Funciones de verificación y pantalla
bool SensorCheckFlow_CheckSensors(void);
void SensorCheckFlow_ShowSensorError(const sensor_status_t* sensor_status);
operation_mode_t SensorCheckFlow_SelectMode(void);
void SensorCheckFlow_ShowRefillInstructions(void);
void SensorCheckFlow_ShowReadyScreen(void);
void SensorCheckFlow_ShowError(const char* error_msg);

/**
 * @brief Get current sensor check flow state
 * @return Current state
 */
sensor_check_flow_state_t SensorCheckFlow_GetState(void);

/**
 * @brief Get current operation mode
 * @return Selected operation mode
 */
operation_mode_t SensorCheckFlow_GetMode(void);

/**
 * @brief Check if sensor check is complete
 * @return true if check complete, false otherwise
 */
bool SensorCheckFlow_IsComplete(void);

/**
 * @brief Get sensor status
 * @return Pointer to sensor status structure
 */
const sensor_status_t* SensorCheckFlow_GetSensorStatus(void);

/*******************************************************************************
 * Internal Functions (can be used for testing)
 ******************************************************************************/

/**
 * @brief Check all sensors and update status
 * @return true if all sensors OK, false if any sensor low
 */
bool SensorCheckFlow_CheckSensors(void);

/**
 * @brief Display sensor error screen
 * @param sensor_status Pointer to sensor status
 */
void SensorCheckFlow_ShowSensorError(const sensor_status_t* sensor_status);

/**
 * @brief Show mode selection screen and wait for user input
 * @return Selected operation mode
 */
operation_mode_t SensorCheckFlow_SelectMode(void);

/**
 * @brief Show refill instructions and wait for completion
 */
void SensorCheckFlow_ShowRefillInstructions(void);

/**
 * @brief Show ready screen and wait for start command
 */
void SensorCheckFlow_ShowReadyScreen(void);

/**
 * @brief Show error screen
 * @param error_msg Error message to display
 */
void SensorCheckFlow_ShowError(const char* error_msg);

/**
 * @brief Convert sensor check state to string
 * @param state Sensor check state
 * @return String representation of state
 */
const char* SensorCheckFlow_StateToString(sensor_check_flow_state_t state);

/**
 * @brief Reinicia el flujo de verificación de sensores a su estado inicial
 */
void SensorCheckFlow_Restart(void);

#endif /* SENSOR_CHECK_FLOW_H */