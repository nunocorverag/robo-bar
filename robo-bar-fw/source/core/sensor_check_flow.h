/*
 * sensor_check_flow.h
 *
 * Sensor Check and Mode Selection Flow Controller Header (MODIFIED for 3 sensors)
 * FRDM-KL25Z Development Board
 */
#ifndef SENSOR_CHECK_FLOW_H
#define SENSOR_CHECK_FLOW_H

#include <stdint.h>
#include <stdbool.h>
#include "../config/system_config.h"
#include "../shared/system_types.h"

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/

/* Operation mode enumeration */
typedef enum {
    OPERATION_MODE_NORMAL = 0,    /* Normal cocktail operation */
    OPERATION_MODE_REFILL         /* Refill/maintenance mode */
} operation_mode_t;

/* Sensor check flow states */
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

/* Sensor status structure (modified for 3 sensors) */
typedef struct {
    bool sensor_1_ok;         /* Sensor 1 status */
    bool sensor_2_ok;         /* Sensor 2 status */
    bool sensor_3_ok;         /* Sensor 3 status */
    uint8_t low_sensors;      /* Bitmask of low sensors (bits 0-2 used) */
    bool can_operate;         /* True if system can operate with current sensor status */
} sensor_status_t;

/* Main sensor check flow structure */
typedef struct {
    sensor_check_flow_state_t current_state;
    operation_mode_t selected_mode;
    sensor_status_t sensor_status;
    bool hardware_initialized;
} sensor_check_flow_t;

/*******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

/* Main flow control functions */
operation_status_t SensorCheckFlow_Run(void);
bool SensorCheckFlow_Init(void);
operation_status_t SensorCheckFlow_Execute(void);
void SensorCheckFlow_Restart(void);

/* Sensor checking and display functions */
bool SensorCheckFlow_CheckSensors(void);
void SensorCheckFlow_ShowSensorError(const sensor_status_t* sensor_status);
operation_mode_t SensorCheckFlow_SelectMode(void);
void SensorCheckFlow_ShowRefillInstructions(void);
void SensorCheckFlow_ShowReadyScreen(void);
void SensorCheckFlow_ShowError(const char* error_msg);

/* Getter functions */
sensor_check_flow_state_t SensorCheckFlow_GetState(void);
operation_mode_t SensorCheckFlow_GetMode(void);
bool SensorCheckFlow_IsComplete(void);
const sensor_status_t* SensorCheckFlow_GetSensorStatus(void);

/* Helper functions */
const char* SensorCheckFlow_StateToString(sensor_check_flow_state_t state);

/*******************************************************************************
 * Constants
 ******************************************************************************/
#define SENSOR_COUNT                3U      /* Total number of sensors */
#define MIN_SENSORS_TO_OPERATE      1U      /* Minimum sensors needed to operate - MODIFICADO */

#endif /* SENSOR_CHECK_FLOW_H */