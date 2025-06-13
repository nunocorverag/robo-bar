/*
 * mixing_serving_flow.h
 * 
 * Mixing and Serving Flow Controller
 * FRDM-KL25Z Development Board
 */

#ifndef MIXING_SERVING_FLOW_H
#define MIXING_SERVING_FLOW_H

#include <stdint.h>
#include <stdbool.h>
#include "../config/system_config.h"
#include "../shared/system_types.h"

/*******************************************************************************
 * Mixing and Serving Flow States
 ******************************************************************************/
typedef enum {
    MIXING_SERVING_STATE_INIT = 0,
    MIXING_SERVING_STATE_MIXING,
    MIXING_SERVING_STATE_SERVING,
    MIXING_SERVING_STATE_COMPLETE,
    MIXING_SERVING_STATE_ERROR
} mixing_serving_flow_state_t;

/*******************************************************************************
 * Mixing Configuration Structure
 ******************************************************************************/
typedef struct {
    uint16_t mixing_time_ms;     // Tiempo de mezclado en ms
    uint16_t serving_time_ms;    // Tiempo de servido en ms
    uint16_t pause_time_ms;      // Tiempo de pausa entre mezclado y servido
} mixing_config_t;

/*******************************************************************************
 * Mixing and Serving Flow Control Structure
 ******************************************************************************/
typedef struct {
    mixing_serving_flow_state_t current_state;
    mixing_config_t config;
    bool flow_initialized;
    uint32_t start_time;         // Para control de tiempo no bloqueante
} mixing_serving_flow_t;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

/**
 * @brief Run the mixing and serving flow - Main entry point
 * @return Operation status (OP_COMPLETED, OP_ERROR, OP_IN_PROGRESS)
 */
operation_status_t MixingServingFlow_Run(void);

/**
 * @brief Initialize mixing and serving flow controller
 * @return true if successful, false otherwise
 */
bool MixingServingFlow_Init(void);

/**
 * @brief Execute mixing and serving flow step by step
 * @return Operation status (OP_COMPLETED, OP_ERROR, OP_IN_PROGRESS)
 */
operation_status_t MixingServingFlow_Execute(void);

/**
 * @brief Set mixing configuration parameters
 * @param mixing_time_ms Time for mixing in milliseconds
 * @param serving_time_ms Time for serving in milliseconds
 * @param pause_time_ms Pause time between mixing and serving
 */
void MixingServingFlow_SetConfig(uint16_t mixing_time_ms, uint16_t serving_time_ms, uint16_t pause_time_ms);

/**
 * @brief Restart mixing and serving flow to initial state
 */
void MixingServingFlow_Restart(void);

/*******************************************************************************
 * Hardware Control Functions - Motor Relay (Mixing)
 ******************************************************************************/

/**
 * @brief Initialize motor relay for mixing (PTE31)
 */
void MOTOR_RELAY_Init(void);

/**
 * @brief Turn on motor relay (activate mixer)
 */
void MOTOR_RELAY_On(void);

/**
 * @brief Turn off motor relay (deactivate mixer)
 */
void MOTOR_RELAY_Off(void);

/*******************************************************************************
 * Hardware Control Functions - Dispenser Relay (Serving)
 ******************************************************************************/

/**
 * @brief Initialize dispenser relay for serving (PTE23)
 */
void DISPENSER_RELAY_Init(void);

/**
 * @brief Turn on dispenser relay (activate dispenser)
 */
void DISPENSER_RELAY_On(void);

/**
 * @brief Turn off dispenser relay (deactivate dispenser)
 */
void DISPENSER_RELAY_Off(void);

/*******************************************************************************
 * Legacy Hardware Control Functions (for backward compatibility)
 ******************************************************************************/

/**
 * @brief Initialize relay for mixing motor (PTE31) - Legacy function
 */
void RELAY_Init(void);

/**
 * @brief Turn on relay (activate mixer) - Legacy function
 */
void RELAY_On(void);

/**
 * @brief Turn off relay (deactivate mixer) - Legacy function
 */
void RELAY_Off(void);

/**
 * @brief Serve beverage using dispenser relay
 */
void ServirBebida(void);

/*******************************************************************************
 * Screen Display Functions
 ******************************************************************************/

/**
 * @brief Show mixing screen
 */
void MixingServingFlow_ShowMixingScreen(void);

/**
 * @brief Show serving screen
 */
void MixingServingFlow_ShowServingScreen(void);

/**
 * @brief Show completion screen
 */
void MixingServingFlow_ShowCompletionScreen(void);

/**
 * @brief Show error screen
 * @param error_msg Error message to display
 */
void MixingServingFlow_ShowError(const char* error_msg);

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

/**
 * @brief Convert mixing serving state to string
 * @param state Mixing serving state
 * @return String representation of state
 */
const char* MixingServingFlow_StateToString(mixing_serving_flow_state_t state);

/**
 * @brief Get current system time (simple counter)
 * @return Current time in milliseconds
 */
uint32_t MixingServingFlow_GetTime(void);

#endif /* MIXING_SERVING_FLOW_H */