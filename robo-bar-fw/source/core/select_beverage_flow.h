/*
 * select_beverage_flow.h
 * 
 * Beverage Selection and Dispensing Flow Controller
 * FRDM-KL25Z Development Board
 */

#ifndef SELECT_BEVERAGE_FLOW_H
#define SELECT_BEVERAGE_FLOW_H

#include <stdint.h>
#include <stdbool.h>
#include "../config/system_config.h"

/*******************************************************************************
 * Beverage Selection Flow States
 ******************************************************************************/
typedef enum {
    BEVERAGE_SELECT_STATE_INIT = 0,
    BEVERAGE_SELECT_STATE_SELECTION,
    BEVERAGE_SELECT_STATE_DISPENSING,
    BEVERAGE_SELECT_STATE_DISPENSING_COMPLETE,
    BEVERAGE_SELECT_STATE_ERROR,
    BEVERAGE_SELECT_STATE_COMPLETE
} beverage_select_flow_state_t;

/*******************************************************************************
 * Beverage Types
 ******************************************************************************/
typedef enum {
    BEVERAGE_MOJITO = 1,
    BEVERAGE_MARGARITA = 2,
    BEVERAGE_CUBA_LIBRE = 3,
    BEVERAGE_VODKA_TONIC = 4
} beverage_type_t;

/*******************************************************************************
 * Beverage Recipe Structure
 ******************************************************************************/
typedef struct {
    const char* nombre;
    uint8_t servos[4];       // Lista de servos activos (255 = inactivo)
    uint16_t tiempo_ms;      // Tiempo de apertura de válvulas
} receta_bebida_t;

/*******************************************************************************
 * Beverage Selection Flow Control Structure
 ******************************************************************************/
typedef struct {
    beverage_select_flow_state_t current_state;
    beverage_select_flow_state_t previous_state;
    beverage_type_t selected_beverage;
    const receta_bebida_t* current_recipe;
    uint32_t state_enter_time;
    uint32_t total_flow_time;
    uint32_t dispensing_start_time;
    bool flow_complete;
    bool error_occurred;
} beverage_select_flow_t;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

/**
 * @brief Initialize beverage selection flow controller
 * @return true if successful, false otherwise
 */
bool BeverageSelectFlow_Init(void);

/**
 * @brief Execute main beverage selection and dispensing flow (blocking)
 * This function will run the complete beverage selection and dispensing sequence
 * @return true if flow successful, false if error
 */
bool BeverageSelectFlow_Execute(void);

/**
 * @brief Run the entire beverage selection and dispensing process
 * This is the main function to start the beverage flow process
 */
void BeverageSelectFlow_Run(void);

/**
 * @brief Get current beverage selection flow state
 * @return Current state
 */
beverage_select_flow_state_t BeverageSelectFlow_GetState(void);

/**
 * @brief Get selected beverage type
 * @return Selected beverage type
 */
beverage_type_t BeverageSelectFlow_GetSelectedBeverage(void);

/**
 * @brief Check if beverage selection flow is complete
 * @return true if flow complete, false otherwise
 */
bool BeverageSelectFlow_IsComplete(void);

/**
 * @brief Force restart of beverage selection flow
 */
void BeverageSelectFlow_Restart(void);

/*******************************************************************************
 * Internal Functions (can be used for testing)
 ******************************************************************************/

/**
 * @brief Show beverage selection screen and wait for user input
 * @return Selected beverage type
 */
beverage_type_t BeverageSelectFlow_SelectBeverage(void);

/**
 * @brief Execute beverage dispensing sequence
 * @param recipe Pointer to beverage recipe
 * @return true if dispensing successful, false if error
 */
bool BeverageSelectFlow_DispenseBeverage(const receta_bebida_t* recipe);

/**
 * @brief Show dispensing screen with beverage name
 * @param beverage_name Name of beverage being dispensed
 */
void BeverageSelectFlow_ShowDispensingScreen(const char* beverage_name);

/**
 * @brief Show completion screen
 * @param beverage_name Name of completed beverage
 */
void BeverageSelectFlow_ShowCompletionScreen(const char* beverage_name);

/**
 * @brief Show error screen
 * @param error_msg Error message to display
 */
void BeverageSelectFlow_ShowError(const char* error_msg);

/**
 * @brief Convert beverage selection state to string
 * @param state Beverage selection state
 * @return String representation of state
 */
const char* BeverageSelectFlow_StateToString(beverage_select_flow_state_t state);

/**
 * @brief Get recipe for beverage type
 * @param beverage_type Type of beverage
 * @return Pointer to recipe, or NULL if invalid type
 */
const receta_bebida_t* BeverageSelectFlow_GetRecipe(beverage_type_t beverage_type);

#endif /* SELECT_BEVERAGE_FLOW_H */
