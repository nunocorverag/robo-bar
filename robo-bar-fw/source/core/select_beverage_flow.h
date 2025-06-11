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
#include "../shared/system_types.h"

/*******************************************************************************
 * Beverage Selection Flow States
 ******************************************************************************/
typedef enum {
    BEVERAGE_SELECT_STATE_INIT = 0,
    BEVERAGE_SELECT_STATE_SELECTION,
    BEVERAGE_SELECT_STATE_DISPENSING,
    BEVERAGE_SELECT_STATE_DISPENSING_COMPLETE,
    BEVERAGE_SELECT_STATE_COMPLETE,
    BEVERAGE_SELECT_STATE_ERROR
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
    beverage_type_t selected_beverage;
    const receta_bebida_t* current_recipe;
    bool flow_initialized;
} beverage_select_flow_t;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

/**
 * @brief Run the beverage selection flow - Main entry point
 * @return Operation status (OP_COMPLETED, OP_ERROR, OP_IN_PROGRESS)
 */
operation_status_t BeverageSelectFlow_Run(void);

/**
 * @brief Initialize beverage selection flow controller
 * @return true if successful, false otherwise
 */
bool BeverageSelectFlow_Init(void);

/**
 * @brief Execute beverage selection flow step by step
 * @return Operation status (OP_COMPLETED, OP_ERROR, OP_IN_PROGRESS)
 */
operation_status_t BeverageSelectFlow_Execute(void);

/*******************************************************************************
 * Screen Display Functions
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

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

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

/**
 * @brief Reinicia el flujo de selección de bebidas a su estado inicial
 */
void BeverageSelectFlow_Restart(void);

#endif /* SELECT_BEVERAGE_FLOW_H */