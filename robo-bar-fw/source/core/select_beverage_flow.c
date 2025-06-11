/*
 * select_beverage_flow.c
 * 
 * Beverage Selection and Dispensing Flow Controller Implementation
 * FRDM-KL25Z Development Board
 */

#include "select_beverage_flow.h"
#include "../drivers/lcd_i2c.h"
#include "../drivers/keypad.h"
#include "../drivers/servo_control.h"
#include "../config/gpio_config.h"
#include <string.h>
#include <stdio.h>

/*******************************************************************************
 * Private Variables
 ******************************************************************************/
static beverage_select_flow_t g_beverage_select_flow;

// Definición de recetas de bebidas
static const receta_bebida_t recetas[] = {
    { "Mojito",      {0, 1, 255, 255}, 1500 },
    { "Margarita",   {2, 0, 255, 255}, 1200 },
    { "Cuba Libre",  {3, 1, 2, 255},   1800 },
    { "Vodka Tonic", {0, 2, 3, 1},     2000 },
};

#define NUM_BEBIDAS (sizeof(recetas)/sizeof(recetas[0]))

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/
static void delay_ms(uint32_t ms);

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

void BeverageSelectFlow_Run(void) {
    if (!BeverageSelectFlow_Init()) {
        Debug_Printf("BeverageSelectFlow: Initialization failed\r\n");
        LED_SetColor(true, false, false);  // Red error
        return;
    }

    if (BeverageSelectFlow_Execute()) {
        Debug_Printf("BeverageSelectFlow: Completed successfully\r\n");
        LED_SetColor(false, true, false);  // Green success
    } else {
        Debug_Printf("BeverageSelectFlow: Execution failed\r\n");
        LED_SetColor(true, false, false);  // Red error
    }
}

/**
 * @brief Initialize beverage selection flow controller
 * @return true if successful, false otherwise
 */
bool BeverageSelectFlow_Init(void) {
    Debug_Printf("BeverageSelectFlow: Initializing...\r\n");
    
    // Initialize structure
    memset(&g_beverage_select_flow, 0, sizeof(beverage_select_flow_t));
    g_beverage_select_flow.current_state = BEVERAGE_SELECT_STATE_INIT;
    g_beverage_select_flow.previous_state = BEVERAGE_SELECT_STATE_INIT;
    
    // Initialize servo control
    servo_control_init();
    
    Debug_Printf("BeverageSelectFlow: Initialization complete\r\n");
    return true;
}

/**
 * @brief Execute main beverage selection and dispensing flow (blocking)
 * This function will run the complete beverage selection and dispensing sequence
 * @return true if flow successful, false if error
 */
bool BeverageSelectFlow_Execute(void) {
    Debug_Printf("BeverageSelectFlow: Starting execution\r\n");
    
    
    while (!g_beverage_select_flow.flow_complete && !g_beverage_select_flow.error_occurred) {
        
        // Update state entry time on state change
        if (g_beverage_select_flow.current_state != g_beverage_select_flow.previous_state) {
            g_beverage_select_flow.previous_state = g_beverage_select_flow.current_state;
            Debug_Printf("BeverageSelectFlow: State changed to %s\r\n", BeverageSelectFlow_StateToString(g_beverage_select_flow.current_state));
        }
        
        switch (g_beverage_select_flow.current_state) {
            case BEVERAGE_SELECT_STATE_INIT:
                g_beverage_select_flow.current_state = BEVERAGE_SELECT_STATE_SELECTION;
                break;
                
            case BEVERAGE_SELECT_STATE_SELECTION:
                g_beverage_select_flow.selected_beverage = BeverageSelectFlow_SelectBeverage();
                if (g_beverage_select_flow.selected_beverage >= BEVERAGE_MOJITO && 
                    g_beverage_select_flow.selected_beverage <= BEVERAGE_VODKA_TONIC) {
                    
                    // Get recipe for selected beverage
                    g_beverage_select_flow.current_recipe = BeverageSelectFlow_GetRecipe(g_beverage_select_flow.selected_beverage);
                    if (g_beverage_select_flow.current_recipe != NULL) {
                        g_beverage_select_flow.current_state = BEVERAGE_SELECT_STATE_DISPENSING;
                    } else {
                        g_beverage_select_flow.current_state = BEVERAGE_SELECT_STATE_ERROR;
                    }
                } else {
                    // Invalid selection, stay in selection state
                    continue;
                }
                break;
                
            case BEVERAGE_SELECT_STATE_DISPENSING:
                BeverageSelectFlow_ShowDispensingScreen(g_beverage_select_flow.current_recipe->nombre);
                
                if (BeverageSelectFlow_DispenseBeverage(g_beverage_select_flow.current_recipe)) {
                    g_beverage_select_flow.current_state = BEVERAGE_SELECT_STATE_DISPENSING_COMPLETE;
                } else {
                    g_beverage_select_flow.current_state = BEVERAGE_SELECT_STATE_ERROR;
                }
                break;
                
            case BEVERAGE_SELECT_STATE_DISPENSING_COMPLETE:
                BeverageSelectFlow_ShowCompletionScreen(g_beverage_select_flow.current_recipe->nombre);
                delay_ms(2000);  // Show completion message for 2 seconds
                g_beverage_select_flow.current_state = BEVERAGE_SELECT_STATE_COMPLETE;
                break;
                
            case BEVERAGE_SELECT_STATE_COMPLETE:
                g_beverage_select_flow.flow_complete = true;
                break;
                
            case BEVERAGE_SELECT_STATE_ERROR:
            default:
                g_beverage_select_flow.error_occurred = true;
                break;
        }
        
        // Small delay to prevent tight loop
        delay_ms(10);
    }
        
    if (g_beverage_select_flow.flow_complete) {
        Debug_Printf("BeverageSelectFlow: Completed successfully in %lu ms\r\n", g_beverage_select_flow.total_flow_time);
        return true;
    } else {
        Debug_Printf("BeverageSelectFlow: Failed with error\r\n");
        return false;
    }
}

/*******************************************************************************
 * Screen Display Functions
 ******************************************************************************/

beverage_type_t BeverageSelectFlow_SelectBeverage(void) {
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Selecciona 1-4:");
    lcd_set_cursor(1, 0);
    lcd_print("Bebida deseada");

    Debug_Printf("BeverageSelectFlow: Waiting for beverage selection...\r\n");

    char key = '\0';
    while (key < '1' || key > '4') {
        key = keypad_getkey();  // Espera una tecla
        Debug_Printf("BeverageSelectFlow: Key pressed: %c\r\n", key);

        if (key >= '1' && key <= '4') {
            beverage_type_t selected = (beverage_type_t)(key - '0');
            Debug_Printf("BeverageSelectFlow: Beverage %d selected\r\n", selected);
            return selected;
        }
    }

    return BEVERAGE_MOJITO;  // Fallback por defecto
}

bool BeverageSelectFlow_DispenseBeverage(const receta_bebida_t* recipe) {
    if (recipe == NULL) {
        Debug_Printf("BeverageSelectFlow: NULL recipe provided\r\n");
        return false;
    }
    
    Debug_Printf("BeverageSelectFlow: Dispensing %s for %d ms\r\n", recipe->nombre, recipe->tiempo_ms);
    
    // Abrir válvulas (servos) según la receta
    for (int i = 0; i < 4; i++) {
        if (recipe->servos[i] < 4) {  // 255 indica servo inactivo
            Debug_Printf("BeverageSelectFlow: Opening servo %d\r\n", recipe->servos[i]);
            servo_control_set_angle(recipe->servos[i], 90);  // Abrir válvula
        }
    }
    
    // Esperar el tiempo especificado para dispensar
    delay_ms(recipe->tiempo_ms);
    
    // Cerrar todas las válvulas
    for (int i = 0; i < 4; i++) {
        if (recipe->servos[i] < 4) {
            Debug_Printf("BeverageSelectFlow: Closing servo %d\r\n", recipe->servos[i]);
            servo_control_set_angle(recipe->servos[i], 0);   // Cerrar válvula
        }
    }
    
    Debug_Printf("BeverageSelectFlow: Dispensing complete\r\n");
    return true;
}

void BeverageSelectFlow_ShowDispensingScreen(const char* beverage_name) {
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Dispensando");
    lcd_set_cursor(1, 0);
    lcd_print("liquidos...");
    
    Debug_Printf("BeverageSelectFlow: Dispensing screen shown for %s\r\n", beverage_name);
}

void BeverageSelectFlow_ShowCompletionScreen(const char* beverage_name) {
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Bebida lista!");
    lcd_set_cursor(1, 0);
    lcd_print(beverage_name);
    
    Debug_Printf("BeverageSelectFlow: Completion screen shown for %s\r\n", beverage_name);
}

void BeverageSelectFlow_ShowError(const char* error_msg) {
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("ERROR:");
    lcd_set_cursor(1, 0);
    lcd_print(error_msg);

    Debug_Printf("BeverageSelectFlow: Error displayed: %s\r\n", error_msg);

    // Flash red LED
    for (int i = 0; i < 10; i++) {
        LED_SetColor(true, false, false);   // Red
        delay_ms(200);
        LED_SetColor(false, false, false);  // Off
        delay_ms(200);
    }
}

/*******************************************************************************
 * Getter Functions
 ******************************************************************************/

beverage_select_flow_state_t BeverageSelectFlow_GetState(void) {
    return g_beverage_select_flow.current_state;
}

beverage_type_t BeverageSelectFlow_GetSelectedBeverage(void) {
    return g_beverage_select_flow.selected_beverage;
}

bool BeverageSelectFlow_IsComplete(void) {
    return g_beverage_select_flow.flow_complete;
}

void BeverageSelectFlow_Restart(void) {
    Debug_Printf("BeverageSelectFlow: Restarting...\r\n");
    memset(&g_beverage_select_flow, 0, sizeof(beverage_select_flow_t));
    g_beverage_select_flow.current_state = BEVERAGE_SELECT_STATE_INIT;
}

const char* BeverageSelectFlow_StateToString(beverage_select_flow_state_t state) {
    switch (state) {
        case BEVERAGE_SELECT_STATE_INIT: return "INIT";
        case BEVERAGE_SELECT_STATE_SELECTION: return "SELECTION";
        case BEVERAGE_SELECT_STATE_DISPENSING: return "DISPENSING";
        case BEVERAGE_SELECT_STATE_DISPENSING_COMPLETE: return "DISPENSING_COMPLETE";
        case BEVERAGE_SELECT_STATE_ERROR: return "ERROR";
        case BEVERAGE_SELECT_STATE_COMPLETE: return "COMPLETE";
        default: return "UNKNOWN";
    }
}

const receta_bebida_t* BeverageSelectFlow_GetRecipe(beverage_type_t beverage_type) {
    if (beverage_type >= BEVERAGE_MOJITO && beverage_type <= BEVERAGE_VODKA_TONIC) {
        return &recetas[beverage_type - 1];  // Convert 1-based to 0-based index
    }
    return NULL;
}

/*******************************************************************************
 * Private Helper Functions
 ******************************************************************************/

void delay_ms(uint32_t ms) {
    uint32_t ticks_per_ms = CLOCK_GetFreq(kCLOCK_CoreSysClk) / 1000;
    for (uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < ticks_per_ms; j++) {
            __NOP();  // No operation, solo para esperar
        }
    }
}