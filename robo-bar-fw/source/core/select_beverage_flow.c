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
 * External Functions
 ******************************************************************************/
extern void Debug_Printf(const char* format, ...);
extern void LED_SetColor(bool red, bool green, bool blue);
extern void Delay(uint32_t ms);

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
 * Public Functions
 ******************************************************************************/

operation_status_t BeverageSelectFlow_Run(void) {
    // Inicializar si no se ha hecho ya
    if (!g_beverage_select_flow.flow_initialized) {
        if (!BeverageSelectFlow_Init()) {
            BeverageSelectFlow_ShowError("Init falló");
            LED_SetColor(true, false, false);  // LED rojo
            return OP_ERROR;
        }
    }

    // Controlar el estado de la operación
    operation_status_t result = OP_IN_PROGRESS;

    // Ejecutar el flujo de selección de bebida paso a paso
    while (result == OP_IN_PROGRESS) {
        result = BeverageSelectFlow_Execute();  // Ejecuta paso por paso

        if (result == OP_ERROR) {
            BeverageSelectFlow_ShowError("Error ejecución");
            LED_SetColor(true, false, false);  // LED rojo
            Debug_Printf("BeverageSelectFlow: Failed with error\r\n");
            break;
        }
    }

    // Si completamos correctamente
    if (result == OP_COMPLETED) {
        LED_SetColor(false, true, false);  // LED verde
        Debug_Printf("BeverageSelectFlow: Completed successfully\r\n");
    }

    return result;
}

bool BeverageSelectFlow_Init(void) {
    Debug_Printf("BeverageSelectFlow: Initializing...\r\n");
    
    // Initialize structure
    memset(&g_beverage_select_flow, 0, sizeof(beverage_select_flow_t));
    g_beverage_select_flow.current_state = BEVERAGE_SELECT_STATE_INIT;
    g_beverage_select_flow.flow_initialized = false;
    
    // Initialize servo control
    servo_control_init();
    
    g_beverage_select_flow.flow_initialized = true;
    Debug_Printf("BeverageSelectFlow: Initialization complete\r\n");
    return true;
}

operation_status_t BeverageSelectFlow_Execute(void) {
    switch (g_beverage_select_flow.current_state) {
        case BEVERAGE_SELECT_STATE_INIT:
            Debug_Printf("BeverageSelectFlow: Starting beverage selection...\r\n");
            g_beverage_select_flow.current_state = BEVERAGE_SELECT_STATE_SELECTION;
            break;
            
        case BEVERAGE_SELECT_STATE_SELECTION:
            Debug_Printf("BeverageSelectFlow: Waiting for beverage selection...\r\n");
            g_beverage_select_flow.selected_beverage = BeverageSelectFlow_SelectBeverage();
            
            if (g_beverage_select_flow.selected_beverage >= BEVERAGE_MOJITO && 
                g_beverage_select_flow.selected_beverage <= BEVERAGE_VODKA_TONIC) {
                
                // Get recipe for selected beverage
                g_beverage_select_flow.current_recipe = BeverageSelectFlow_GetRecipe(g_beverage_select_flow.selected_beverage);
                if (g_beverage_select_flow.current_recipe != NULL) {
                    Debug_Printf("BeverageSelectFlow: Selected %s\r\n", g_beverage_select_flow.current_recipe->nombre);
                    g_beverage_select_flow.current_state = BEVERAGE_SELECT_STATE_DISPENSING;
                } else {
                    Debug_Printf("BeverageSelectFlow: Failed to get recipe\r\n");
                    g_beverage_select_flow.current_state = BEVERAGE_SELECT_STATE_ERROR;
                }
            } else {
                Debug_Printf("BeverageSelectFlow: Invalid beverage selection\r\n");
                g_beverage_select_flow.current_state = BEVERAGE_SELECT_STATE_ERROR;
            }
            break;
            
        case BEVERAGE_SELECT_STATE_DISPENSING:
            Debug_Printf("BeverageSelectFlow: Starting dispensing process...\r\n");
            BeverageSelectFlow_ShowDispensingScreen(g_beverage_select_flow.current_recipe->nombre);
            
            if (BeverageSelectFlow_DispenseBeverage(g_beverage_select_flow.current_recipe)) {
                g_beverage_select_flow.current_state = BEVERAGE_SELECT_STATE_DISPENSING_COMPLETE;
            } else {
                Debug_Printf("BeverageSelectFlow: Dispensing failed\r\n");
                g_beverage_select_flow.current_state = BEVERAGE_SELECT_STATE_ERROR;
            }
            break;
            
        case BEVERAGE_SELECT_STATE_DISPENSING_COMPLETE:
            Debug_Printf("BeverageSelectFlow: Dispensing completed\r\n");
            BeverageSelectFlow_ShowCompletionScreen(g_beverage_select_flow.current_recipe->nombre);
            Delay(2000);  // Show completion message for 2 seconds
            g_beverage_select_flow.current_state = BEVERAGE_SELECT_STATE_COMPLETE;
            break;
            
        case BEVERAGE_SELECT_STATE_COMPLETE:
            Debug_Printf("BeverageSelectFlow: Flow completed successfully\r\n");
            return OP_COMPLETED;
            
        case BEVERAGE_SELECT_STATE_ERROR:
            Debug_Printf("BeverageSelectFlow: Error state\r\n");
            BeverageSelectFlow_ShowError("Proceso fallido");
            return OP_ERROR;
            
        default:
            Debug_Printf("BeverageSelectFlow: Unknown state\r\n");
            g_beverage_select_flow.current_state = BEVERAGE_SELECT_STATE_ERROR;
            return OP_ERROR;
    }
    
    return OP_IN_PROGRESS;
}

/*******************************************************************************
 * Screen Display Functions
 ******************************************************************************/

beverage_type_t BeverageSelectFlow_SelectBeverage(void) {
    if (!lcd_is_initialized()) {
        Debug_Printf("BeverageSelectFlow: LCD not initialized, cannot show selection screen\r\n");
        return BEVERAGE_MOJITO;  // Fallback
    }
    
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
    Delay(recipe->tiempo_ms);
    
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
    if (!lcd_is_initialized()) {
        Debug_Printf("BeverageSelectFlow: LCD not initialized for dispensing screen\r\n");
        return;
    }
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Dispensando");
    lcd_set_cursor(1, 0);
    lcd_print("liquidos...");
    
    Debug_Printf("BeverageSelectFlow: Dispensing screen shown for %s\r\n", beverage_name);
}

void BeverageSelectFlow_ShowCompletionScreen(const char* beverage_name) {
    if (!lcd_is_initialized()) {
        Debug_Printf("BeverageSelectFlow: LCD not initialized for completion screen\r\n");
        return;
    }
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Bebida lista!");
    lcd_set_cursor(1, 0);
    lcd_print(beverage_name);
    
    Debug_Printf("BeverageSelectFlow: Completion screen shown for %s\r\n", beverage_name);
}

void BeverageSelectFlow_ShowError(const char* error_msg) {
    Debug_Printf("BeverageSelectFlow: Showing error: %s\r\n", error_msg);
    
    if (!lcd_is_initialized()) {
        Debug_Printf("BeverageSelectFlow: LCD not initialized for error display\r\n");
        return;
    }
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("ERROR:");
    
    if (error_msg) {
        lcd_set_cursor(1, 0);
        lcd_print(error_msg);
    }
    
    // Flash red LED
    for (int i = 0; i < 5; i++) {
        LED_SetColor(true, false, false);   // Red
        Delay(200);
        LED_SetColor(false, false, false);  // Off
        Delay(200);
    }
}

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

const char* BeverageSelectFlow_StateToString(beverage_select_flow_state_t state) {
    switch (state) {
        case BEVERAGE_SELECT_STATE_INIT: return "INIT";
        case BEVERAGE_SELECT_STATE_SELECTION: return "SELECTION";
        case BEVERAGE_SELECT_STATE_DISPENSING: return "DISPENSING";
        case BEVERAGE_SELECT_STATE_DISPENSING_COMPLETE: return "DISPENSING_COMPLETE";
        case BEVERAGE_SELECT_STATE_COMPLETE: return "COMPLETE";
        case BEVERAGE_SELECT_STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

const receta_bebida_t* BeverageSelectFlow_GetRecipe(beverage_type_t beverage_type) {
    if (beverage_type >= BEVERAGE_MOJITO && beverage_type <= BEVERAGE_VODKA_TONIC) {
        return &recetas[beverage_type - 1];  // Convert 1-based to 0-based index
    }
    return NULL;
}

void BeverageSelectFlow_Restart(void) {
    Debug_Printf("BeverageSelectFlow: Restarting flow...\r\n");
    
    memset(&g_beverage_select_flow, 0, sizeof(beverage_select_flow_t));
    g_beverage_select_flow.current_state = BEVERAGE_SELECT_STATE_INIT;
    
    Debug_Printf("BeverageSelectFlow: Flow restarted\r\n");
}
