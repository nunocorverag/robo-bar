#include "startup_flow.h"
#include "../drivers/lcd_i2c.h"
#include "../drivers/keypad.h"
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
static startup_flow_t g_startup_flow;

/*******************************************************************************
 * Public Functions
 ******************************************************************************/
operation_status_t StartupFlow_Run(void) {
    // Inicializar si no se ha hecho ya
    if (!g_startup_flow.hardware_initialized) {
        if (!StartupFlow_Init()) {
            StartupFlow_ShowError("Init falló");
            LED_SetColor(true, false, false);  // LED rojo
            return OP_ERROR;  // Retornamos OP_ERROR si la inicialización falla
        }
    }

    // Controlar el estado de la operación
    operation_status_t result = OP_IN_PROGRESS;

    // Ejecutar el flujo de inicialización paso a paso
    while (result == OP_IN_PROGRESS) {
        result = StartupFlow_Execute();  // Ejecuta paso por paso

        if (result == OP_ERROR) {
            StartupFlow_ShowError("Error ejecución");
            LED_SetColor(true, false, false);  // LED rojo
            Debug_Printf("StartupFlow: Failed with error\r\n");
            break;  // Si hay error, salimos del bucle
        }
    }

    // Si completamos correctamente
    if (result == OP_COMPLETED) {
        LED_SetColor(false, true, false);  // LED verde
        Debug_Printf("StartupFlow: Completed successfully\r\n");
    }

    return result;  // Retorna el estado de la operación (ya sea OP_COMPLETED o OP_ERROR)
}


bool StartupFlow_Init(void) {
    memset(&g_startup_flow, 0, sizeof(startup_flow_t));
    g_startup_flow.current_state = STARTUP_STATE_INIT;
    g_startup_flow.hardware_initialized = false;
    return true;
}

operation_status_t StartupFlow_Execute(void) {
    switch (g_startup_flow.current_state) {
        case STARTUP_STATE_INIT:
            Debug_Printf("StartupFlow: Initializing hardware...\r\n");
            
            // Inicializar I2C primero
            i2c_init();
            Delay(100);
            
            // Probar comunicación I2C antes de inicializar LCD
            if (!lcd_test_communication()) {
                Debug_Printf("StartupFlow: I2C communication failed\r\n");
                g_startup_flow.current_state = STARTUP_STATE_ERROR;
                return OP_ERROR;
            }
            
            Debug_Printf("StartupFlow: I2C communication OK\r\n");
            
            // Inicializar LCD
            if (!lcd_init()) {
                Debug_Printf("StartupFlow: LCD initialization failed\r\n");
                g_startup_flow.current_state = STARTUP_STATE_ERROR;
                return OP_ERROR;
            }
            
            Debug_Printf("StartupFlow: LCD initialized successfully\r\n");
            
            // Inicializar keypad
            keypad_init();
            Debug_Printf("StartupFlow: Keypad initialized\r\n");
            
            // Mostrar pantalla de inicio
            StartupFlow_ShowInitScreen();
            g_startup_flow.current_state = STARTUP_STATE_INITIALIZING;
            g_startup_flow.hardware_initialized = true;
            
            Debug_Printf("StartupFlow: Hardware initialization complete\r\n");
            break;

        case STARTUP_STATE_INITIALIZING:
            // Mostrar pantalla por 2 segundos
            Debug_Printf("StartupFlow: Showing init screen...\r\n");
            Delay(1000);
            g_startup_flow.current_state = STARTUP_STATE_COMPLETE;
            break;

        case STARTUP_STATE_COMPLETE:
            Debug_Printf("StartupFlow: Startup completed successfully\r\n");
            return OP_COMPLETED;

        case STARTUP_STATE_ERROR:
            Debug_Printf("StartupFlow: Error state\r\n");
            StartupFlow_ShowError("Hardware Init");
            return OP_ERROR;
    }
    
    return OP_IN_PROGRESS;
}

/*******************************************************************************
 * Screen Display Functions
 ******************************************************************************/
void StartupFlow_ShowInitScreen(void) {
    if (!lcd_is_initialized()) {
        Debug_Printf("StartupFlow: LCD not initialized, cannot show screen\r\n");
        return;
    }
    
    Debug_Printf("StartupFlow: Displaying init screen\r\n");
    
    if (!lcd_clear()) {
        Debug_Printf("StartupFlow: Failed to clear LCD\r\n");
        return;
    }
    
    if (!lcd_set_cursor(0, 0)) {
        Debug_Printf("StartupFlow: Failed to set cursor\r\n");
        return;
    }
    
    if (!lcd_print("Robo-Bar v1.0")) {
        Debug_Printf("StartupFlow: Failed to print line 1\r\n");
        return;
    }
    
    if (!lcd_set_cursor(1, 0)) {
        Debug_Printf("StartupFlow: Failed to set cursor line 2\r\n");
        return;
    }
    
    if (!lcd_print("Inicializando...")) {
        Debug_Printf("StartupFlow: Failed to print line 2\r\n");
        return;
    }
    
    Debug_Printf("StartupFlow: Init screen displayed successfully\r\n");
}

void StartupFlow_ShowError(const char* error_msg) {
    Debug_Printf("StartupFlow: Showing error: %s\r\n", error_msg);
    
    if (!lcd_is_initialized()) {
        Debug_Printf("StartupFlow: LCD not initialized for error display\r\n");
        return;
    }
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("ERROR:");
    
    if (error_msg) {
        lcd_set_cursor(1, 0);
        lcd_print(error_msg);
    }
}

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/
const char* StartupFlow_StateToString(startup_flow_state_t state) {
    switch (state) {
        case STARTUP_STATE_INIT: return "INIT";
        case STARTUP_STATE_INITIALIZING: return "INITIALIZING";
        case STARTUP_STATE_ERROR: return "ERROR";
        case STARTUP_STATE_COMPLETE: return "COMPLETE";
        default: return "UNKNOWN";
    }
}

void StartupFlow_Restart(void) {
    Debug_Printf("StartupFlow: Restarting flow...\r\n");
    
    // Reset solo el estado, pero mantener hardware_initialized = true
    // para evitar reinicializar el hardware innecesariamente
    bool hw_init_status = g_startup_flow.hardware_initialized;
    
    memset(&g_startup_flow, 0, sizeof(startup_flow_t));
    g_startup_flow.current_state = STARTUP_STATE_INIT;
    g_startup_flow.hardware_initialized = hw_init_status;
    
    Debug_Printf("StartupFlow: Flow restarted\r\n");
}