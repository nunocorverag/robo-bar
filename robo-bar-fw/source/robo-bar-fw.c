/*
 * Robo-Bar System - Main Application (Complete with Mixing/Serving)
 * FRDM-KL25Z Development Board
 */
#include "config/board_init.h"
#include "core/startup_flow.h"
#include "core/select_beverage_flow.h"
#include "core/sensor_check_flow.h"
#include "core/mixing_serving_flow.h"
#include "shared/system_types.h"
#include <stdarg.h>
#include <stdio.h>

// Estados del sistema principal
typedef enum {
    SYSTEM_STATE_STARTUP,
    SYSTEM_STATE_SENSOR_CHECK,
    SYSTEM_STATE_BEVERAGE_SELECTION,
    SYSTEM_STATE_MIXING_SERVING,
    SYSTEM_STATE_WAITING_RESTART,
    SYSTEM_STATE_ERROR
} system_state_t;

// Variables globales del sistema
static system_state_t g_system_state = SYSTEM_STATE_STARTUP;
static bool g_startup_done = false;

/*******************************************************************************
 * Funciones de Utilidad
 ******************************************************************************/
void Debug_Printf(const char* format, ...) {
    va_list args;
    char buffer[256];

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    for (int i = 0; buffer[i] != '\0'; i++) {
        while (!(UART0->S1 & UART_S1_TDRE_MASK));
        UART0->D = buffer[i];
    }
}

void LED_SetColor(bool red, bool green, bool blue) {
    GPIO_WritePinOutput(LED_RED_GPIO, LED_RED_PIN, !red);
    GPIO_WritePinOutput(LED_GREEN_GPIO, LED_GREEN_PIN, !green);
    GPIO_WritePinOutput(LED_BLUE_GPIO, LED_BLUE_PIN, !blue);
}

// Delay simple sin SysTick
void Delay(uint32_t ms) {
    // Aproximado para 48MHz (ajusta según tu frecuencia)
    volatile uint32_t delay = ms * (SystemCoreClock / 1000 / 4);
    while (delay--);
}

/*******************************************************************************
 * Controlador Principal del Sistema
 ******************************************************************************/
void SystemController_Run(void) {
    static operation_status_t op_status = OP_IN_PROGRESS;

    switch (g_system_state) {
        case SYSTEM_STATE_STARTUP:
            if (!g_startup_done) {
                Debug_Printf("System: Starting initialization...\r\n");
                LED_SetColor(true, true, false); // Amarillo - Inicializando
                
                operation_status_t op_status = StartupFlow_Run();

                if (op_status == OP_COMPLETED) {
                    Debug_Printf("System: Startup completed\r\n");
                    g_startup_done = true;
                    g_system_state = SYSTEM_STATE_SENSOR_CHECK;
                    LED_SetColor(false, true, false); // Verde
                    Delay(1000);
                    
                    // RESTART: Preparar el siguiente flujo
                    SensorCheckFlow_Restart();
                    op_status = OP_IN_PROGRESS; // Reset op_status
                }
                else if (op_status == OP_ERROR) {
                    Debug_Printf("ERROR: Startup failed!\r\n");
                    LED_SetColor(true, false, false); // Rojo
                    g_system_state = SYSTEM_STATE_ERROR;
                }
            }
            else {
                g_system_state = SYSTEM_STATE_SENSOR_CHECK;
                // RESTART: Preparar el flujo de sensores
                SensorCheckFlow_Restart();
                op_status = OP_IN_PROGRESS; // Reset op_status
            }
            break;

        case SYSTEM_STATE_SENSOR_CHECK:
            Debug_Printf("System: Checking sensors...\r\n");
            LED_SetColor(false, false, true); // Azul - Verificando sensores
            
            op_status = SensorCheckFlow_Run();
            
            if (op_status == OP_COMPLETED) {
                Debug_Printf("System: Sensors OK\r\n");
                g_system_state = SYSTEM_STATE_BEVERAGE_SELECTION;
                LED_SetColor(false, true, false); // Verde
                Delay(500);
                
                // RESTART: Preparar el siguiente flujo
                BeverageSelectFlow_Restart();
                op_status = OP_IN_PROGRESS; // Reset op_status
            }
            else if (op_status == OP_ERROR) {
                Debug_Printf("ERROR: Sensor check failed!\r\n");
                LED_SetColor(true, false, false); // Rojo
                g_system_state = SYSTEM_STATE_ERROR;
            }
            break;

        case SYSTEM_STATE_BEVERAGE_SELECTION:
            Debug_Printf("System: Ready for beverage selection\r\n");
            LED_SetColor(true, false, true); // Magenta - Selección
            
            op_status = BeverageSelectFlow_Run();
            
            if (op_status == OP_COMPLETED) {
                Debug_Printf("System: Beverage dispensed! Starting mixing and serving...\r\n");
                g_system_state = SYSTEM_STATE_MIXING_SERVING;
                LED_SetColor(false, true, false); // Verde
                Delay(500);
                
                // RESTART: Preparar el flujo de mezclado y servido
                MixingServingFlow_Restart();
                op_status = OP_IN_PROGRESS; // Reset op_status
            }
            else if (op_status == OP_ERROR) {
                Debug_Printf("ERROR: Beverage selection failed!\r\n");
                LED_SetColor(true, false, false); // Rojo
                g_system_state = SYSTEM_STATE_ERROR;
            }
            break;

        case SYSTEM_STATE_MIXING_SERVING:
            Debug_Printf("System: Mixing and serving beverage...\r\n");
            LED_SetColor(false, true, true); // Cian - Mezclando y sirviendo
            
            op_status = MixingServingFlow_Run();
            
            if (op_status == OP_COMPLETED) {
                Debug_Printf("System: Mixing and serving completed!\r\n");
                g_system_state = SYSTEM_STATE_WAITING_RESTART;
                LED_SetColor(false, true, false); // Verde
                
                // RESTART: Reset op_status para el próximo ciclo
                op_status = OP_IN_PROGRESS;
            }
            else if (op_status == OP_ERROR) {
                Debug_Printf("ERROR: Mixing and serving failed!\r\n");
                LED_SetColor(true, false, false); // Rojo
                g_system_state = SYSTEM_STATE_ERROR;
            }
            break;

        case SYSTEM_STATE_WAITING_RESTART:
            Debug_Printf("System: Complete cycle finished. Restarting in 3 seconds...\r\n");
            LED_SetColor(false, true, false); // Verde
            Delay(3000);
            g_system_state = SYSTEM_STATE_SENSOR_CHECK;
            
            // RESTART: Preparar el flujo de sensores para el nuevo ciclo
            SensorCheckFlow_Restart();
            op_status = OP_IN_PROGRESS; // Reset op_status
            break;

        case SYSTEM_STATE_ERROR:
            Debug_Printf("System: Critical error! Attempting recovery in 5 seconds...\r\n");
            LED_SetColor(true, false, false); // Rojo
            Delay(5000);
            
            // RESTART: Intentar recuperación reiniciando todos los flujos
            StartupFlow_Restart();
            SensorCheckFlow_Restart();
            BeverageSelectFlow_Restart();
            MixingServingFlow_Restart();
            
            // Reiniciar desde el chequeo de sensores
            g_system_state = SYSTEM_STATE_SENSOR_CHECK;
            op_status = OP_IN_PROGRESS; // Reset op_status
            break;
    }
}

/*******************************************************************************
 * Función Principal
 ******************************************************************************/
int main(void) {
    // Inicializar hardware básico
    BOARD_InitAll();
    
    // Mensaje inicial
    Debug_Printf("\r\n\r\n=== Robo-Bar System (Complete with Mixing/Serving) ===\r\n");
    Debug_Printf("System Initialized. Starting main loop...\r\n");
    Debug_Printf("Flow sequence: Startup -> Sensors -> Beverage Selection -> Mixing/Serving -> Restart\r\n");
    
    // LED inicial - Amarillo para indicar inicialización
    LED_SetColor(true, true, false);
    
    // Bucle principal
    while (1) {
//        StartupFlow_Run();
//        BeverageSelectFlow_Run();
//        BeverageSelectFlow_Restart();
//        MixingServingFlow_Run();
//        MixingServingFlow_Restart();
        SystemController_Run();
        Delay(100); // Pequeño delay para estabilidad
    }
}
