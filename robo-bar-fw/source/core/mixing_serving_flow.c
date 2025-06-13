/*
 * mixing_serving_flow.c
 * 
 * Mixing and Serving Flow Controller Implementation
 * FRDM-KL25Z Development Board
 */

#include "mixing_serving_flow.h"
#include "../drivers/lcd_i2c.h"
#include "../config/gpio_config.h"
#include "MKL25Z4.h"
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
static mixing_serving_flow_t g_mixing_serving_flow;

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

operation_status_t MixingServingFlow_Run(void) {
    // Inicializar si no se ha hecho ya
    if (!g_mixing_serving_flow.flow_initialized) {
        if (!MixingServingFlow_Init()) {
            MixingServingFlow_ShowError("Init falló");
            LED_SetColor(true, false, false);  // LED rojo
            return OP_ERROR;
        }
    }

    Debug_Printf("MixingServingFlow: Starting complete mixing and serving process...\r\n");
    
    // Ejecutar todo el proceso secuencialmente con delays bloqueantes
    LED_SetColor(false, false, true);  // LED azul para mezclado
    MixingServingFlow_ShowMixingScreen();
    
    // Iniciar mezclado
    MOTOR_RELAY_On();
    Debug_Printf("MixingServingFlow: Mixing for %d ms...\r\n", g_mixing_serving_flow.config.mixing_time_ms);
    Delay(g_mixing_serving_flow.config.mixing_time_ms);
    
    // Detener mezclado
    MOTOR_RELAY_Off();
    Debug_Printf("MixingServingFlow: Mixing complete\r\n");
    
    // Pausa entre mezclado y servido
    if (g_mixing_serving_flow.config.pause_time_ms > 0) {
        Debug_Printf("MixingServingFlow: Pausing for %d ms...\r\n", g_mixing_serving_flow.config.pause_time_ms);
        Delay(g_mixing_serving_flow.config.pause_time_ms);
    }
    
    // Mostrar pantalla de servido e iniciar servido
    LED_SetColor(true, false, true);  // LED magenta para servido
    MixingServingFlow_ShowServingScreen();
    
    DISPENSER_RELAY_On();  // Activar dispensador
    Debug_Printf("MixingServingFlow: Serving for %d ms...\r\n", g_mixing_serving_flow.config.serving_time_ms);
    Delay(g_mixing_serving_flow.config.serving_time_ms);
    
    // Detener dispensador
    DISPENSER_RELAY_Off();
    Debug_Printf("MixingServingFlow: Serving complete\r\n");
    
    // Mostrar pantalla de finalización
    LED_SetColor(false, true, false);  // LED verde
    MixingServingFlow_ShowCompletionScreen();
    Delay(2000);  // Mostrar mensaje por 2 segundos
    
    Debug_Printf("MixingServingFlow: Process completed successfully\r\n");
    return OP_COMPLETED;
}

bool MixingServingFlow_Init(void) {
    Debug_Printf("MixingServingFlow: Initializing...\r\n");
    
    // Initialize structure
    memset(&g_mixing_serving_flow, 0, sizeof(mixing_serving_flow_t));
    g_mixing_serving_flow.current_state = MIXING_SERVING_STATE_INIT;
    g_mixing_serving_flow.flow_initialized = false;
    
    // Set default configuration
    MixingServingFlow_SetConfig(5000, 5000, 5000); // 5s mixing, 5s serving, 5s pause
    
    // Initialize hardware
    MOTOR_RELAY_Init();
    DISPENSER_RELAY_Init();
    
    g_mixing_serving_flow.flow_initialized = true;
    Debug_Printf("MixingServingFlow: Initialization complete\r\n");
    return true;
}

// Función simplificada para compatibilidad, pero ahora MixingServingFlow_Run hace todo el trabajo
operation_status_t MixingServingFlow_Execute(void) {
    switch (g_mixing_serving_flow.current_state) {
        case MIXING_SERVING_STATE_INIT:
            g_mixing_serving_flow.current_state = MIXING_SERVING_STATE_COMPLETE;
            return OP_COMPLETED;
            
        case MIXING_SERVING_STATE_COMPLETE:
            return OP_COMPLETED;
            
        case MIXING_SERVING_STATE_ERROR:
            return OP_ERROR;
            
        default:
            return OP_ERROR;
    }
}

void MixingServingFlow_SetConfig(uint16_t mixing_time_ms, uint16_t serving_time_ms, uint16_t pause_time_ms) {
    g_mixing_serving_flow.config.mixing_time_ms = mixing_time_ms;
    g_mixing_serving_flow.config.serving_time_ms = serving_time_ms;
    g_mixing_serving_flow.config.pause_time_ms = pause_time_ms;
    
    Debug_Printf("MixingServingFlow: Config set - Mixing: %dms, Serving: %dms, Pause: %dms\r\n",
                 mixing_time_ms, serving_time_ms, pause_time_ms);
}

void MixingServingFlow_Restart(void) {
    Debug_Printf("MixingServingFlow: Restarting flow...\r\n");
    
    // Ensure hardware is safe
    MOTOR_RELAY_Off();
    DISPENSER_RELAY_Off();
    
    // Reset state
    g_mixing_serving_flow.current_state = MIXING_SERVING_STATE_INIT;
    g_mixing_serving_flow.start_time = 0;
    
    Debug_Printf("MixingServingFlow: Flow restarted\r\n");
}

/*******************************************************************************
 * Hardware Control Functions - Motor Relay
 ******************************************************************************/

void MOTOR_RELAY_Init(void) {
    SIM->SCGC5 |= MOTOR_RELAY_PORT_CLK;
    PORTE->PCR[MOTOR_RELAY_PIN] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
    GPIOE->PDDR |= (1U << MOTOR_RELAY_PIN);
    GPIOE->PCOR = (1U << MOTOR_RELAY_PIN); // Relé apagado al inicio
    
    Debug_Printf("MixingServingFlow: Motor Relay initialized (PTE31)\r\n");
}

void MOTOR_RELAY_On(void) {
    GPIOE->PSOR = (1U << MOTOR_RELAY_PIN); // Relé ON para motor de mezclado
    Debug_Printf("MixingServingFlow: Motor Relay ON (mixing started)\r\n");
}

void MOTOR_RELAY_Off(void) {
    GPIOE->PCOR = (1U << MOTOR_RELAY_PIN); // Relé OFF para motor de mezclado
    Debug_Printf("MixingServingFlow: Motor Relay OFF (mixing stopped)\r\n");
}

/*******************************************************************************
 * Hardware Control Functions - Dispenser Relay
 ******************************************************************************/

void DISPENSER_RELAY_Init(void) {
    SIM->SCGC5 |= DISPENSER_RELAY_PORT_CLK;
    PORTE->PCR[DISPENSER_RELAY_PIN] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
    GPIOE->PDDR |= (1U << DISPENSER_RELAY_PIN);
    GPIOE->PCOR = (1U << DISPENSER_RELAY_PIN); // Relé apagado al inicio
    
    Debug_Printf("MixingServingFlow: Dispenser Relay initialized (PTE23)\r\n");
}

void DISPENSER_RELAY_On(void) {
    GPIOE->PSOR = (1U << DISPENSER_RELAY_PIN); // Relé ON para dispensador
    Debug_Printf("MixingServingFlow: Dispenser Relay ON (serving started)\r\n");
}

void DISPENSER_RELAY_Off(void) {
    GPIOE->PCOR = (1U << DISPENSER_RELAY_PIN); // Relé OFF para dispensador
    Debug_Printf("MixingServingFlow: Dispenser Relay OFF (serving stopped)\r\n");
}

/*******************************************************************************
 * Legacy Functions - Mantener compatibilidad
 ******************************************************************************/

// Mantener las funciones RELAY_* para compatibilidad con código existente
void RELAY_Init(void) {
    MOTOR_RELAY_Init();
}

void RELAY_On(void) {
    MOTOR_RELAY_On();
}

void RELAY_Off(void) {
    MOTOR_RELAY_Off();
}

void ServirBebida(void) {
    // Activar dispensador por tiempo definido
    DISPENSER_RELAY_On();
    Delay(g_mixing_serving_flow.config.serving_time_ms); 
    
    // Desactivar dispensador
    DISPENSER_RELAY_Off();
    
    Debug_Printf("MixingServingFlow: Beverage served using dispenser relay\r\n");
}

/*******************************************************************************
 * Screen Display Functions
 ******************************************************************************/

void MixingServingFlow_ShowMixingScreen(void) {
    if (!lcd_is_initialized()) {
        Debug_Printf("MixingServingFlow: LCD not initialized for mixing screen\r\n");
        return;
    }
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Mezclando");
    lcd_set_cursor(1, 0);
    lcd_print("bebida...");
    
    Debug_Printf("MixingServingFlow: Mixing screen displayed\r\n");
}

void MixingServingFlow_ShowServingScreen(void) {
    if (!lcd_is_initialized()) {
        Debug_Printf("MixingServingFlow: LCD not initialized for serving screen\r\n");
        return;
    }
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Sirviendo");
    lcd_set_cursor(1, 0);
    lcd_print("bebida...");
    
    Debug_Printf("MixingServingFlow: Serving screen displayed\r\n");
}

void MixingServingFlow_ShowCompletionScreen(void) {
    if (!lcd_is_initialized()) {
        Debug_Printf("MixingServingFlow: LCD not initialized for completion screen\r\n");
        return;
    }
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Bebida lista!");
    lcd_set_cursor(1, 0);
    lcd_print("Disfrute!");
    
    Debug_Printf("MixingServingFlow: Completion screen displayed\r\n");
}

void MixingServingFlow_ShowError(const char* error_msg) {
    Debug_Printf("MixingServingFlow: Showing error: %s\r\n", error_msg);
    
    if (!lcd_is_initialized()) {
        Debug_Printf("MixingServingFlow: LCD not initialized for error display\r\n");
        return;
    }
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("ERROR Mezcla:");
    
    if (error_msg) {
        lcd_set_cursor(1, 0);
        lcd_print(error_msg);
    }
    
    // Flash red LED
    for (int i = 0; i < 3; i++) {
        LED_SetColor(true, false, false);   // Red
        Delay(300);
        LED_SetColor(false, false, false);  // Off
        Delay(300);
    }
}

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

const char* MixingServingFlow_StateToString(mixing_serving_flow_state_t state) {
    switch (state) {
        case MIXING_SERVING_STATE_INIT: return "INIT";
        case MIXING_SERVING_STATE_MIXING: return "MIXING";
        case MIXING_SERVING_STATE_SERVING: return "SERVING";
        case MIXING_SERVING_STATE_COMPLETE: return "COMPLETE";
        case MIXING_SERVING_STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

// Función simplificada - el timing real se maneja con Delay()
uint32_t MixingServingFlow_GetTime(void) {
    static uint32_t counter = 0;
    return counter++;
}