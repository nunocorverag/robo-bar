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
 * Hardware Definitions
 ******************************************************************************/
#define RELAY_PIN 31 // PTE31
#define RELAY_PORT_CLK SIM_SCGC5_PORTE_MASK

// Configuración PWM para servo
#define PWM_MOD 20833
#define SERVO_MIN_PULSE 1041    // 0° - Válvula cerrada
#define SERVO_MAX_PULSE 4167    // 180° - Válvula abierta
#define SERVO_90_PULSE 2604     // 90° - Posición intermedia

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
    RELAY_On();
    Debug_Printf("MixingServingFlow: Mixing for %d ms...\r\n", g_mixing_serving_flow.config.mixing_time_ms);
    Delay(g_mixing_serving_flow.config.mixing_time_ms);
    
    // Detener mezclado
    RELAY_Off();
    Debug_Printf("MixingServingFlow: Mixing complete\r\n");
    
    // Pausa entre mezclado y servido
    if (g_mixing_serving_flow.config.pause_time_ms > 0) {
        Debug_Printf("MixingServingFlow: Pausing for %d ms...\r\n", g_mixing_serving_flow.config.pause_time_ms);
        Delay(g_mixing_serving_flow.config.pause_time_ms);
    }
    
    // Mostrar pantalla de servido e iniciar servido
    LED_SetColor(true, false, true);  // LED magenta para servido
    MixingServingFlow_ShowServingScreen();
    
    servo5_set_angle(180);  // Abrir válvula
    Debug_Printf("MixingServingFlow: Serving for %d ms...\r\n", g_mixing_serving_flow.config.serving_time_ms);
    Delay(g_mixing_serving_flow.config.serving_time_ms);
    
    // Cerrar válvula
    servo5_set_angle(0);
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
    RELAY_Init();
    servo5_init();
    
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
    RELAY_Off();
    servo5_set_angle(0);
    
    // Reset state
    g_mixing_serving_flow.current_state = MIXING_SERVING_STATE_INIT;
    g_mixing_serving_flow.start_time = 0;
    
    Debug_Printf("MixingServingFlow: Flow restarted\r\n");
}

/*******************************************************************************
 * Hardware Control Functions
 ******************************************************************************/

void RELAY_Init(void) {
    SIM->SCGC5 |= RELAY_PORT_CLK;
    PORTE->PCR[RELAY_PIN] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
    GPIOE->PDDR |= (1U << RELAY_PIN);
    GPIOE->PCOR = (1U << RELAY_PIN); // Relé apagado al inicio (lógica inversa)
    
    Debug_Printf("MixingServingFlow: Relay initialized (PTE31)\r\n");
}

void RELAY_On(void) {
    GPIOE->PSOR = (1U << RELAY_PIN); // Relé ON (lógica inversa - HIGH activa)
    Debug_Printf("MixingServingFlow: Relay ON (mixing started)\r\n");
}

void RELAY_Off(void) {
    GPIOE->PCOR = (1U << RELAY_PIN); // Relé OFF (lógica inversa - LOW desactiva)
    Debug_Printf("MixingServingFlow: Relay OFF (mixing stopped)\r\n");
}

void servo5_init(void) {
    // Habilitar relojes para puerto D y TPM0
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // TPM usa MCGFLLCLK (48 MHz)
    
    // Configurar PTD0 como TPM0_CH0 para servo 5
    PORTD->PCR[0] = PORT_PCR_MUX(4); // PTD0 → TPM0_CH0
    
    // Configurar TPM0 para CH0 (servo 5)
    TPM0->SC = 0; // Detener TPM0
    TPM0->MOD = PWM_MOD; // Período de 20ms
    TPM0->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // PWM modo edge-aligned
    TPM0->CONTROLS[0].CnV = SERVO_MIN_PULSE; // Iniciar en 0° (válvula cerrada)
    TPM0->SC = TPM_SC_PS(4) | TPM_SC_CMOD(1); // Prescaler /16, iniciar TPM
    
    Debug_Printf("MixingServingFlow: Servo 5 initialized (PTD0)\r\n");
}

void servo5_set_angle(uint16_t angle) {
    uint16_t pulse;
    
    if (angle == 0) {
        pulse = SERVO_MIN_PULSE;    // 0° - Válvula cerrada
    } else if (angle == 90) {
        pulse = SERVO_90_PULSE;     // 90° - Posición intermedia
    } else if (angle == 180) {
        pulse = SERVO_MAX_PULSE;    // 180° - Válvula abierta
    } else {
        // Interpolación lineal para ángulos intermedios
        pulse = SERVO_MIN_PULSE + ((SERVO_MAX_PULSE - SERVO_MIN_PULSE) * angle) / 180;
    }
    
    TPM0->CONTROLS[0].CnV = pulse;
    Debug_Printf("MixingServingFlow: Servo 5 set to %d degrees\r\n", angle);
}

void ServirBebida(void) {
    // Abrir válvula completamente (servo 5 en PTD0)
    servo5_set_angle(180);
    Delay(150); // Tiempo de dispensado
    
    // Cerrar válvula completamente
    servo5_set_angle(0);
    
    Debug_Printf("MixingServingFlow: Beverage served\r\n");
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