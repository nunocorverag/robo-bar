/*
 * sensor_check_flow.c
 * 
 * Sensor Check and Mode Selection Flow Controller Implementation (MODIFIED for 3 sensors)
 * FRDM-KL25Z Development Board
 */

#include "sensor_check_flow.h"
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
static sensor_check_flow_t g_sensor_check_flow;
static const char* SENSOR_NAMES[3] = {"Liquido 1", "Liquido 2", "Liquido 3"};

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/
static bool read_sensor(uint8_t sensor_id);

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

operation_status_t SensorCheckFlow_Run(void) {
    // Inicializar si no se ha hecho ya
    if (!g_sensor_check_flow.hardware_initialized) {
        if (!SensorCheckFlow_Init()) {
            SensorCheckFlow_ShowError("Init falló");
            LED_SetColor(true, false, false);  // LED rojo
            return OP_ERROR;
        }
    }

    // Controlar el estado de la operación
    operation_status_t result = OP_IN_PROGRESS;

    // Ejecutar el flujo de verificación de sensores paso a paso
    while (result == OP_IN_PROGRESS) {
        result = SensorCheckFlow_Execute();

        if (result == OP_ERROR) {
            SensorCheckFlow_ShowError("Error ejecución");
            LED_SetColor(true, false, false);  // LED rojo
            Debug_Printf("SensorCheckFlow: Failed with error\r\n");
            break;
        }
    }

    // Si completamos correctamente
    if (result == OP_COMPLETED) {
        LED_SetColor(false, true, false);  // LED verde
        Debug_Printf("SensorCheckFlow: Completed successfully\r\n");
    }

    return result;
}

bool SensorCheckFlow_Init(void) {
    Debug_Printf("SensorCheckFlow: Initializing...\r\n");
    
    // Initialize structure
    memset(&g_sensor_check_flow, 0, sizeof(sensor_check_flow_t));
    g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_INIT;
    g_sensor_check_flow.hardware_initialized = true;
    
    Debug_Printf("SensorCheckFlow: Initialization complete\r\n");
    return true;
}

operation_status_t SensorCheckFlow_Execute(void) {
    switch (g_sensor_check_flow.current_state) {
        case SENSOR_CHECK_STATE_INIT:
            Debug_Printf("SensorCheckFlow: Starting sensor verification...\r\n");
            g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_CHECKING_SENSORS;
            break;
            
        case SENSOR_CHECK_STATE_CHECKING_SENSORS:
            Debug_Printf("SensorCheckFlow: Checking sensors...\r\n");
            if (SensorCheckFlow_CheckSensors()) {
                g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_MODE_SELECTION;
            } else {
                g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_SENSOR_ERROR;
            }
            break;
            
        case SENSOR_CHECK_STATE_SENSOR_ERROR:
            Debug_Printf("SensorCheckFlow: Handling sensor error...\r\n");
            SensorCheckFlow_ShowSensorError(&g_sensor_check_flow.sensor_status);
            if (g_sensor_check_flow.sensor_status.can_operate) {
                g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_MODE_SELECTION;
            } else {
                SensorCheckFlow_ShowRefillInstructions();
                g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_WAITING_REFILL;
            }
            break;
            
        case SENSOR_CHECK_STATE_MODE_SELECTION:
            Debug_Printf("SensorCheckFlow: Mode selection...\r\n");
            g_sensor_check_flow.selected_mode = SensorCheckFlow_SelectMode();
            if (g_sensor_check_flow.selected_mode == OPERATION_MODE_NORMAL) {
                g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_READY;
            } else if (g_sensor_check_flow.selected_mode == OPERATION_MODE_REFILL) {
                g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_REFILL_MODE;
            } else {
                g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_ERROR;
                return OP_ERROR;
            }
            break;
            
        case SENSOR_CHECK_STATE_REFILL_MODE:
            Debug_Printf("SensorCheckFlow: Refill mode...\r\n");
            SensorCheckFlow_ShowRefillInstructions();
            g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_WAITING_REFILL;
            break;
            
        case SENSOR_CHECK_STATE_WAITING_REFILL:
            Debug_Printf("SensorCheckFlow: Waiting for refill completion...\r\n");
            g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_RECHECK_SENSORS;
            break;
            
        case SENSOR_CHECK_STATE_RECHECK_SENSORS:
            Debug_Printf("SensorCheckFlow: Rechecking sensors...\r\n");
            if (SensorCheckFlow_CheckSensors()) {
                g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_READY;
            } else {
                g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_SENSOR_ERROR;
            }
            break;
            
        case SENSOR_CHECK_STATE_READY:
            Debug_Printf("SensorCheckFlow: System ready...\r\n");
            SensorCheckFlow_ShowReadyScreen();
            g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_COMPLETE;
            break;
            
        case SENSOR_CHECK_STATE_COMPLETE:
            Debug_Printf("SensorCheckFlow: Flow completed successfully\r\n");
            return OP_COMPLETED;
            
        case SENSOR_CHECK_STATE_ERROR:
        default:
            Debug_Printf("SensorCheckFlow: Error state\r\n");
            SensorCheckFlow_ShowError("Flow Error");
            return OP_ERROR;
    }
    
    return OP_IN_PROGRESS;
}

/*******************************************************************************
 * Screen Display Functions
 ******************************************************************************/

bool SensorCheckFlow_CheckSensors(void) {
    if (!lcd_is_initialized()) {
        Debug_Printf("SensorCheckFlow: LCD not initialized, cannot show sensor check\r\n");
        return false;
    }
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Verificando...");
    lcd_set_cursor(1, 0);
    lcd_print("Sensores");
    
    Debug_Printf("SensorCheckFlow: Checking sensors...\r\n");
    
    // Reset sensor status
    memset(&g_sensor_check_flow.sensor_status, 0, sizeof(sensor_status_t));
    
    // Check each sensor (only 3 now)
    g_sensor_check_flow.sensor_status.sensor_1_ok = read_sensor(1);
    g_sensor_check_flow.sensor_status.sensor_2_ok = read_sensor(2);
    g_sensor_check_flow.sensor_status.sensor_3_ok = read_sensor(3);
    
    // Create bitmask of low sensors (only 3 bits used)
    if (!g_sensor_check_flow.sensor_status.sensor_1_ok) g_sensor_check_flow.sensor_status.low_sensors |= (1 << 0);
    if (!g_sensor_check_flow.sensor_status.sensor_2_ok) g_sensor_check_flow.sensor_status.low_sensors |= (1 << 1);
    if (!g_sensor_check_flow.sensor_status.sensor_3_ok) g_sensor_check_flow.sensor_status.low_sensors |= (1 << 2);
    
    // Check if system can operate - MODIFICADO: solo error crítico si TODOS los sensores están en bajo
    uint8_t ok_count = 0;
    if (g_sensor_check_flow.sensor_status.sensor_1_ok) ok_count++;
    if (g_sensor_check_flow.sensor_status.sensor_2_ok) ok_count++;
    if (g_sensor_check_flow.sensor_status.sensor_3_ok) ok_count++;
    
    // CAMBIO PRINCIPAL: El sistema puede operar mientras tenga al menos 1 sensor OK
    // Solo error crítico cuando ok_count == 0 (todos los sensores en bajo)
    g_sensor_check_flow.sensor_status.can_operate = (ok_count >= 1);
    
    Debug_Printf("SensorCheckFlow: Sensor check complete - OK:%d, Low:%02X, CanOperate:%d\r\n", 
                 ok_count, g_sensor_check_flow.sensor_status.low_sensors, g_sensor_check_flow.sensor_status.can_operate);
    
    Delay(1000);
    
    return (g_sensor_check_flow.sensor_status.low_sensors == 0);
}

void SensorCheckFlow_ShowSensorError(const sensor_status_t* sensor_status) {
    if (!lcd_is_initialized()) {
        Debug_Printf("SensorCheckFlow: LCD not initialized for sensor error display\r\n");
        return;
    }
    
    lcd_clear();
    
    if (sensor_status->can_operate) {
        // Caso: algunos sensores están bajos pero al menos 1 está OK
        lcd_set_cursor(0, 0);
        lcd_print("Niveles bajos:");
        
        // Show which sensors are low (only check first 3)
        char low_sensors_str[17] = {0};
        int pos = 0;
        
        for (int i = 0; i < 3; i++) {
            if (sensor_status->low_sensors & (1 << i)) {
                if (pos > 0) {
                    low_sensors_str[pos++] = ',';
                }
                low_sensors_str[pos++] = '1' + i;  // Sensor numbers 1-3
            }
        }
        
        lcd_set_cursor(1, 0);
        lcd_print(low_sensors_str);
        Debug_Printf("SensorCheckFlow: Showing sensor error - Low sensors: %s\r\n", low_sensors_str);
        
        Delay(1000);
    } else {
        // Caso: ERROR CRÍTICO - TODOS los sensores están en bajo (0 sensores OK)
        lcd_set_cursor(0, 0);
        lcd_print("ERROR CRITICO:");
        lcd_set_cursor(1, 0);
        lcd_print("Sin liquidos");
        Debug_Printf("SensorCheckFlow: Critical error - NO liquids available (all sensors low)\r\n");
        Delay(1000);
    }
}

operation_mode_t SensorCheckFlow_SelectMode(void) {
    if (!lcd_is_initialized()) {
        Debug_Printf("SensorCheckFlow: LCD not initialized for mode selection\r\n");
        return OPERATION_MODE_NORMAL;
    }
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("1-Normal");
    lcd_set_cursor(1, 0);
    lcd_print("2-Resanado");

    Debug_Printf("SensorCheckFlow: Waiting for mode selection...\r\n");

    char key = '\0';
    while (key != '1' && key != '2') {
        key = keypad_getkey();
        Debug_Printf("SensorCheckFlow: Key pressed: %c\r\n", key);

        if (key == '1') {
            Debug_Printf("SensorCheckFlow: Normal mode selected\r\n");
            return OPERATION_MODE_NORMAL;
        } else if (key == '2') {
            Debug_Printf("SensorCheckFlow: Refill mode selected\r\n");
            return OPERATION_MODE_REFILL;
        }
    }

    return OPERATION_MODE_NORMAL;
}

void SensorCheckFlow_ShowRefillInstructions(void) {
    if (!lcd_is_initialized()) {
        Debug_Printf("SensorCheckFlow: LCD not initialized for refill instructions\r\n");
        return;
    }
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Rellene faltantes");
    lcd_set_cursor(1, 0);
    lcd_print("Presione #");
    
    Debug_Printf("SensorCheckFlow: Waiting for refill completion...\r\n");
    
    char key = '\0';
    while (key != '#') {
        key = keypad_getkey();
        Debug_Printf("SensorCheckFlow: Key pressed during refill: %c\r\n", key);
    }
    
    Debug_Printf("SensorCheckFlow: Refill completed, proceeding to recheck\r\n");
}

void SensorCheckFlow_ShowReadyScreen(void) {
    if (!lcd_is_initialized()) {
        Debug_Printf("SensorCheckFlow: LCD not initialized for ready screen\r\n");
        return;
    }
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Listo!");
    lcd_set_cursor(1, 0);
    lcd_print("Presione *");

    Debug_Printf("SensorCheckFlow: System ready, waiting for start command...\r\n");

    char key = '\0';
    while (key != '*') {
        key = keypad_getkey();
        Debug_Printf("SensorCheckFlow: Key pressed at ready screen: %c\r\n", key);
    }

    Debug_Printf("SensorCheckFlow: Start command received\r\n");
}

void SensorCheckFlow_ShowError(const char* error_msg) {
    Debug_Printf("SensorCheckFlow: Showing error: %s\r\n", error_msg);
    
    if (!lcd_is_initialized()) {
        Debug_Printf("SensorCheckFlow: LCD not initialized for error display\r\n");
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
 * Getter Functions
 ******************************************************************************/

sensor_check_flow_state_t SensorCheckFlow_GetState(void) {
    return g_sensor_check_flow.current_state;
}

operation_mode_t SensorCheckFlow_GetMode(void) {
    return g_sensor_check_flow.selected_mode;
}

bool SensorCheckFlow_IsComplete(void) {
    return (g_sensor_check_flow.current_state == SENSOR_CHECK_STATE_COMPLETE);
}

const sensor_status_t* SensorCheckFlow_GetSensorStatus(void) {
    return &g_sensor_check_flow.sensor_status;
}

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

const char* SensorCheckFlow_StateToString(sensor_check_flow_state_t state) {
    switch (state) {
        case SENSOR_CHECK_STATE_INIT: return "INIT";
        case SENSOR_CHECK_STATE_CHECKING_SENSORS: return "CHECKING_SENSORS";
        case SENSOR_CHECK_STATE_SENSOR_ERROR: return "SENSOR_ERROR";
        case SENSOR_CHECK_STATE_MODE_SELECTION: return "MODE_SELECTION";
        case SENSOR_CHECK_STATE_REFILL_MODE: return "REFILL_MODE";
        case SENSOR_CHECK_STATE_WAITING_REFILL: return "WAITING_REFILL";
        case SENSOR_CHECK_STATE_RECHECK_SENSORS: return "RECHECK_SENSORS";
        case SENSOR_CHECK_STATE_READY: return "READY";
        case SENSOR_CHECK_STATE_ERROR: return "ERROR";
        case SENSOR_CHECK_STATE_COMPLETE: return "COMPLETE";
        default: return "UNKNOWN";
    }
}

/*******************************************************************************
 * Private Helper Functions
 ******************************************************************************/

static bool read_sensor(uint8_t sensor_id) {
    bool sensor_value = false;
    
    switch (sensor_id) {
        case 1:
            sensor_value = GPIO_ReadPinInput(SENSOR_1_GPIO, SENSOR_1_PIN);
            break;
        case 2:
            sensor_value = GPIO_ReadPinInput(SENSOR_2_GPIO, SENSOR_2_PIN);
            break;
        case 3:
            sensor_value = GPIO_ReadPinInput(SENSOR_3_GPIO, SENSOR_3_PIN);
            break;
        default:
            Debug_Printf("SensorCheckFlow: Invalid sensor ID: %d\r\n", sensor_id);
            return false;
    }
    
    Debug_Printf("SensorCheckFlow: Sensor %d = %s\r\n", sensor_id, sensor_value ? "HIGH" : "LOW");
    
    // Assuming sensors are active HIGH when liquid is present
    // If your sensors are active LOW, change this to: return !sensor_value;
    return sensor_value;
}

void SensorCheckFlow_Restart(void) {
    Debug_Printf("SensorCheckFlow: Restarting flow...\r\n");
    
    // Reset solo el estado, pero mantener hardware_initialized = true
    bool hw_init_status = g_sensor_check_flow.hardware_initialized;
    
    memset(&g_sensor_check_flow, 0, sizeof(sensor_check_flow_t));
    g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_INIT;
    g_sensor_check_flow.hardware_initialized = hw_init_status;
    
    Debug_Printf("SensorCheckFlow: Flow restarted\r\n");
}