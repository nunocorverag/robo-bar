/*
 * sensor_check_flow.c
 * 
 * Sensor Check and Mode Selection Flow Controller Implementation (FIXED)
 * FRDM-KL25Z Development Board
 */

#include "sensor_check_flow.h"
#include "../drivers/lcd_i2c.h"
#include "../drivers/keypad.h"
#include "../config/gpio_config.h"
#include <string.h>
#include <stdio.h>

/*******************************************************************************
 * Private Variables
 ******************************************************************************/
static sensor_check_flow_t g_sensor_check_flow;
static const char* SENSOR_NAMES[4] = {"Liquido 1", "Liquido 2", "Liquido 3", "Liquido 4"};

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/
static bool read_sensor(uint8_t sensor_id);
static void delay_ms(uint32_t ms);

void SensorCheckFlow_Run(void) {
    if (!SensorCheckFlow_Init()) {
        SensorCheckFlow_ShowError("Init falló");
        LED_SetColor(true, false, false);  // LED rojo
        return;
    }

    if (SensorCheckFlow_Execute()) {
        LED_SetColor(false, true, false);  // LED verde
    } else {
        SensorCheckFlow_ShowError("Error ejecución");
        LED_SetColor(true, false, false);  // LED rojo
    }
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

bool SensorCheckFlow_Init(void) {
    Debug_Printf("SensorCheckFlow: Initializing...\r\n");
    
    // Initialize structure
    memset(&g_sensor_check_flow, 0, sizeof(sensor_check_flow_t));
    g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_INIT;
    g_sensor_check_flow.previous_state = SENSOR_CHECK_STATE_INIT;
    
    Debug_Printf("SensorCheckFlow: Initialization complete\r\n");
    return true;
}

bool SensorCheckFlow_Execute(void) {
    Debug_Printf("SensorCheckFlow: Starting execution\r\n");
    
    
    while (!g_sensor_check_flow.check_complete && !g_sensor_check_flow.error_occurred) {
        
        // Update state entry time on state change
        if (g_sensor_check_flow.current_state != g_sensor_check_flow.previous_state) {
            g_sensor_check_flow.previous_state = g_sensor_check_flow.current_state;
            Debug_Printf("SensorCheckFlow: State changed to %s\r\n", SensorCheckFlow_StateToString(g_sensor_check_flow.current_state));
        }
        
        switch (g_sensor_check_flow.current_state) {
            case SENSOR_CHECK_STATE_INIT:
                g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_CHECKING_SENSORS;
                break;
                
            case SENSOR_CHECK_STATE_CHECKING_SENSORS:
                if (SensorCheckFlow_CheckSensors()) {
                    g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_MODE_SELECTION;
                } else {
                    g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_SENSOR_ERROR;
                }
                break;
                
            case SENSOR_CHECK_STATE_SENSOR_ERROR:
                SensorCheckFlow_ShowSensorError(&g_sensor_check_flow.sensor_status);
                if (g_sensor_check_flow.sensor_status.can_operate) {
                    g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_MODE_SELECTION;
                } else {
                    SensorCheckFlow_ShowRefillInstructions();  // Instrucciones de recarga
                    g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_WAITING_REFILL;  // Estado de espera para relleno
                }
                break;
                
            case SENSOR_CHECK_STATE_MODE_SELECTION:
                g_sensor_check_flow.selected_mode = SensorCheckFlow_SelectMode();
                if (g_sensor_check_flow.selected_mode == OPERATION_MODE_NORMAL) {
                    g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_READY;
                } else if (g_sensor_check_flow.selected_mode == OPERATION_MODE_REFILL) {
                    g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_REFILL_MODE;
                } else {
                    // Invalid selection, stay in mode selection
                    continue;
                }
                break;
                
            case SENSOR_CHECK_STATE_REFILL_MODE:
                SensorCheckFlow_ShowRefillInstructions();
                g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_WAITING_REFILL;
                break;
                
            case SENSOR_CHECK_STATE_WAITING_REFILL:
                // Wait for # key press (handled in ShowRefillInstructions)
                g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_RECHECK_SENSORS;
                break;
                
            case SENSOR_CHECK_STATE_RECHECK_SENSORS:
                if (SensorCheckFlow_CheckSensors()) {
                    g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_READY;
                } else {
                    g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_SENSOR_ERROR;
                }
                break;
                
            case SENSOR_CHECK_STATE_READY:
                SensorCheckFlow_ShowReadyScreen();
                g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_COMPLETE;
                break;
                
            case SENSOR_CHECK_STATE_COMPLETE:
                g_sensor_check_flow.check_complete = true;
                break;
                
            case SENSOR_CHECK_STATE_ERROR:
            default:
                g_sensor_check_flow.error_occurred = true;
                break;
        }
        
        // Small delay to prevent tight loop
        delay_ms(10);
    }
        
    if (g_sensor_check_flow.check_complete) {
        Debug_Printf("SensorCheckFlow: Completed successfully");
        return true;
    } else {
        Debug_Printf("SensorCheckFlow: Failed with error\r\n");
        return false;
    }
}

/*******************************************************************************
 * Screen Display Functions
 ******************************************************************************/

bool SensorCheckFlow_CheckSensors(void) {
    lcd_clear();
    lcd_set_cursor(0, 0);  // Primera línea
    lcd_print("Verificando...");
    lcd_set_cursor(1, 0);  // Segunda línea
    lcd_print("Sensores");
    
    Debug_Printf("SensorCheckFlow: Checking sensors...\r\n");
    
    // Reset sensor status
    memset(&g_sensor_check_flow.sensor_status, 0, sizeof(sensor_status_t));
    
    // Check each sensor
    g_sensor_check_flow.sensor_status.sensor_1_ok = read_sensor(1);
    g_sensor_check_flow.sensor_status.sensor_2_ok = read_sensor(2);
    g_sensor_check_flow.sensor_status.sensor_3_ok = read_sensor(3);
    g_sensor_check_flow.sensor_status.sensor_4_ok = read_sensor(4);
    
    // Create bitmask of low sensors
    if (!g_sensor_check_flow.sensor_status.sensor_1_ok) g_sensor_check_flow.sensor_status.low_sensors |= (1 << 0);
    if (!g_sensor_check_flow.sensor_status.sensor_2_ok) g_sensor_check_flow.sensor_status.low_sensors |= (1 << 1);
    if (!g_sensor_check_flow.sensor_status.sensor_3_ok) g_sensor_check_flow.sensor_status.low_sensors |= (1 << 2);
    if (!g_sensor_check_flow.sensor_status.sensor_4_ok) g_sensor_check_flow.sensor_status.low_sensors |= (1 << 3);
    
    // Check if system can operate (at least 2 sensors OK)
    uint8_t ok_count = 0;
    if (g_sensor_check_flow.sensor_status.sensor_1_ok) ok_count++;
    if (g_sensor_check_flow.sensor_status.sensor_2_ok) ok_count++;
    if (g_sensor_check_flow.sensor_status.sensor_3_ok) ok_count++;
    if (g_sensor_check_flow.sensor_status.sensor_4_ok) ok_count++;
    
    g_sensor_check_flow.sensor_status.can_operate = (ok_count >= 2);
    
    Debug_Printf("SensorCheckFlow: Sensor check complete - OK:%d, Low:%02X, CanOperate:%d\r\n", ok_count, g_sensor_check_flow.sensor_status.low_sensors, g_sensor_check_flow.sensor_status.can_operate);
    
    delay_ms(1000);  // Show checking message
    
    return (g_sensor_check_flow.sensor_status.low_sensors == 0);
}

void SensorCheckFlow_ShowSensorError(const sensor_status_t* sensor_status) {
    lcd_clear();
    
    if (sensor_status->can_operate) {
        lcd_set_cursor(0, 0);  // Primera línea
        lcd_print("Niveles bajos:");
        
        // Show which sensors are low
        char low_sensors_str[17] = {0};
        int pos = 0;
        
        for (int i = 0; i < 4; i++) {
            if (sensor_status->low_sensors & (1 << i)) {
                if (pos > 0) {
                    low_sensors_str[pos++] = ',';
                }
                low_sensors_str[pos++] = '1' + i;  // Sensor numbers 1-4
            }
        }
        
        lcd_set_cursor(1, 0);  // Segunda línea
        lcd_print(low_sensors_str);
        Debug_Printf("SensorCheckFlow: Showing sensor error - Low sensors: %s\r\n", low_sensors_str);
        
        delay_ms(3000);  // Show error for 3 seconds
    } else {
        lcd_set_cursor(0, 0);  // Primera línea
        lcd_print("ERROR CRITICO:");
        lcd_set_cursor(1, 0);  // Segunda línea
        lcd_print("Poco liquido");
        Debug_Printf("SensorCheckFlow: Critical error - insufficient liquids\r\n");
        delay_ms(3000);
    }
}

operation_mode_t SensorCheckFlow_SelectMode(void) {
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("1-Normal");
    lcd_set_cursor(1, 0);
    lcd_print("2-Resanado");

    Debug_Printf("SensorCheckFlow: Waiting for mode selection...\r\n");

    char key = '\0';
    while (key != '1' && key != '2') {
        key = keypad_getkey();  // Espera una tecla
        Debug_Printf("SensorCheckFlow: Key pressed: %c\r\n", key);

        if (key == '1') {
            Debug_Printf("SensorCheckFlow: Normal mode selected\r\n");
            return OPERATION_MODE_NORMAL;
        } else if (key == '2') {
            Debug_Printf("SensorCheckFlow: Refill mode selected\r\n");
            return OPERATION_MODE_REFILL;
        }
    }

    return OPERATION_MODE_NORMAL;  // Fallback por defecto
}

void SensorCheckFlow_ShowRefillInstructions(void) {
    lcd_clear();
    lcd_set_cursor(0, 0);  // Primera línea
    lcd_print("Rellene faltantes");
    lcd_set_cursor(1, 0);  // Segunda línea
    lcd_print("Presione #");
    
    Debug_Printf("SensorCheckFlow: Waiting for refill completion...\r\n");
    
    char key = '\0';
    while (key != '#') {
        key = keypad_getkey();  // Espera la tecla presionada
        Debug_Printf("SensorCheckFlow: Key pressed during refill: %c\r\n", key);
    }
    
    Debug_Printf("SensorCheckFlow: Refill completed, proceeding to recheck\r\n");
}

void SensorCheckFlow_ShowReadyScreen(void) {
    lcd_clear();  // Limpiar la pantalla
    lcd_set_cursor(0, 0);  // Establecer el cursor en la primera línea
    lcd_print("Listo!");  // Mostrar el mensaje "Listo!" en la pantalla
    lcd_set_cursor(1, 0);  // Establecer el cursor en la segunda línea
    lcd_print("Presione *");  // Mostrar el mensaje "Presione *"

    Debug_Printf("SensorCheckFlow: System ready, waiting for start command...\r\n");

    char key = '\0';
    while (key != '*') {
        key = keypad_getkey();  // Esperar a que se presione una tecla
        Debug_Printf("SensorCheckFlow: Key pressed at ready screen: %c\r\n", key);
    }

    Debug_Printf("SensorCheckFlow: Start command received\r\n");
}

void SensorCheckFlow_ShowError(const char* error_msg) {
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("ERROR:");
    lcd_set_cursor(1, 0);
    lcd_print(error_msg);

    Debug_Printf("SensorCheckFlow: Error displayed: %s\r\n", error_msg);

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

sensor_check_flow_state_t SensorCheckFlow_GetState(void) {
    return g_sensor_check_flow.current_state;
}

operation_mode_t SensorCheckFlow_GetMode(void) {
    return g_sensor_check_flow.selected_mode;
}

bool SensorCheckFlow_IsComplete(void) {
    return g_sensor_check_flow.check_complete;
}

const sensor_status_t* SensorCheckFlow_GetSensorStatus(void) {
    return &g_sensor_check_flow.sensor_status;
}

void SensorCheckFlow_Restart(void) {
    Debug_Printf("SensorCheckFlow: Restarting...\r\n");
    memset(&g_sensor_check_flow, 0, sizeof(sensor_check_flow_t));
    g_sensor_check_flow.current_state = SENSOR_CHECK_STATE_INIT;
}

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
        case 4:
            sensor_value = GPIO_ReadPinInput(SENSOR_4_GPIO, SENSOR_4_PIN);
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

void delay_ms(uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms * 4000; i++) {
        __NOP();
    }
}