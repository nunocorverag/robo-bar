/*
 * startup_flow.c
 * 
 * System Startup Flow Controller Implementation
 * FRDM-KL25Z Development Board
 */

#include "startup_flow.h"
#include "../drivers/lcd_i2c.h"
#include "../drivers/keypad.h"
#include "../config/gpio_config.h"
#include "../tasks/freertos_tasks.h"
#include <string.h>
#include <stdio.h>

/*******************************************************************************
 * Private Variables
 ******************************************************************************/
static startup_flow_t g_startup_flow;
static const char* SENSOR_NAMES[4] = {"Liquido 1", "Liquido 2", "Liquido 3", "Liquido 4"};

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/
static bool read_sensor(uint8_t sensor_id);
static void delay_ms(uint32_t ms);
static uint32_t get_tick_count(void);

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

bool StartupFlow_Init(void) {
    Debug_Printf("StartupFlow: Initializing...\r\n");
    
    // Initialize structure
    memset(&g_startup_flow, 0, sizeof(startup_flow_t));
    g_startup_flow.current_state = STARTUP_STATE_INIT;
    g_startup_flow.previous_state = STARTUP_STATE_INIT;
    
    // Initialize LCD and Keypad (sin valor de retorno)
    lcd_init(); 
    keypad_init();
    
    Debug_Printf("StartupFlow: Initialization complete\r\n");
    return true;
}


bool StartupFlow_Execute(void) {
    Debug_Printf("StartupFlow: Starting execution\r\n");
    
    g_startup_flow.total_startup_time = get_tick_count();
    
    while (!g_startup_flow.startup_complete && !g_startup_flow.error_occurred) {
        
        // Update state entry time on state change
        if (g_startup_flow.current_state != g_startup_flow.previous_state) {
            g_startup_flow.state_enter_time = get_tick_count();
            g_startup_flow.previous_state = g_startup_flow.current_state;
            Debug_Printf("StartupFlow: State changed to %s\r\n", 
                        StartupFlow_StateToString(g_startup_flow.current_state));
        }
        
        switch (g_startup_flow.current_state) {
            case STARTUP_STATE_INIT:
                g_startup_flow.current_state = STARTUP_STATE_INITIALIZING;
                break;
                
            case STARTUP_STATE_INITIALIZING:
                StartupFlow_ShowInitScreen();
                delay_ms(2000);  // Show init screen for 2 seconds
                g_startup_flow.current_state = STARTUP_STATE_CHECKING_SENSORS;
                break;
                
            case STARTUP_STATE_CHECKING_SENSORS:
                if (StartupFlow_CheckSensors()) {
                    g_startup_flow.current_state = STARTUP_STATE_MODE_SELECTION;
                } else {
                    g_startup_flow.current_state = STARTUP_STATE_SENSOR_ERROR;
                }
                break;
                
            case STARTUP_STATE_SENSOR_ERROR:
                StartupFlow_ShowSensorError(&g_startup_flow.sensor_status);
                if (g_startup_flow.sensor_status.can_operate) {
                    g_startup_flow.current_state = STARTUP_STATE_MODE_SELECTION;
                } else {
                    // Critical error - cannot operate
                    StartupFlow_ShowError("Sin liquidos!");
                    g_startup_flow.error_occurred = true;
                }
                break;
                
            case STARTUP_STATE_MODE_SELECTION:
                g_startup_flow.selected_mode = StartupFlow_SelectMode();
                if (g_startup_flow.selected_mode == OPERATION_MODE_NORMAL) {
                    g_startup_flow.current_state = STARTUP_STATE_READY;
                } else if (g_startup_flow.selected_mode == OPERATION_MODE_REFILL) {
                    g_startup_flow.current_state = STARTUP_STATE_REFILL_MODE;
                } else {
                    // Invalid selection, stay in mode selection
                    continue;
                }
                break;
                
            case STARTUP_STATE_REFILL_MODE:
                StartupFlow_ShowRefillInstructions();
                g_startup_flow.current_state = STARTUP_STATE_WAITING_REFILL;
                break;
                
            case STARTUP_STATE_WAITING_REFILL:
                // Wait for # key press (handled in ShowRefillInstructions)
                g_startup_flow.current_state = STARTUP_STATE_RECHECK_SENSORS;
                break;
                
            case STARTUP_STATE_RECHECK_SENSORS:
                if (StartupFlow_CheckSensors()) {
                    g_startup_flow.current_state = STARTUP_STATE_READY;
                } else {
                    g_startup_flow.current_state = STARTUP_STATE_SENSOR_ERROR;
                }
                break;
                
            case STARTUP_STATE_READY:
                StartupFlow_ShowReadyScreen();
                g_startup_flow.current_state = STARTUP_STATE_COMPLETE;
                break;
                
            case STARTUP_STATE_COMPLETE:
                g_startup_flow.startup_complete = true;
                break;
                
            case STARTUP_STATE_ERROR:
            default:
                g_startup_flow.error_occurred = true;
                break;
        }
        
        // Small delay to prevent tight loop
        delay_ms(10);
    }
    
    g_startup_flow.total_startup_time = get_tick_count() - g_startup_flow.total_startup_time;
    
    if (g_startup_flow.startup_complete) {
        Debug_Printf("StartupFlow: Completed successfully in %lu ms\r\n", 
                    g_startup_flow.total_startup_time);
        return true;
    } else {
        Debug_Printf("StartupFlow: Failed with error\r\n");
        return false;
    }
}

void StartupFlow_Task(void *pvParameters) {
    Debug_Printf("StartupFlow: Task started\r\n");
    
    // Initialize startup flow
    if (!StartupFlow_Init()) {
        Debug_Printf("StartupFlow: Task initialization failed\r\n");
        LED_SetColor(true, false, false);  // Red error
        vTaskDelete(NULL);
        return;
    }
    
    // Execute startup flow
    if (StartupFlow_Execute()) {
        Debug_Printf("StartupFlow: Task completed successfully\r\n");
        LED_SetColor(false, true, false);  // Green success
        
        // Signal other tasks that startup is complete
        if (xEventGroupSystem) {
            xEventGroupSetBits(xEventGroupSystem, (1 << 1));  // Startup complete flag
        }
    } else {
        Debug_Printf("StartupFlow: Task failed\r\n");
        LED_SetColor(true, false, false);  // Red error
    }
    
    // Task complete, delete self
    vTaskDelete(NULL);
}

/*******************************************************************************
 * Screen Display Functions
 ******************************************************************************/

void StartupFlow_ShowInitScreen(void) {
    lcd_clear();
    lcd_set_cursor(0, 0);  // Primera línea
    lcd_print("Robo-Bar v1.0");
    lcd_set_cursor(1, 0);  // Segunda línea
    lcd_print("Inicializando...");
    Debug_Printf("StartupFlow: Showing initialization screen\r\n");
}

bool StartupFlow_CheckSensors(void) {
    lcd_clear();
    lcd_set_cursor(0, 0);  // Primera línea
    lcd_print("Verificando...");
    lcd_set_cursor(1, 0);  // Segunda línea
    lcd_print("Sensores");
    
    Debug_Printf("StartupFlow: Checking sensors...\r\n");
    
    // Reset sensor status
    memset(&g_startup_flow.sensor_status, 0, sizeof(sensor_status_t));
    
    // Check each sensor
    g_startup_flow.sensor_status.sensor_1_ok = read_sensor(1);
    g_startup_flow.sensor_status.sensor_2_ok = read_sensor(2);
    g_startup_flow.sensor_status.sensor_3_ok = read_sensor(3);
    g_startup_flow.sensor_status.sensor_4_ok = read_sensor(4);
    
    // Create bitmask of low sensors
    if (!g_startup_flow.sensor_status.sensor_1_ok) g_startup_flow.sensor_status.low_sensors |= (1 << 0);
    if (!g_startup_flow.sensor_status.sensor_2_ok) g_startup_flow.sensor_status.low_sensors |= (1 << 1);
    if (!g_startup_flow.sensor_status.sensor_3_ok) g_startup_flow.sensor_status.low_sensors |= (1 << 2);
    if (!g_startup_flow.sensor_status.sensor_4_ok) g_startup_flow.sensor_status.low_sensors |= (1 << 3);
    
    // Check if system can operate (at least 2 sensors OK)
    uint8_t ok_count = 0;
    if (g_startup_flow.sensor_status.sensor_1_ok) ok_count++;
    if (g_startup_flow.sensor_status.sensor_2_ok) ok_count++;
    if (g_startup_flow.sensor_status.sensor_3_ok) ok_count++;
    if (g_startup_flow.sensor_status.sensor_4_ok) ok_count++;
    
    g_startup_flow.sensor_status.can_operate = (ok_count >= 2);
    
    Debug_Printf("StartupFlow: Sensor check complete - OK:%d, Low:%02X, CanOperate:%d\r\n",
                ok_count, g_startup_flow.sensor_status.low_sensors, 
                g_startup_flow.sensor_status.can_operate);
    
    delay_ms(1000);  // Show checking message
    
    return (g_startup_flow.sensor_status.low_sensors == 0);
}

void StartupFlow_ShowSensorError(const sensor_status_t* sensor_status) {
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
        Debug_Printf("StartupFlow: Showing sensor error - Low sensors: %s\r\n", low_sensors_str);
        
        delay_ms(3000);  // Show error for 3 seconds
    } else {
        lcd_set_cursor(0, 0);  // Primera línea
        lcd_print("ERROR CRITICO:");
        lcd_set_cursor(1, 0);  // Segunda línea
        lcd_print("Muy pocos liquidos");
        Debug_Printf("StartupFlow: Critical error - insufficient liquids\r\n");
        delay_ms(3000);
    }
}

operation_mode_t StartupFlow_SelectMode(void) {
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("1-Normal");
    lcd_set_cursor(1, 0);
    lcd_print("2-Resanado");

    Debug_Printf("StartupFlow: Waiting for mode selection...\r\n");

    char key = '\0';
    while (key != '1' && key != '2') {
        key = keypad_getkey();  // Espera una tecla
        Debug_Printf("StartupFlow: Key pressed: %c\r\n", key);

        if (key == '1') {
            Debug_Printf("StartupFlow: Normal mode selected\r\n");
            return OPERATION_MODE_NORMAL;
        } else if (key == '2') {
            Debug_Printf("StartupFlow: Refill mode selected\r\n");
            return OPERATION_MODE_REFILL;
        }
    }

    return OPERATION_MODE_NORMAL;  // Fallback por defecto
}


void StartupFlow_ShowRefillInstructions(void) {
    lcd_clear();
    lcd_set_cursor(0, 0);  // Primera línea
    lcd_print("Rellene faltantes");
    lcd_set_cursor(1, 0);  // Segunda línea
    lcd_print("Presione #");
    
    Debug_Printf("StartupFlow: Waiting for refill completion...\r\n");
    
    char key = '\0';
    while (key != '#') {
        key = keypad_getkey();  // Espera la tecla presionada
        Debug_Printf("StartupFlow: Key pressed during refill: %c\r\n", key);
    }
    
    Debug_Printf("StartupFlow: Refill completed, proceeding to recheck\r\n");
}


void StartupFlow_ShowReadyScreen(void) {
    lcd_clear();  // Limpiar la pantalla
    lcd_set_cursor(0, 0);  // Establecer el cursor en la primera línea
    lcd_print("Listo!");  // Mostrar el mensaje "Listo!" en la pantalla
    lcd_set_cursor(1, 0);  // Establecer el cursor en la segunda línea
    lcd_print("Presione *");  // Mostrar el mensaje "Presione *"

    Debug_Printf("StartupFlow: System ready, waiting for start command...\r\n");

    char key = '\0';
    while (key != '*') {
        key = keypad_getkey();  // Esperar a que se presione una tecla
        Debug_Printf("StartupFlow: Key pressed at ready screen: %c\r\n", key);
    }

    Debug_Printf("StartupFlow: Start command received\r\n");
}


void StartupFlow_ShowError(const char* error_msg) {
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("ERROR:");
    lcd_set_cursor(1, 0);
    lcd_print(error_msg);

    Debug_Printf("StartupFlow: Error displayed: %s\r\n", error_msg);

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

startup_flow_state_t StartupFlow_GetState(void) {
    return g_startup_flow.current_state;
}

operation_mode_t StartupFlow_GetMode(void) {
    return g_startup_flow.selected_mode;
}

bool StartupFlow_IsComplete(void) {
    return g_startup_flow.startup_complete;
}

const sensor_status_t* StartupFlow_GetSensorStatus(void) {
    return &g_startup_flow.sensor_status;
}

void StartupFlow_Restart(void) {
    Debug_Printf("StartupFlow: Restarting...\r\n");
    memset(&g_startup_flow, 0, sizeof(startup_flow_t));
    g_startup_flow.current_state = STARTUP_STATE_INIT;
}

const char* StartupFlow_StateToString(startup_flow_state_t state) {
    switch (state) {
        case STARTUP_STATE_INIT: return "INIT";
        case STARTUP_STATE_INITIALIZING: return "INITIALIZING";
        case STARTUP_STATE_CHECKING_SENSORS: return "CHECKING_SENSORS";
        case STARTUP_STATE_SENSOR_ERROR: return "SENSOR_ERROR";
        case STARTUP_STATE_MODE_SELECTION: return "MODE_SELECTION";
        case STARTUP_STATE_REFILL_MODE: return "REFILL_MODE";
        case STARTUP_STATE_WAITING_REFILL: return "WAITING_REFILL";
        case STARTUP_STATE_RECHECK_SENSORS: return "RECHECK_SENSORS";
        case STARTUP_STATE_READY: return "READY";
        case STARTUP_STATE_ERROR: return "ERROR";
        case STARTUP_STATE_COMPLETE: return "COMPLETE";
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
            Debug_Printf("StartupFlow: Invalid sensor ID: %d\r\n", sensor_id);
            return false;
    }
    
    Debug_Printf("StartupFlow: Sensor %d = %s\r\n", sensor_id, sensor_value ? "HIGH" : "LOW");
    
    // Assuming sensors are active HIGH when liquid is present
    // If your sensors are active LOW, change this to: return !sensor_value;
    return sensor_value;
}

static void delay_ms(uint32_t ms) {
    // Use FreeRTOS delay
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static uint32_t get_tick_count(void) {
    // Return FreeRTOS tick count in milliseconds
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}