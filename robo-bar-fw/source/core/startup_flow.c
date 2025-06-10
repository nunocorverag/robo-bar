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

// Reemplaza la función StartupFlow_Init() en tu startup_flow.c

bool StartupFlow_Init(void) {
    Debug_Printf("StartupFlow: Initializing...\r\n");
    
    // Initialize structure
    memset(&g_startup_flow, 0, sizeof(startup_flow_t));
    g_startup_flow.current_state = STARTUP_STATE_INIT;
    g_startup_flow.previous_state = STARTUP_STATE_INIT;
    
    // Initialize I2C
    Debug_Printf("Initializing I2C...\r\n");
    i2c_init();
    Debug_Printf("I2C Initialized.\r\n");

    // Initialize LCD and Keypad (sin valor de retorno)
    Debug_Printf("Initializing LCD...\r\n");
    if (!lcd_init()) {
        Debug_Printf("ERROR: LCD initialization failed!\r\n");
        return false; // Retornar error si el LCD falla
    }
    Debug_Printf("LCD Initialized successfully.\r\n");

    // Initialize Keypad
    Debug_Printf("Initializing Keypad...\r\n");
    keypad_init();
    Debug_Printf("Keypad Initialized.\r\n");
    
    Debug_Printf("StartupFlow: Initialization complete\r\n");
    return true;
}

// Reemplaza todas las funciones de pantalla para manejar errores:

void StartupFlow_ShowInitScreen(void) {
    if (!lcd_clear()) {
        Debug_Printf("ERROR: LCD clear failed in ShowInitScreen\r\n");
        return;
    }
    
    if (!lcd_set_cursor(0, 0)) {
        Debug_Printf("ERROR: LCD set_cursor failed in ShowInitScreen\r\n");
        return;
    }
    
    if (!lcd_print("Robo-Bar v1.0")) {
        Debug_Printf("ERROR: LCD print failed in ShowInitScreen\r\n");
        return;
    }
    
    if (!lcd_set_cursor(1, 0)) {
        Debug_Printf("ERROR: LCD set_cursor failed in ShowInitScreen (line 2)\r\n");
        return;
    }
    
    if (!lcd_print("Inicializando...")) {
        Debug_Printf("ERROR: LCD print failed in ShowInitScreen (line 2)\r\n");
        return;
    }
    
    Debug_Printf("StartupFlow: Showing initialization screen\r\n");
}

bool StartupFlow_CheckSensors(void) {
    if (!lcd_clear()) {
        Debug_Printf("ERROR: LCD clear failed in CheckSensors\r\n");
    }

    if (!lcd_set_cursor(0, 0) || !lcd_print("Verificando...")) {
        Debug_Printf("ERROR: LCD operation failed in CheckSensors (line 1)\r\n");
    }

    if (!lcd_set_cursor(1, 0) || !lcd_print("Sensores")) {
        Debug_Printf("ERROR: LCD operation failed in CheckSensors (line 2)\r\n");
    }
    
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
    if (!lcd_clear()) {
        Debug_Printf("ERROR: LCD clear failed in ShowSensorError\r\n");
        return;
    }
    
    if (sensor_status->can_operate) {
        if (!lcd_set_cursor(0, 0) || !lcd_print("Niveles bajos:")) {
            Debug_Printf("ERROR: LCD operation failed in ShowSensorError\r\n");
            return;
        }
        
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
        
        if (!lcd_set_cursor(1, 0) || !lcd_print(low_sensors_str)) {
            Debug_Printf("ERROR: LCD operation failed showing low sensors\r\n");
        }

        Debug_Printf("StartupFlow: Showing sensor error - Low sensors: %s\r\n", low_sensors_str);
        delay_ms(3000);  // Show error for 3 seconds
    } else {
        if (!lcd_set_cursor(0, 0) || !lcd_print("ERROR CRITICO:")) {
            Debug_Printf("ERROR: LCD operation failed in critical error (line 1)\r\n");
        }

        if (!lcd_set_cursor(1, 0) || !lcd_print("Muy pocos liquidos")) {
            Debug_Printf("ERROR: LCD operation failed in critical error (line 2)\r\n");
        }

        Debug_Printf("StartupFlow: Critical error - insufficient liquids\r\n");
        delay_ms(3000);
    }
}

operation_mode_t StartupFlow_SelectMode(void) {
    if (!lcd_clear()) {
        Debug_Printf("ERROR: LCD clear failed in SelectMode\r\n");
    }

    if (!lcd_set_cursor(0, 0) || !lcd_print("1-Normal")) {
        Debug_Printf("ERROR: LCD operation failed in SelectMode (line 1)\r\n");
    }

    if (!lcd_set_cursor(1, 0) || !lcd_print("2-Rellenado")) {
        Debug_Printf("ERROR: LCD operation failed in SelectMode (line 2)\r\n");
    }

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
    if (!lcd_clear()) {
        Debug_Printf("ERROR: LCD clear failed in ShowRefillInstructions\r\n");
        return;
    }

    if (!lcd_set_cursor(0, 0) || !lcd_print("Rellene faltantes")) {
        Debug_Printf("ERROR: LCD operation failed in ShowRefillInstructions (line 1)\r\n");
    }

    if (!lcd_set_cursor(1, 0) || !lcd_print("Presione #")) {
        Debug_Printf("ERROR: LCD operation failed in ShowRefillInstructions (line 2)\r\n");
    }
    
    Debug_Printf("StartupFlow: Waiting for refill completion...\r\n");
    
    char key = '\0';
    while (key != '#') {
        key = keypad_getkey();  // Espera la tecla presionada
        Debug_Printf("StartupFlow: Key pressed during refill: %c\r\n", key);
    }
    
    Debug_Printf("StartupFlow: Refill completed, proceeding to recheck\r\n");
}

void StartupFlow_ShowReadyScreen(void) {
    if (!lcd_clear()) {
        Debug_Printf("ERROR: LCD clear failed in ShowReadyScreen\r\n");
        return;
    }

    if (!lcd_set_cursor(0, 0) || !lcd_print("Listo!")) {
        Debug_Printf("ERROR: LCD operation failed in ShowReadyScreen (line 1)\r\n");
    }

    if (!lcd_set_cursor(1, 0) || !lcd_print("Presione *")) {
        Debug_Printf("ERROR: LCD operation failed in ShowReadyScreen (line 2)\r\n");
    }

    Debug_Printf("StartupFlow: System ready, waiting for start command...\r\n");

    char key = '\0';
    while (key != '*') {
        key = keypad_getkey();  // Esperar a que se presione una tecla
        Debug_Printf("StartupFlow: Key pressed at ready screen: %c\r\n", key);
    }

    Debug_Printf("StartupFlow: Start command received\r\n");
}

void StartupFlow_ShowError(const char* error_msg) {
    if (!lcd_clear()) {
        Debug_Printf("ERROR: LCD clear failed in ShowError\r\n");
        return;
    }

    if (!lcd_set_cursor(0, 0) || !lcd_print("ERROR:")) {
        Debug_Printf("ERROR: LCD operation failed in ShowError (line 1)\r\n");
    }

    if (!lcd_set_cursor(1, 0) || !lcd_print(error_msg)) {
        Debug_Printf("ERROR: LCD operation failed in ShowError (line 2)\r\n");
    }

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
