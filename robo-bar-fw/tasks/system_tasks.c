/*
 * system_tasks.c
 * 
 * FreeRTOS task implementations for Robo-Bar project
 * FRDM-KL25Z Development Board
 * 
 * Implements all system tasks including:
 * - Servo control and liquid dispensing
 * - Sensor monitoring and data acquisition
 * - User interface (keypad and LCD)
 * - Motor control (conveyor and mixing)
 * - System monitoring and diagnostics
 * - UART communication and debugging
 */

#include "system_tasks.h"
#include "gpio_config.h"
#include "pwm_config.h"
#include "uart_config.h"
#include "lcd_config.h"
#include "keypad_config.h"
#include <string.h>
#include <stdio.h>

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

/* Task Handles */
TaskHandle_t xTaskHandleEmergency = NULL;
TaskHandle_t xTaskHandleServoControl = NULL;
TaskHandle_t xTaskHandleSensorMonitor = NULL;
TaskHandle_t xTaskHandleMotorControl = NULL;
TaskHandle_t xTaskHandleUserInterface = NULL;
TaskHandle_t xTaskHandleSystemMonitor = NULL;
TaskHandle_t xTaskHandleUartComm = NULL;
TaskHandle_t xTaskHandleLedStatus = NULL;

/* Queues */
QueueHandle_t xQueueServoCommands = NULL;
QueueHandle_t xQueueMotorCommands = NULL;
QueueHandle_t xQueueSensorData = NULL;
QueueHandle_t xQueueUserInput = NULL;
QueueHandle_t xQueueSystemMessages = NULL;
QueueHandle_t xQueueDebugOutput = NULL;

/* Semaphores */
SemaphoreHandle_t xSemaphoreI2C = NULL;
SemaphoreHandle_t xSemaphoreUART = NULL;
SemaphoreHandle_t xSemaphoreSensorData = NULL;

/* Event Groups */
EventGroupHandle_t xEventGroupSystem = NULL;

/* Timers */
TimerHandle_t xTimerSystemHeartbeat = NULL;
TimerHandle_t xTimerCleaningReminder = NULL;

/* System State Variables */
volatile system_state_t g_system_state = SYSTEM_STATE_INIT;
volatile system_error_t g_last_error = SYSTEM_ERROR_NONE;
volatile uint32_t g_system_uptime_seconds = 0;

/* Dispensing Context */
static dispense_context_t g_dispense_context = {
    .state = DISPENSE_STATE_IDLE,
    .current_ingredient = 0,
    .dispense_progress_ms = 0,
    .start_time = 0,
    .active_recipe = NULL
};

/* Recipe Storage */
static drink_recipe_t g_recipes[MAX_RECIPES];
static uint8_t g_recipe_count = 0;

/* System Statistics */
static struct {
    uint32_t emergency_stops;
    uint32_t recipes_completed;
    uint32_t sensor_alarms;
    uint32_t motor_faults;
    uint32_t communication_errors;
} g_system_stats = {0};

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/
static void system_init_hardware(void);
static void system_init_default_recipes_internal(void);
static bool system_validate_recipe(const drink_recipe_t *recipe);
static void system_update_led_status(void);
static void system_process_emergency_stop(void);
static void system_handle_servo_timeout(uint8_t servo_id);

/*******************************************************************************
 * Task Initialization Functions
 ******************************************************************************/

bool system_tasks_init(void)
{
    BaseType_t xResult;
    bool init_success = true;
    
    TASK_INFO_PRINT("Initializing FreeRTOS tasks and synchronization objects...");
    
    /* Initialize hardware first */
    system_init_hardware();
    
    /* Create Queues */
    xQueueServoCommands = xQueueCreate(QUEUE_LENGTH_SERVO_COMMANDS, sizeof(system_message_t));
    xQueueMotorCommands = xQueueCreate(QUEUE_LENGTH_MOTOR_COMMANDS, sizeof(system_message_t));
    xQueueSensorData = xQueueCreate(QUEUE_LENGTH_SENSOR_DATA, sizeof(system_message_t));
    xQueueUserInput = xQueueCreate(QUEUE_LENGTH_USER_INPUT, sizeof(system_message_t));
    xQueueSystemMessages = xQueueCreate(QUEUE_LENGTH_SYSTEM_MESSAGES, sizeof(system_message_t));
    xQueueDebugOutput = xQueueCreate(QUEUE_LENGTH_DEBUG_OUTPUT, sizeof(system_message_t));
    
    if (!xQueueServoCommands || !xQueueMotorCommands || !xQueueSensorData ||
        !xQueueUserInput || !xQueueSystemMessages || !xQueueDebugOutput) {
        TASK_ERROR_PRINT("Failed to create queues");
        init_success = false;
    }
    
    /* Create Semaphores */
    xSemaphoreI2C = xSemaphoreCreateMutex();
    xSemaphoreUART = xSemaphoreCreateMutex();
    xSemaphoreSensorData = xSemaphoreCreateMutex();
    
    if (!xSemaphoreI2C || !xSemaphoreUART || !xSemaphoreSensorData) {
        TASK_ERROR_PRINT("Failed to create semaphores");
        init_success = false;
    }
    
    /* Create Event Group */
    xEventGroupSystem = xEventGroupCreate();
    if (!xEventGroupSystem) {
        TASK_ERROR_PRINT("Failed to create event group");
        init_success = false;
    }
    
    /* Create Timers */
    xTimerSystemHeartbeat = xTimerCreate("HeartbeatTimer", 
                                        pdMS_TO_TICKS(SYSTEM_HEARTBEAT_INTERVAL_MS),
                                        pdTRUE, /* Auto-reload */
                                        (void*)0,
                                        vTimerCallbackSystemHeartbeat);
    
    xTimerCleaningReminder = xTimerCreate("CleaningTimer",
                                         pdMS_TO_TICKS(CLEANING_REMINDER_INTERVAL_MS),
                                         pdTRUE, /* Auto-reload */
                                         (void*)0,
                                         vTimerCallbackCleaningReminder);
    
    if (!xTimerSystemHeartbeat || !xTimerCleaningReminder) {
        TASK_ERROR_PRINT("Failed to create timers");
        init_success = false;
    }
    
    /* Create Tasks */
    xResult = xTaskCreate(vTaskEmergencyMonitor, "Emergency", 
                         TASK_STACK_SIZE_EMERGENCY, NULL, 
                         TASK_PRIORITY_EMERGENCY, &xTaskHandleEmergency);
    if (xResult != pdPASS) {
        TASK_ERROR_PRINT("Failed to create Emergency task");
        init_success = false;
    }
    
    xResult = xTaskCreate(vTaskServoControl, "ServoCtrl", 
                         TASK_STACK_SIZE_SERVO_CONTROL, NULL, 
                         TASK_PRIORITY_SERVO_CONTROL, &xTaskHandleServoControl);
    if (xResult != pdPASS) {
        TASK_ERROR_PRINT("Failed to create Servo Control task");
        init_success = false;
    }
    
    xResult = xTaskCreate(vTaskSensorMonitor, "SensorMon", 
                         TASK_STACK_SIZE_SENSOR_MONITOR, NULL, 
                         TASK_PRIORITY_SENSOR_MONITOR, &xTaskHandleSensorMonitor);
    if (xResult != pdPASS) {
        TASK_ERROR_PRINT("Failed to create Sensor Monitor task");
        init_success = false;
    }
    
    xResult = xTaskCreate(vTaskMotorControl, "MotorCtrl", 
                         TASK_STACK_SIZE_MOTOR_CONTROL, NULL, 
                         TASK_PRIORITY_MOTOR_CONTROL, &xTaskHandleMotorControl);
    if (xResult != pdPASS) {
        TASK_ERROR_PRINT("Failed to create Motor Control task");
        init_success = false;
    }
    
    xResult = xTaskCreate(vTaskUserInterface, "UserUI", 
                         TASK_STACK_SIZE_USER_INTERFACE, NULL, 
                         TASK_PRIORITY_USER_INTERFACE, &xTaskHandleUserInterface);
    if (xResult != pdPASS) {
        TASK_ERROR_PRINT("Failed to create User Interface task");
        init_success = false;
    }
    
    xResult = xTaskCreate(vTaskSystemMonitor, "SysMon", 
                         TASK_STACK_SIZE_SYSTEM_MONITOR, NULL, 
                         TASK_PRIORITY_SYSTEM_MONITOR, &xTaskHandleSystemMonitor);
    if (xResult != pdPASS) {
        TASK_ERROR_PRINT("Failed to create System Monitor task");
        init_success = false;
    }
    
    xResult = xTaskCreate(vTaskUartComm, "UartComm", 
                         TASK_STACK_SIZE_UART_COMM, NULL, 
                         TASK_PRIORITY_UART_COMM, &xTaskHandleUartComm);
    if (xResult != pdPASS) {
        TASK_ERROR_PRINT("Failed to create UART Communication task");
        init_success = false;
    }
    
    xResult = xTaskCreate(vTaskLedStatus, "LedStatus", 
                         TASK_STACK_SIZE_LED_STATUS, NULL, 
                         TASK_PRIORITY_LED_STATUS, &xTaskHandleLedStatus);
    if (xResult != pdPASS) {
        TASK_ERROR_PRINT("Failed to create LED Status task");
        init_success = false;
    }
    
    /* Initialize default recipes */
    system_init_default_recipes_internal();
    
    /* Start timers */
    if (init_success) {
        xTimerStart(xTimerSystemHeartbeat, 0);
        xTimerStart(xTimerCleaningReminder, 0);
        TASK_INFO_PRINT("All tasks and objects created successfully");
    }
    
    return init_success;
}

void system_tasks_start_scheduler(void)
{
    TASK_INFO_PRINT("Starting FreeRTOS scheduler...");
    g_system_state = SYSTEM_STATE_IDLE;
    vTaskStartScheduler(); /* This function should not return */
    
    /* If we reach here, there was insufficient memory */
    TASK_ERROR_PRINT("Failed to start FreeRTOS scheduler - insufficient memory!");
    g_last_error = SYSTEM_ERROR_MEMORY;
    g_system_state = SYSTEM_STATE_ERROR;
    
    while (1) {
        /* Error indication */
        gpio_led_set_rgb(true, false, false); /* Red LED */
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_led_set_rgb(false, false, false);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/*******************************************************************************
 * Task Implementations
 ******************************************************************************/

void vTaskEmergencyMonitor(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    bool emergency_active = false;
    uint32_t emergency_start_time = 0;
    
    TASK_INFO_PRINT("Emergency Monitor task started");
    
    for (;;) {
        /* Check emergency stop button */
        bool emergency_pressed = gpio_pin_read(EMERGENCY_STOP_GPIO, EMERGENCY_STOP_PIN);
        
        if (emergency_pressed && !emergency_active) {
            /* Emergency stop activated */
            emergency_active = true;
            emergency_start_time = xTaskGetTickCount();
            
            TASK_ERROR_PRINT("EMERGENCY STOP ACTIVATED!");
            system_process_emergency_stop();
            g_system_stats.emergency_stops++;
        }
        else if (!emergency_pressed && emergency_active) {
            /* Check if emergency has been released for sufficient time */
            if ((xTaskGetTickCount() - emergency_start_time) > pdMS_TO_TICKS(EMERGENCY_STOP_DEBOUNCE_MS)) {
                emergency_active = false;
                TASK_INFO_PRINT("Emergency stop released");
                
                /* Allow system reset if in error state */
                if (g_system_state == SYSTEM_STATE_ERROR) {
                    system_clear_event(EVENT_EMERGENCY_STOP);
                }
            }
        }
        
        /* Check for other critical system events */
        EventBits_t events = xEventGroupGetBits(xEventGroupSystem);
        
        if (events & EVENT_SYSTEM_ERROR) {
            TASK_ERROR_PRINT("Critical system error detected");
            if (g_system_state != SYSTEM_STATE_ERROR) {
                g_system_state = SYSTEM_STATE_ERROR;
                system_emergency_stop();
            }
        }
        
        if (events & EVENT_SENSOR_ALARM) {
            TASK_ERROR_PRINT("Sensor alarm detected");
            /* Handle sensor-specific alarms */
        }
        
        if (events & EVENT_MOTOR_FAULT) {
            TASK_ERROR_PRINT("Motor fault detected");
            /* Stop all motors */
            gpio_motor_stop(CONVEYOR_MOTOR_INDEX);
            gpio_motor_stop(MIXING_MOTOR_INDEX);
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_PERIOD_EMERGENCY_MS));
    }
}

void vTaskServoControl(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    system_message_t message;
    uint32_t servo_timeouts[SERVO_COUNT + 1] = {0}; /* +1 for direction servo */
    
    TASK_INFO_PRINT("Servo Control task started");
    
    /* Initialize all servos to neutral position */
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        pwm_servo_set_neutral_position(i);
        pwm_servo_enable(i);
    }
    pwm_direction_servo_stop();
    
    for (;;) {
        /* Process servo commands from queue */
        while (xQueueReceive(xQueueServoCommands, &message, 0) == pdTRUE) {
            if (message.type == MSG_TYPE_SERVO_COMMAND) {
                uint8_t servo_id = message.data.servo_cmd.servo_id;
                uint8_t target_angle = message.data.servo_cmd.target_angle;
                uint16_t duration = message.data.servo_cmd.duration_ms;
                
                if (pwm_servo_is_valid_id(servo_id)) {
                    TASK_DEBUG_PRINT("Moving servo %d to angle %d", servo_id, target_angle);
                    pwm_servo_set_angle(servo_id, target_angle);
                    servo_timeouts[servo_id] = xTaskGetTickCount() + pdMS_TO_TICKS(SERVO_TIMEOUT_MS);
                }
            }
        }
        
        /* Check for servo timeouts */
        for (uint8_t i = 0; i < SERVO_COUNT; i++) {
            if (servo_timeouts[i] != 0 && xTaskGetTickCount() > servo_timeouts[i]) {
                TASK_ERROR_PRINT("Servo %d timeout!", i);
                system_handle_servo_timeout(i);
                servo_timeouts[i] = 0;
            }
        }
        
        /* Handle dispensing process */
        if (g_dispense_context.state != DISPENSE_STATE_IDLE) {
            switch (g_dispense_context.state) {
                case DISPENSE_STATE_POSITIONING:
                    /* Move to next ingredient position */
                    if (g_dispense_context.active_recipe && 
                        g_dispense_context.current_ingredient < g_dispense_context.active_recipe->ingredient_count) {
                        
                        recipe_ingredient_t *ingredient = 
                            &g_dispense_context.active_recipe->ingredients[g_dispense_context.current_ingredient];
                        
                        /* Position direction servo */
                        uint8_t position_angle = (ingredient->ingredient_id * 30); /* 30° per ingredient */
                        pwm_servo_set_angle(PWM_DIRECTION_SERVO_INDEX, position_angle);
                        
                        g_dispense_context.state = DISPENSE_STATE_DISPENSING;
                        g_dispense_context.dispense_progress_ms = 0;
                        TASK_INFO_PRINT("Positioning for ingredient %d", ingredient->ingredient_id);
                    }
                    break;
                    
                case DISPENSE_STATE_DISPENSING:
                    /* Control dispensing servo */
                    if (g_dispense_context.active_recipe && 
                        g_dispense_context.current_ingredient < g_dispense_context.active_recipe->ingredient_count) {
                        
                        recipe_ingredient_t *ingredient = 
                            &g_dispense_context.active_recipe->ingredients[g_dispense_context.current_ingredient];
                        
                        if (g_dispense_context.dispense_progress_ms < ingredient->dispense_time_ms) {
                            /* Continue dispensing */
                            pwm_servo_set_angle(ingredient->ingredient_id, 180); /* Open position */
                            g_dispense_context.dispense_progress_ms += TASK_PERIOD_SERVO_CONTROL_MS;
                        }
                        else {
                            /* Finished this ingredient */
                            pwm_servo_set_angle(ingredient->ingredient_id, 0); /* Close position */
                            g_dispense_context.current_ingredient++;
                            
                            if (g_dispense_context.current_ingredient >= g_dispense_context.active_recipe->ingredient_count) {
                                g_dispense_context.state = DISPENSE_STATE_MIXING;
                                TASK_INFO_PRINT("All ingredients dispensed, starting mixing");
                            }
                            else {
                                g_dispense_context.state = DISPENSE_STATE_POSITIONING;
                            }
                        }
                    }
                    break;
                    
                case DISPENSE_STATE_MIXING:
                    /* Mixing is handled by motor control task */
                    g_dispense_context.state = DISPENSE_STATE_COMPLETE;
                    break;
                    
                case DISPENSE_STATE_COMPLETE:
                    /* Recipe completed */
                    TASK_INFO_PRINT("Recipe '%s' completed successfully", 
                                   g_dispense_context.active_recipe->name);
                    g_system_stats.recipes_completed++;
                    
                    /* Reset dispensing context */
                    g_dispense_context.state = DISPENSE_STATE_IDLE;
                    g_dispense_context.current_ingredient = 0;
                    g_dispense_context.active_recipe = NULL;
                    
                    /* Signal completion */
                    system_set_event(EVENT_SERVO_COMPLETE);
                    break;
                    
                case DISPENSE_STATE_ERROR:
                    TASK_ERROR_PRINT("Dispensing error occurred");
                    g_dispense_context.state = DISPENSE_STATE_IDLE;
                    system_set_event(EVENT_SYSTEM_ERROR);
                    break;
                    
                default:
                    break;
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_PERIOD_SERVO_CONTROL_MS));
    }
}

void vTaskSensorMonitor(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    system_message_t sensor_message;
    uint8_t sensor_alarm_counts[WATER_LEVEL_SENSORS_COUNT] = {0};
    static uint8_t previous_sensor_states = 0;
    
    TASK_INFO_PRINT("Sensor Monitor task started");
    
    for (;;) {
        /* Read all water level sensors */
        uint8_t current_sensor_states = gpio_sensors_read_all();
        
        /* Check each sensor */
        for (uint8_t i = 0; i < WATER_LEVEL_SENSORS_COUNT; i++) {
            bool current_state = (current_sensor_states >> i) & 0x01;
            bool previous_state = (previous_sensor_states >> i) & 0x01;
            
            /* Detect state changes */
            if (current_state != previous_state) {
                TASK_DEBUG_PRINT("Sensor %d state changed: %s", i, current_state ? "HIGH" : "LOW");
                
                /* Send sensor data message */
                sensor_message.type = MSG_TYPE_SENSOR_DATA;
                sensor_message.timestamp = xTaskGetTickCount();
                sensor_message.data.sensor_data.sensor_id = i;
                sensor_message.data.sensor_data.value = current_state ? 1 : 0;
                sensor_message.data.sensor_data.alarm_state = false;
                
                xQueueSend(xQueueSensorData, &sensor_message, 0);
            }
            
            /* Check for alarm conditions (empty containers) */
            if (!current_state) { /* Assuming LOW = empty */
                sensor_alarm_counts[i]++;
                if (sensor_alarm_counts[i] > SENSOR_ALARM_THRESHOLD) {
                    TASK_ERROR_PRINT("Sensor %d alarm: Container empty!", i);
                    
                    sensor_message.type = MSG_TYPE_SENSOR_DATA;
                    sensor_message.timestamp = xTaskGetTickCount();
                    sensor_message.data.sensor_data.sensor_id = i;
                    sensor_message.data.sensor_data.value = 0;
                    sensor_message.data.sensor_data.alarm_state = true;
                    
                    xQueueSend(xQueueSensorData, &sensor_message, 0);
                    system_set_event(EVENT_SENSOR_ALARM);
                    g_system_stats.sensor_alarms++;
                    
                    sensor_alarm_counts[i] = 0; /* Reset counter */
                }
            }
            else {
                sensor_alarm_counts[i] = 0; /* Reset counter when sensor is OK */
            }
        }
        
        previous_sensor_states = current_sensor_states;
        
        /* Read position sensors */
        bool pos1 = gpio_pin_read(POSITION_SENSOR_1_GPIO, POSITION_SENSOR_1_PIN);
        bool pos2 = gpio_pin_read(POSITION_SENSOR_2_GPIO, POSITION_SENSOR_2_PIN);
        
        /* Update system with position information if needed */
        (void)pos1; /* Placeholder for position logic */
        (void)pos2;
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_PERIOD_SENSOR_MONITOR_MS));
    }
}

void vTaskMotorControl(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    system_message_t message;
    uint32_t mixing_start_time = 0;
    uint16_t mixing_duration_ms = 0;
    bool mixing_active = false;
    
    TASK_INFO_PRINT("Motor Control task started");
    
    /* Initialize motors in stopped state */
    gpio_motor_stop(CONVEYOR_MOTOR_INDEX);
    gpio_motor_stop(MIXING_MOTOR_INDEX);
    
    for (;;) {
        /* Process motor commands from queue */
        while (xQueueReceive(xQueueMotorCommands, &message, 0) == pdTRUE) {
            if (message.type == MSG_TYPE_MOTOR_COMMAND) {
                uint8_t motor_id = message.data.motor_cmd.motor_id;
                bool enable = message.data.motor_cmd.enable;
                bool direction = message.data.motor_cmd.direction;
                uint8_t speed = message.data.motor_cmd.speed;
                
                TASK_DEBUG_PRINT("Motor %d command: enable=%d, dir=%d, speed=%d", 
                                motor_id, enable, direction, speed);
                
                if (enable) {
                    gpio_motor_set_direction(motor_id, direction);
                    gpio_motor_set_enable(motor_id, true);
                }
                else {
                    gpio_motor_stop(motor_id);
                }
            }
        }
        
        /* Handle automatic mixing process */
        if (g_dispense_context.state == DISPENSE_STATE_MIXING && !mixing_active) {
            if (g_dispense_context.active_recipe) {
                mixing_duration_ms = g_dispense_context.active_recipe->mixing_time_ms;
                mixing_start_time = xTaskGetTickCount();
                mixing_active = true;
                
                TASK_INFO_PRINT("Starting mixing for %d ms", mixing_duration_ms);
                
                /* Start mixing motor */
                gpio_motor_set_direction(MIXING_MOTOR_INDEX, true); /* Forward */
                gpio_motor_set_enable(MIXING_MOTOR_INDEX, true);
            }
        }
        
        /* Check mixing completion */
        if (mixing_active) {
            uint32_t elapsed_time = xTaskGetTickCount() - mixing_start_time;
            if (elapsed_time >= pdMS_TO_TICKS(mixing_duration_ms)) {
                TASK_INFO_PRINT("Mixing completed");
                
                /* Stop mixing motor */
                gpio_motor_stop(MIXING_MOTOR_INDEX);
                mixing_active = false;
                
                /* Update dispensing state */
                if (g_dispense_context.state == DISPENSE_STATE_MIXING) {
                    g_dispense_context.state = DISPENSE_STATE_COMPLETE;
                }
            }
        }
        
        /* Monitor motor faults (basic timeout detection) */
        static uint32_t motor_start_times[2] = {0, 0};
        for (uint8_t i = 0; i < 2; i++) {
            /* Add motor fault detection logic here if needed */
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_PERIOD_MOTOR_CONTROL_MS));
    }
}

void vTaskUserInterface(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    system_message_t ui_message;
    static uint8_t last_key = 0xFF;
    static char lcd_line1[17] = "Robo-Bar Ready  ";
    static char lcd_line2[17] = "Press key...    ";
    static bool lcd_needs_update = true;
    uint8_t menu_state = 0; /* 0=main, 1=recipe select, 2=dispensing */
    
    TASK_INFO_PRINT("User Interface task started");
    
    /* Initialize LCD */
    if (xSemaphoreTake(xSemaphoreI2C, pdMS_TO_TICKS(100)) == pdTRUE) {
        lcd_init();
        lcd_clear();
        lcd_puts(0, 0, lcd_line1);
        lcd_puts(0, 1, lcd_line2);
        xSemaphoreGive(xSemaphoreI2C);
    }
    
    for (;;) {
        /* Scan keypad */
        uint8_t current_key = gpio_keypad_scan();
        
        if (current_key != 0xFF && current_key != last_key) {
            TASK_DEBUG_PRINT("Key pressed: %d", current_key);
            
            /* Send user input message */
            ui_message.type = MSG_TYPE_USER_INPUT;
            ui_message.timestamp = xTaskGetTickCount();
            ui_message.data.user_input.key_code = current_key;
            ui_message.data.user_input.is_pressed = true;
            
            xQueueSend(xQueueUserInput, &ui_message, 0);
            system_set_event(EVENT_USER_INPUT);
            
            /* Process key based on menu state */
            switch (menu_state) {
                case 0: /* Main menu */
                    if (current_key >= 1 && current_key <= g_recipe_count) {
                        /* Recipe selection */
                        const drink_recipe_t *recipe = system_get_recipe(current_key - 1);
                        if (recipe) {
                            snprintf(lcd_line1, sizeof(lcd_line1), "Recipe: %s", recipe->name);
                            snprintf(lcd_line2, sizeof(lcd_line2), "Press * to start");
                            menu_state = 1;
                            lcd_needs_update = true;
                        }
                    }
                    break;
                    
                case 1: /* Recipe selected */
                    if (current_key == 10) { /* '*' key */
                        const drink_recipe_t *recipe = system_get_recipe(0); /* Use first recipe for demo */
                        if (system_execute_recipe(recipe)) {
                            snprintf(lcd_line1, sizeof(lcd_line1), "Preparing...");
                            snprintf(lcd_line2, sizeof(lcd_line2), "%s", recipe->name);
                            menu_state = 2;
                            lcd_needs_update = true;
                        }
                    }
                    else if (current_key == 11) { /* '#' key - cancel */
                        snprintf(lcd_line1, sizeof(lcd_line1), "Robo-Bar Ready  ");
                        snprintf(lcd_line2, sizeof(lcd_line2), "Press key...    ");
                        menu_state = 0;
                        lcd_needs_update = true;
                    }
                    break;
                    
                case 2: /* Dispensing */
                    if (g_dispense_context.state == DISPENSE_STATE_IDLE ||
                        g_dispense_context.state == DISPENSE_STATE_COMPLETE) {
                        snprintf(lcd_line1, sizeof(lcd_line1), "Drink ready!    ");
                        snprintf(lcd_line2, sizeof(lcd_line2), "Press any key   ");
                        menu_state = 0;
                        lcd_needs_update = true;
                    }
                    break;
            }
            
last_key = current_key;
        }
        else if (current_key == 0xFF) {
            last_key = 0xFF; /* No key pressed */
        }
        
        /* Update LCD display if needed */
        if (lcd_needs_update) {
            if (xSemaphoreTake(xSemaphoreI2C, pdMS_TO_TICKS(100)) == pdTRUE) {
                lcd_clear();
                lcd_puts(0, 0, lcd_line1);
                lcd_puts(0, 1, lcd_line2);
                xSemaphoreGive(xSemaphoreI2C);
                lcd_needs_update = false;
                TASK_DEBUG_PRINT("LCD updated");
            }
        }
        
        /* Update display based on system state */
        if (menu_state == 2) { /* During dispensing */
            switch (g_dispense_context.state) {
                case DISPENSE_STATE_POSITIONING:
                    snprintf(lcd_line2, sizeof(lcd_line2), "Positioning...  ");
                    lcd_needs_update = true;
                    break;
                case DISPENSE_STATE_DISPENSING:
                    snprintf(lcd_line2, sizeof(lcd_line2), "Dispensing %d/%d ", 
                            g_dispense_context.current_ingredient + 1,
                            g_dispense_context.active_recipe ? g_dispense_context.active_recipe->ingredient_count : 0);
                    lcd_needs_update = true;
                    break;
                case DISPENSE_STATE_MIXING:
                    snprintf(lcd_line2, sizeof(lcd_line2), "Mixing...       ");
                    lcd_needs_update = true;
                    break;
                case DISPENSE_STATE_COMPLETE:
                    snprintf(lcd_line1, sizeof(lcd_line1), "Drink Ready!    ");
                    snprintf(lcd_line2, sizeof(lcd_line2), "Enjoy!          ");
                    lcd_needs_update = true;
                    break;
                case DISPENSE_STATE_ERROR:
                    snprintf(lcd_line1, sizeof(lcd_line1), "ERROR!          ");
                    snprintf(lcd_line2, sizeof(lcd_line2), "Check system    ");
                    lcd_needs_update = true;
                    break;
                default:
                    break;
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_PERIOD_USER_INTERFACE_MS));
    }
}

void vTaskSystemMonitor(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    system_message_t status_message;
    static uint32_t last_free_heap = 0;
    static uint32_t min_free_heap = 0xFFFFFFFF;
    
    TASK_INFO_PRINT("System Monitor task started");
    
    for (;;) {
        /* Update system uptime */
        g_system_uptime_seconds = xTaskGetTickCount() / configTICK_RATE_HZ;
        
        /* Monitor heap usage */
        uint32_t current_free_heap = xPortGetFreeHeapSize();
        if (min_free_heap > current_free_heap) {
            min_free_heap = current_free_heap;
        }
        
        /* Check for memory leaks or low memory */
        if (current_free_heap < (configTOTAL_HEAP_SIZE / 4)) { /* Less than 25% free */
            TASK_ERROR_PRINT("Low memory warning: %d bytes free", current_free_heap);
            g_last_error = SYSTEM_ERROR_MEMORY;
            system_set_event(EVENT_SYSTEM_ERROR);
        }
        
        /* Monitor task stack usage */
        UBaseType_t high_water_mark;
        
        high_water_mark = uxTaskGetStackHighWaterMark(xTaskHandleServoControl);
        if (high_water_mark < 50) { /* Less than 50 words free */
            TASK_ERROR_PRINT("Servo task stack usage critical: %d words free", high_water_mark);
        }
        
        high_water_mark = uxTaskGetStackHighWaterMark(xTaskHandleSensorMonitor);
        if (high_water_mark < 50) {
            TASK_ERROR_PRINT("Sensor task stack usage critical: %d words free", high_water_mark);
        }
        
        high_water_mark = uxTaskGetStackHighWaterMark(xTaskHandleUserInterface);
        if (high_water_mark < 50) {
            TASK_ERROR_PRINT("UI task stack usage critical: %d words free", high_water_mark);
        }
        
        /* Check system queues */
        UBaseType_t messages_waiting;
        
        messages_waiting = uxQueueMessagesWaiting(xQueueServoCommands);
        if (messages_waiting >= (QUEUE_LENGTH_SERVO_COMMANDS - 1)) {
            TASK_ERROR_PRINT("Servo command queue nearly full: %d messages", messages_waiting);
        }
        
        messages_waiting = uxQueueMessagesWaiting(xQueueSensorData);
        if (messages_waiting >= (QUEUE_LENGTH_SENSOR_DATA - 2)) {
            TASK_ERROR_PRINT("Sensor data queue nearly full: %d messages", messages_waiting);
        }
        
        /* Send system status message */
        status_message.type = MSG_TYPE_SYSTEM_STATUS;
        status_message.timestamp = xTaskGetTickCount();
        status_message.data.system_status.state = g_system_state;
        status_message.data.system_status.error = g_last_error;
        status_message.data.system_status.uptime = g_system_uptime_seconds;
        
        xQueueSend(xQueueSystemMessages, &status_message, 0);
        
        /* Check for maintenance requirements */
        if ((g_system_stats.recipes_completed % 50) == 0 && g_system_stats.recipes_completed > 0) {
            TASK_INFO_PRINT("Maintenance due: %d recipes completed", g_system_stats.recipes_completed);
            system_set_event(EVENT_MAINTENANCE_DUE);
        }
        
        /* Log system statistics periodically */
        if ((g_system_uptime_seconds % 300) == 0) { /* Every 5 minutes */
            TASK_INFO_PRINT("System Stats - Uptime: %ds, Recipes: %d, Alarms: %d, Errors: %d",
                           g_system_uptime_seconds, g_system_stats.recipes_completed,
                           g_system_stats.sensor_alarms, g_system_stats.emergency_stops);
        }
        
        last_free_heap = current_free_heap;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_PERIOD_SYSTEM_MONITOR_MS));
    }
}

void vTaskUartComm(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    system_message_t debug_message;
    char uart_buffer[128];
    static uint16_t message_counter = 0;
    
    TASK_INFO_PRINT("UART Communication task started");
    
    /* Initialize UART */
    uart_init(UART_DEBUG_INSTANCE, UART_DEBUG_BAUDRATE);
    uart_puts(UART_DEBUG_INSTANCE, "\r\n=== Robo-Bar System Started ===\r\n");
    
    for (;;) {
        /* Process debug messages from queue */
        while (xQueueReceive(xQueueDebugOutput, &debug_message, 0) == pdTRUE) {
            if (debug_message.type == MSG_TYPE_DEBUG_OUTPUT) {
                if (xSemaphoreTake(xSemaphoreUART, pdMS_TO_TICKS(100)) == pdTRUE) {
                    snprintf(uart_buffer, sizeof(uart_buffer), "[%010d] %s\r\n",
                            debug_message.timestamp, debug_message.data.debug_msg.message);
                    uart_puts(UART_DEBUG_INSTANCE, uart_buffer);
                    xSemaphoreGive(xSemaphoreUART);
                }
            }
            else if (debug_message.type == MSG_TYPE_ERROR_REPORT) {
                if (xSemaphoreTake(xSemaphoreUART, pdMS_TO_TICKS(100)) == pdTRUE) {
                    snprintf(uart_buffer, sizeof(uart_buffer), "[ERROR] Task %d: %s\r\n",
                            debug_message.data.error_report.source_task,
                            debug_message.data.error_report.description);
                    uart_puts(UART_DEBUG_INSTANCE, uart_buffer);
                    xSemaphoreGive(xSemaphoreUART);
                }
            }
        }
        
        /* Check for incoming UART commands */
        if (uart_data_available(UART_DEBUG_INSTANCE)) {
            char received_char = uart_getc(UART_DEBUG_INSTANCE);
            
            switch (received_char) {
                case 's': /* System status */
                case 'S':
                    if (xSemaphoreTake(xSemaphoreUART, pdMS_TO_TICKS(100)) == pdTRUE) {
                        snprintf(uart_buffer, sizeof(uart_buffer),
                                "System Status:\r\n"
                                "  State: %d\r\n"
                                "  Uptime: %ds\r\n"
                                "  Free Heap: %d bytes\r\n"
                                "  Recipes Completed: %d\r\n"
                                "  Emergency Stops: %d\r\n",
                                g_system_state, g_system_uptime_seconds,
                                xPortGetFreeHeapSize(), g_system_stats.recipes_completed,
                                g_system_stats.emergency_stops);
                        uart_puts(UART_DEBUG_INSTANCE, uart_buffer);
                        xSemaphoreGive(xSemaphoreUART);
                    }
                    break;
                    
                case 't': /* Task statistics */
                case 'T':
                    system_print_runtime_stats();
                    break;
                    
                case 'r': /* Reset statistics */
                case 'R':
                    memset(&g_system_stats, 0, sizeof(g_system_stats));
                    uart_puts(UART_DEBUG_INSTANCE, "Statistics reset\r\n");
                    break;
                    
                case 'h': /* Help */
                case 'H':
                case '?':
                    uart_puts(UART_DEBUG_INSTANCE,
                             "Commands:\r\n"
                             "  s - System status\r\n"
                             "  t - Task statistics\r\n"
                             "  r - Reset statistics\r\n"
                             "  h - Help\r\n");
                    break;
                    
                default:
                    break;
            }
        }
        
        /* Send periodic heartbeat */
        if ((message_counter % 100) == 0) { /* Every 10 seconds */
            if (xSemaphoreTake(xSemaphoreUART, pdMS_TO_TICKS(50)) == pdTRUE) {
                snprintf(uart_buffer, sizeof(uart_buffer), "Heartbeat: %d\r\n", message_counter);
                uart_puts(UART_DEBUG_INSTANCE, uart_buffer);
                xSemaphoreGive(xSemaphoreUART);
            }
        }
        
        message_counter++;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_PERIOD_UART_COMM_MS));
    }
}

void vTaskLedStatus(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    static uint8_t blink_counter = 0;
    static bool led_state = false;
    
    TASK_INFO_PRINT("LED Status task started");
    
    for (;;) {
        /* Update LEDs based on system state */
        switch (g_system_state) {
            case SYSTEM_STATE_INIT:
                /* Blue blinking during initialization */
                gpio_led_set_rgb(false, false, led_state);
                break;
                
            case SYSTEM_STATE_IDLE:
                /* Green solid when idle */
                gpio_led_set_rgb(false, true, false);
                break;
                
            case SYSTEM_STATE_DISPENSING:
                /* Yellow blinking during operation */
                gpio_led_set_rgb(led_state, led_state, false);
                break;
                
            case SYSTEM_STATE_ERROR:
                /* Red blinking on error */
                gpio_led_set_rgb(led_state, false, false);
                break;
                
            case SYSTEM_STATE_EMERGENCY:
                /* Red solid during emergency */
                gpio_led_set_rgb(true, false, false);
                break;
                
            case SYSTEM_STATE_MAINTENANCE:
                /* Purple blinking for maintenance */
                gpio_led_set_rgb(led_state, false, led_state);
                break;
                
            default:
                /* White blinking for unknown state */
                gpio_led_set_rgb(led_state, led_state, led_state);
                break;
        }
        
        /* Toggle LED state for blinking */
        led_state = !led_state;
        blink_counter++;
        
        /* Check for special events */
        EventBits_t events = xEventGroupGetBits(xEventGroupSystem);
        
        if (events & EVENT_EMERGENCY_STOP) {
            /* Override with red solid */
            gpio_led_set_rgb(true, false, false);
        }
        else if (events & EVENT_CLEANING_REQUIRED) {
            /* Fast purple blinking */
            if ((blink_counter % 2) == 0) {
                gpio_led_set_rgb(true, false, true);
            }
            else {
                gpio_led_set_rgb(false, false, false);
            }
        }
        else if (events & EVENT_MAINTENANCE_DUE) {
            /* Slow orange blinking */
            if ((blink_counter % 4) == 0) {
                gpio_led_set_rgb(true, true, false);
            }
            else if ((blink_counter % 4) == 2) {
                gpio_led_set_rgb(false, false, false);
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_PERIOD_LED_STATUS_MS));
    }
}

/*******************************************************************************
 * Timer Callback Implementations
 ******************************************************************************/

void vTimerCallbackSystemHeartbeat(TimerHandle_t xTimer)
{
    (void)xTimer;
    
    /* This callback runs every second */
    static uint32_t heartbeat_counter = 0;
    heartbeat_counter++;
    
    /* Toggle onboard LED periodically to show system is alive */
    if ((heartbeat_counter % 10) == 0) { /* Every 10 seconds */
        gpio_led_toggle(GPIO_LED_GREEN_INDEX);
    }
    
    /* Check system health */
    if (xPortGetFreeHeapSize() < 512) { /* Less than 512 bytes free */
        TASK_ERROR_PRINT("Critical memory shortage!");
        system_set_event(EVENT_SYSTEM_ERROR);
    }
}

void vTimerCallbackCleaningReminder(TimerHandle_t xTimer)
{
    (void)xTimer;
    
    TASK_INFO_PRINT("Cleaning reminder triggered");
    system_set_event(EVENT_CLEANING_REQUIRED);
    
    /* Send cleaning message to UI */
    system_message_t cleaning_msg;
    cleaning_msg.type = MSG_TYPE_SYSTEM_STATUS;
    cleaning_msg.timestamp = xTaskGetTickCount();
    cleaning_msg.data.system_status.state = SYSTEM_STATE_MAINTENANCE;
    cleaning_msg.data.system_status.error = SYSTEM_ERROR_NONE;
    cleaning_msg.data.system_status.uptime = g_system_uptime_seconds;
    
    xQueueSend(xQueueSystemMessages, &cleaning_msg, 0);
}

/*******************************************************************************
 * Utility Function Implementations
 ******************************************************************************/

bool system_send_message(const system_message_t *message, uint32_t timeout_ms)
{
    if (!message) {
        return false;
    }
    
    return (xQueueSend(xQueueSystemMessages, message, pdMS_TO_TICKS(timeout_ms)) == pdTRUE);
}

bool system_receive_message(system_message_t *message, uint32_t timeout_ms)
{
    if (!message) {
        return false;
    }
    
    return (xQueueReceive(xQueueSystemMessages, message, pdMS_TO_TICKS(timeout_ms)) == pdTRUE);
}

void system_set_event(EventBits_t event_bits)
{
    if (xEventGroupSystem) {
        xEventGroupSetBits(xEventGroupSystem, event_bits);
    }
}

void system_clear_event(EventBits_t event_bits)
{
    if (xEventGroupSystem) {
        xEventGroupClearBits(xEventGroupSystem, event_bits);
    }
}

EventBits_t system_wait_for_event(EventBits_t event_bits, bool clear_on_exit, 
                                  bool wait_for_all, uint32_t timeout_ms)
{
    if (!xEventGroupSystem) {
        return 0;
    }
    
    return xEventGroupWaitBits(xEventGroupSystem, event_bits, 
                              clear_on_exit ? pdTRUE : pdFALSE,
                              wait_for_all ? pdTRUE : pdFALSE,
                              pdMS_TO_TICKS(timeout_ms));
}

bool system_execute_recipe(const drink_recipe_t *recipe)
{
    if (!recipe || !system_validate_recipe(recipe)) {
        TASK_ERROR_PRINT("Invalid recipe");
        return false;
    }
    
    if (g_dispense_context.state != DISPENSE_STATE_IDLE) {
        TASK_ERROR_PRINT("System busy, cannot start new recipe");
        return false;
    }
    
    /* Check system state */
    if (g_system_state != SYSTEM_STATE_IDLE) {
        TASK_ERROR_PRINT("System not ready for recipe execution");
        return false;
    }
    
    /* Initialize dispensing context */
    g_dispense_context.state = DISPENSE_STATE_POSITIONING;
    g_dispense_context.current_ingredient = 0;
    g_dispense_context.dispense_progress_ms = 0;
    g_dispense_context.start_time = xTaskGetTickCount();
    g_dispense_context.active_recipe = (drink_recipe_t*)recipe; /* Cast away const for internal use */
    
    /* Update system state */
    g_system_state = SYSTEM_STATE_DISPENSING;
    
    TASK_INFO_PRINT("Starting recipe: %s", recipe->name);
    return true;
}

const dispense_context_t* system_get_dispense_context(void)
{
    return &g_dispense_context;
}

void system_emergency_stop(void)
{
    TASK_ERROR_PRINT("EMERGENCY STOP ACTIVATED");
    
    /* Set emergency event */
    system_set_event(EVENT_EMERGENCY_STOP);
    
    /* Stop all servos */
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        pwm_servo_set_neutral_position(i);
    }
    pwm_direction_servo_stop();
    
    /* Stop all motors */
    gpio_motor_stop(CONVEYOR_MOTOR_INDEX);
    gpio_motor_stop(MIXING_MOTOR_INDEX);
    
    /* Reset dispensing context */
    g_dispense_context.state = DISPENSE_STATE_IDLE;
    g_dispense_context.active_recipe = NULL;
    
    /* Update system state */
    g_system_state = SYSTEM_STATE_EMERGENCY;
    g_last_error = SYSTEM_ERROR_EMERGENCY_STOP;
    
    /* Increment emergency stop counter */
    g_system_stats.emergency_stops++;
}

bool system_emergency_reset(void)
{
    if (g_system_state != SYSTEM_STATE_EMERGENCY && g_system_state != SYSTEM_STATE_ERROR) {
        TASK_ERROR_PRINT("System not in emergency state");
        return false;
    }
    
    /* Clear emergency events */
    system_clear_event(EVENT_EMERGENCY_STOP | EVENT_SYSTEM_ERROR);
    
    /* Reset system state */
    g_system_state = SYSTEM_STATE_IDLE;
    g_last_error = SYSTEM_ERROR_NONE;
    
    /* Re-initialize hardware if needed */
    system_init_hardware();
    
    TASK_INFO_PRINT("System reset from emergency state");
    return true;
}

void system_get_task_stats(TaskHandle_t task_handle, TaskStatus_t *stats)
{
    if (!task_handle || !stats) {
        return;
    }
    
    vTaskGetInfo(task_handle, stats, pdTRUE, eInvalid);
}

void system_print_runtime_stats(void)
{
    char stats_buffer[40 * 9]; /* Enough for 8 tasks + header */
    
    if (xSemaphoreTake(xSemaphoreUART, pdMS_TO_TICKS(200)) == pdTRUE) {
        uart_puts(UART_DEBUG_INSTANCE, "\r\n=== Task Runtime Statistics ===\r\n");
        
        vTaskGetRunTimeStats(stats_buffer);
        uart_puts(UART_DEBUG_INSTANCE, stats_buffer);
        
        uart_puts(UART_DEBUG_INSTANCE, "\r\n=== Task Status ===\r\n");
        
        TaskStatus_t task_status;
        
        /* Print status for each task */
        if (xTaskHandleEmergency) {
            system_get_task_stats(xTaskHandleEmergency, &task_status);
            snprintf(stats_buffer, sizeof(stats_buffer), 
                    "Emergency: State=%d, Priority=%d, Stack=%d\r\n",
                    task_status.eCurrentState, task_status.uxCurrentPriority,
                    task_status.usStackHighWaterMark);
            uart_puts(UART_DEBUG_INSTANCE, stats_buffer);
        }
        
        if (xTaskHandleServoControl) {
            system_get_task_stats(xTaskHandleServoControl, &task_status);
            snprintf(stats_buffer, sizeof(stats_buffer), 
                    "ServoCtrl: State=%d, Priority=%d, Stack=%d\r\n",
                    task_status.eCurrentState, task_status.uxCurrentPriority,
                    task_status.usStackHighWaterMark);
            uart_puts(UART_DEBUG_INSTANCE, stats_buffer);
        }
        
        if (xTaskHandleUserInterface) {
            system_get_task_stats(xTaskHandleUserInterface, &task_status);
            snprintf(stats_buffer, sizeof(stats_buffer), 
                    "UserUI: State=%d, Priority=%d, Stack=%d\r\n",
                    task_status.eCurrentState, task_status.uxCurrentPriority,
                    task_status.usStackHighWaterMark);
            uart_puts(UART_DEBUG_INSTANCE, stats_buffer);
        }
        
        uart_puts(UART_DEBUG_INSTANCE, "===============================\r\n");
        xSemaphoreGive(xSemaphoreUART);
    }
}

/*******************************************************************************
 * Recipe Management Implementations
 ******************************************************************************/

void system_init_default_recipes(void)
{
    system_init_default_recipes_internal();
}

static void system_init_default_recipes_internal(void)
{
    TASK_INFO_PRINT("Initializing default recipes...");
    
    /* Recipe 1: Water Mix */
    g_recipes[0].recipe_id = 0;
    strncpy(g_recipes[0].name, "Water Mix", sizeof(g_recipes[0].name) - 1);
    g_recipes[0].ingredient_count = 2;
    g_recipes[0].ingredients[0].ingredient_id = 0;
    g_recipes[0].ingredients[0].amount_ml = 100;
    g_recipes[0].ingredients[0].dispense_time_ms = 2000;
    g_recipes[0].ingredients[1].ingredient_id = 1;
    g_recipes[0].ingredients[1].amount_ml = 50;
    g_recipes[0].ingredients[1].dispense_time_ms = 1000;
    g_recipes[0].mixing_time_ms = 3000;
    g_recipes[0].requires_cleaning = false;
    
    /* Recipe 2: Energy Drink */
    g_recipes[1].recipe_id = 1;
    strncpy(g_recipes[1].name, "Energy Drink", sizeof(g_recipes[1].name) - 1);
    g_recipes[1].ingredient_count = 3;
    g_recipes[1].ingredients[0].ingredient_id = 2;
    g_recipes[1].ingredients[0].amount_ml = 75;
    g_recipes[1].ingredients[0].dispense_time_ms = 1500;
    g_recipes[1].ingredients[1].ingredient_id = 3;
    g_recipes[1].ingredients[1].amount_ml = 25;
    g_recipes[1].ingredients[1].dispense_time_ms = 500;
    g_recipes[1].ingredients[2].ingredient_id = 0;
    g_recipes[1].ingredients[2].amount_ml = 100;
    g_recipes[1].ingredients[2].dispense_time_ms = 2000;
    g_recipes[1].mixing_time_ms = 5000;
    g_recipes[1].requires_cleaning = false;
    
    /* Recipe 3: Fruit Juice */
    g_recipes[2].recipe_id = 2;
    strncpy(g_recipes[2].name, "Fruit Juice", sizeof(g_recipes[2].name) - 1);
    g_recipes[2].ingredient_count = 4;
    g_recipes[2].ingredients[0].ingredient_id = 4;
    g_recipes[2].ingredients[0].amount_ml = 60;
    g_recipes[2].ingredients[0].dispense_time_ms = 1200;
    g_recipes[2].ingredients[1].ingredient_id = 5;
    g_recipes[2].ingredients[1].amount_ml = 40;
    g_recipes[2].ingredients[1].dispense_time_ms = 800;
    g_recipes[2].ingredients[2].ingredient_id = 0;
    g_recipes[2].ingredients[2].amount_ml = 80;
    g_recipes[2].ingredients[2].dispense_time_ms = 1600;
    g_recipes[2].ingredients[3].ingredient_id = 1;
    g_recipes[2].ingredients[3].amount_ml = 20;
    g_recipes[2].ingredients[3].dispense_time_ms = 400;
    g_recipes[2].mixing_time_ms = 4000;
    g_recipes[2].requires_cleaning = true;
    
    g_recipe_count = 3;
    TASK_INFO_PRINT("Loaded %d default recipes", g_recipe_count);
}

const drink_recipe_t* system_get_recipe(uint8_t recipe_id)
{
    if (recipe_id >= g_recipe_count) {
        return NULL;
    }
    
    return &g_recipes[recipe_id];
}

uint8_t system_get_recipe_count(void)
{
    return g_recipe_count;
}

/*******************************************************************************
 * Private Function Implementations
 ******************************************************************************/

static void system_init_hardware(void)
{
    TASK_INFO_PRINT("Initializing hardware...");
    
    /* Initialize GPIO configuration */
    gpio_config_init_all();
    
    /* Initialize PWM for servos */
    pwm_init_all_servos();
    
    /* Initialize UART for debugging */
    uart_init(UART_DEBUG_INSTANCE, UART_DEBUG_BAUDRATE);
    
    /* Initialize I2C for LCD */
    /* Note: I2C initialization would be done in lcd_config.c */
    
    /* Set initial LED state */
    gpio_led_set_rgb(false, false, true); /* Blue during init */
    
    TASK_INFO_PRINT("Hardware initialization complete");
}

static bool system_validate_recipe(const drink_recipe_t *recipe)
{
    if (!recipe) {
        return false;
    }
    
    if (recipe->ingredient_count == 0 || recipe->ingredient_count > MAX_INGREDIENTS_PER_RECIPE) {
        TASK_ERROR_PRINT("Invalid ingredient count: %d", recipe->ingredient_count);
        return false;
    }
    
    for (uint8_t i = 0; i < recipe->ingredient_count; i++) {
        if (recipe->ingredients[i].ingredient_id >= SERVO_COUNT) {
            TASK_ERROR_PRINT("Invalid ingredient ID: %d", recipe->ingredients[i].ingredient_id);
            return false;
        }
        
        if (recipe->ingredients[i].amount_ml == 0 || recipe->ingredients[i].amount_ml > 500) {
            TASK_ERROR_PRINT("Invalid amount: %d ml", recipe->ingredients[i].amount_ml);
            return false;
        }
        
        if (recipe->ingredients[i].dispense_time_ms == 0 || recipe->ingredients[i].dispense_time_ms > 10000) {
            TASK_ERROR_PRINT("Invalid dispense time: %d ms", recipe->ingredients[i].dispense_time_ms);
            return false;
        }
    }
    
    return true;
}

static void system_update_led_status(void)
{
    /* This function is called by the LED status task */
    /* Implementation is in vTaskLedStatus */
}

static void system_process_emergency_stop(void)
{
    /* Stop all ongoing operations immediately */
    TASK_ERROR_PRINT("Emergency stop triggered!");
    
    /* Set emergency stop event */
    system_set_event(EVENT_EMERGENCY_STOP);
    
    /* Stop all servos immediately */
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        pwm_set_servo_angle(i, SERVO_ANGLE_NEUTRAL);
    }
    
    /* Stop all motors */
    gpio_motor_stop(MOTOR_CONVEYOR);
    gpio_motor_stop(MOTOR_MIXER);
    
    /* Reset dispensing context */
    g_dispense_context.state = DISPENSE_STATE_ERROR;
    g_dispense_context.current_ingredient = 0;
    g_dispense_context.dispense_progress_ms = 0;
    g_dispense_context.active_recipe = NULL;
    
    /* Set error LED indication */
    gpio_led_set_rgb(true, false, false); /* Red LED for emergency */
    
    /* Log emergency event */
    system_message_t emergency_msg;
    emergency_msg.type = MSG_TYPE_ERROR_REPORT;
    emergency_msg.timestamp = xTaskGetTickCount();
    emergency_msg.data.error_report.error_type = SYSTEM_ERROR_EMERGENCY_STOP;
    emergency_msg.data.error_report.source_task = 0xFF; /* System level */
    strncpy(emergency_msg.data.error_report.description, "Emergency Stop", 
            sizeof(emergency_msg.data.error_report.description) - 1);
    emergency_msg.data.error_report.description[sizeof(emergency_msg.data.error_report.description) - 1] = '\0';
    
    /* Send emergency message (non-blocking) */
    system_send_message(&emergency_msg, 0);
    
    /* Update system state */
    g_system_state = SYSTEM_STATE_EMERGENCY;
    
    TASK_ERROR_PRINT("Emergency stop processing complete");
}

static void system_handle_servo_timeout(uint8_t servo_id)
{
    if (servo_id >= SERVO_COUNT) {
        TASK_ERROR_PRINT("Invalid servo ID for timeout: %d", servo_id);
        return;
    }
    
    TASK_ERROR_PRINT("Servo %d timeout detected", servo_id);
    
    /* Stop the servo */
    pwm_set_servo_angle(servo_id, SERVO_ANGLE_NEUTRAL);
    
    /* Set servo fault event */
    system_set_event(EVENT_SERVO_COMPLETE | EVENT_SYSTEM_ERROR);
    
    /* Update dispensing context if this servo was active */
    if (g_dispense_context.state == DISPENSE_STATE_DISPENSING) {
        /* Check if this is the current dispensing servo */
        if (g_dispense_context.active_recipe && 
            g_dispense_context.current_ingredient < g_dispense_context.active_recipe->ingredient_count) {
            
            recipe_ingredient_t *current = &g_dispense_context.active_recipe->ingredients[g_dispense_context.current_ingredient];
            if (current->ingredient_id == servo_id) {
                /* This servo timed out during dispensing */
                g_dispense_context.state = DISPENSE_STATE_ERROR;
                TASK_ERROR_PRINT("Dispensing failed due to servo %d timeout", servo_id);
            }
        }
    }
    
    /* Create error message */
    system_message_t error_msg;
    error_msg.type = MSG_TYPE_ERROR_REPORT;
    error_msg.timestamp = xTaskGetTickCount();
    error_msg.data.error_report.error_type = SYSTEM_ERROR_SERVO_TIMEOUT;
    error_msg.data.error_report.source_task = (uint8_t)eTaskGetState(xTaskHandleServoControl);
    snprintf(error_msg.data.error_report.description, 
             sizeof(error_msg.data.error_report.description), 
             "Servo %d timeout", servo_id);
    
    /* Send error message */
    system_send_message(&error_msg, pdMS_TO_TICKS(100));
    
    /* Set error LED indication */
    gpio_led_set_rgb(true, true, false); /* Yellow LED for servo error */
}

/*******************************************************************************
 * Additional Utility Functions
 ******************************************************************************/

static void system_update_dispensing_progress(void)
{
    if (g_dispense_context.state != DISPENSE_STATE_DISPENSING || 
        !g_dispense_context.active_recipe) {
        return;
    }
    
    uint32_t current_time = xTaskGetTickCount();
    uint32_t elapsed_time = current_time - g_dispense_context.start_time;
    
    /* Update progress */
    g_dispense_context.dispense_progress_ms = elapsed_time;
    
    /* Check if current ingredient is complete */
    if (g_dispense_context.current_ingredient < g_dispense_context.active_recipe->ingredient_count) {
        recipe_ingredient_t *current = &g_dispense_context.active_recipe->ingredients[g_dispense_context.current_ingredient];
        
        if (elapsed_time >= pdMS_TO_TICKS(current->dispense_time_ms)) {
            /* Current ingredient complete */
            pwm_set_servo_angle(current->ingredient_id, SERVO_ANGLE_NEUTRAL);
            
            /* Move to next ingredient */
            g_dispense_context.current_ingredient++;
            
            if (g_dispense_context.current_ingredient >= g_dispense_context.active_recipe->ingredient_count) {
                /* All ingredients dispensed, start mixing */
                g_dispense_context.state = DISPENSE_STATE_MIXING;
                g_dispense_context.start_time = current_time;
                
                /* Start mixing motor */
                gpio_motor_set_direction(MOTOR_MIXER, true);
                gpio_motor_enable(MOTOR_MIXER, true);
                
                TASK_INFO_PRINT("Starting mixing phase");
            } else {
                /* Start next ingredient */
                recipe_ingredient_t *next = &g_dispense_context.active_recipe->ingredients[g_dispense_context.current_ingredient];
                pwm_set_servo_angle(next->ingredient_id, SERVO_ANGLE_DISPENSE);
                g_dispense_context.start_time = current_time;
                
                TASK_INFO_PRINT("Dispensing ingredient %d", g_dispense_context.current_ingredient);
            }
        }
    }
}

static void system_update_mixing_progress(void)
{
    if (g_dispense_context.state != DISPENSE_STATE_MIXING || 
        !g_dispense_context.active_recipe) {
        return;
    }
    
    uint32_t current_time = xTaskGetTickCount();
    uint32_t elapsed_time = current_time - g_dispense_context.start_time;
    
    /* Check if mixing is complete */
    if (elapsed_time >= pdMS_TO_TICKS(g_dispense_context.active_recipe->mixing_time_ms)) {
        /* Stop mixing motor */
        gpio_motor_enable(MOTOR_MIXER, false);
        
        /* Recipe complete */
        g_dispense_context.state = DISPENSE_STATE_COMPLETE;
        
        /* Set completion event */
        system_set_event(EVENT_SERVO_COMPLETE);
        
        TASK_INFO_PRINT("Recipe '%s' completed successfully", 
                       g_dispense_context.active_recipe->name);
        
        /* Create completion message */
        system_message_t completion_msg;
        completion_msg.type = MSG_TYPE_SYSTEM_STATUS;
        completion_msg.timestamp = current_time;
        completion_msg.data.system_status.state = SYSTEM_STATE_READY;
        completion_msg.data.system_status.error = SYSTEM_ERROR_NONE;
        completion_msg.data.system_status.uptime = current_time;
        
        system_send_message(&completion_msg, pdMS_TO_TICKS(100));
        
        /* Check if cleaning is required */
        if (g_dispense_context.active_recipe->requires_cleaning) {
            system_set_event(EVENT_CLEANING_REQUIRED);
        }
    }
}

static bool system_check_sensor_alarms(void)
{
    bool alarm_detected = false;
    uint8_t sensor_states = gpio_read_all_sensors();
    
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        bool sensor_active = (sensor_states & (1 << i)) != 0;
        
        /* Check for low liquid level (assuming sensors are active-low when liquid present) */
        if (!sensor_active) {
            TASK_ERROR_PRINT("Low liquid level detected on sensor %d", i);
            
            /* Create sensor alarm message */
            system_message_t alarm_msg;
            alarm_msg.type = MSG_TYPE_SENSOR_DATA;
            alarm_msg.timestamp = xTaskGetTickCount();
            alarm_msg.data.sensor_data.sensor_id = i;
            alarm_msg.data.sensor_data.value = 0; /* Low level */
            alarm_msg.data.sensor_data.alarm_state = true;
            
            system_send_message(&alarm_msg, pdMS_TO_TICKS(50));
            alarm_detected = true;
        }
    }
    
    if (alarm_detected) {
        system_set_event(EVENT_SENSOR_ALARM);
    }
    
    return alarm_detected;
}

static void system_process_user_command(uint8_t key_code)
{
    TASK_INFO_PRINT("Processing user command: key %d", key_code);
    
    /* Map keypad keys to actions */
    switch (key_code) {
        case 0x01: /* Key '1' - Recipe 1 */
        case 0x02: /* Key '2' - Recipe 2 */
        case 0x03: /* Key '3' - Recipe 3 */
        case 0x04: /* Key '4' - Recipe 4 */
        case 0x05: /* Key '5' - Recipe 5 */
        case 0x06: /* Key '6' - Recipe 6 */
        {
            uint8_t recipe_id = key_code - 1;
            const drink_recipe_t *recipe = system_get_recipe(recipe_id);
            if (recipe) {
                if (g_system_state == SYSTEM_STATE_READY) {
                    system_execute_recipe(recipe);
                } else {
                    TASK_ERROR_PRINT("Cannot execute recipe - system not ready");
                }
            } else {
                TASK_ERROR_PRINT("Recipe %d not found", recipe_id);
            }
            break;
        }
        
        case 0x0A: /* Key '*' - Emergency stop */
            system_process_emergency_stop();
            break;
            
        case 0x0B: /* Key '0' - System reset */
            if (g_system_state == SYSTEM_STATE_EMERGENCY) {
                system_emergency_reset();
            }
            break;
            
        case 0x0C: /* Key '#' - Cleaning cycle */
            if (g_system_state == SYSTEM_STATE_READY) {
                /* Start cleaning cycle - this could be a special recipe */
                TASK_INFO_PRINT("Starting cleaning cycle");
                system_clear_event(EVENT_CLEANING_REQUIRED);
                /* Implement cleaning logic here */
            }
            break;
            
        default:
            TASK_DEBUG_PRINT("Unhandled key code: 0x%02X", key_code);
            break;
    }
}

static void system_check_maintenance_schedule(void)
{
    static uint32_t last_check_time = 0;
    uint32_t current_time = xTaskGetTickCount();
    
    /* Check maintenance every hour */
    if (current_time - last_check_time >= pdMS_TO_TICKS(3600000)) { /* 1 hour */
        last_check_time = current_time;
        
        /* Check operating hours */
        uint32_t operating_hours = current_time / pdMS_TO_TICKS(3600000);
        
        /* Schedule maintenance every 100 operating hours */
        if (operating_hours > 0 && (operating_hours % 100) == 0) {
            system_set_event(EVENT_MAINTENANCE_DUE);
            TASK_INFO_PRINT("Maintenance due - %lu operating hours", operating_hours);
        }
        
        /* Check for cleaning requirements */
        static uint8_t drinks_made = 0;
        drinks_made++; /* This should be updated when recipes complete */
        
        /* Require cleaning every 10 drinks */
        if (drinks_made >= 10) {
            system_set_event(EVENT_CLEANING_REQUIRED);
            drinks_made = 0;
            TASK_INFO_PRINT("Cleaning required - 10 drinks completed");
        }
    }
}

/*******************************************************************************
 * System State Management
 ******************************************************************************/

static void system_update_state_machine(void)
{
    static system_state_t previous_state = SYSTEM_STATE_INIT;
    
    if (g_system_state != previous_state) {
        TASK_INFO_PRINT("System state changed: %d -> %d", previous_state, g_system_state);
        previous_state = g_system_state;
        
        /* Handle state transitions */
        switch (g_system_state) {
            case SYSTEM_STATE_INIT:
                gpio_led_set_rgb(false, false, true); /* Blue */
                break;
                
            case SYSTEM_STATE_READY:
                gpio_led_set_rgb(false, true, false); /* Green */
                break;
                
            case SYSTEM_STATE_DISPENSING:
                gpio_led_set_rgb(false, true, true); /* Cyan */
                break;
                
            case SYSTEM_STATE_MIXING:
                gpio_led_set_rgb(true, false, true); /* Magenta */
                break;
                
            case SYSTEM_STATE_CLEANING:
                gpio_led_set_rgb(true, true, false); /* Yellow */
                break;
                
            case SYSTEM_STATE_EMERGENCY:
                gpio_led_set_rgb(true, false, false); /* Red */
                break;
                
            case SYSTEM_STATE_MAINTENANCE:
                /* Blinking yellow handled in LED task */
                break;
                
            case SYSTEM_STATE_ERROR:
                gpio_led_set_rgb(true, false, false); /* Red */
                break;
                
            default:
                TASK_ERROR_PRINT("Unknown system state: %d", g_system_state);
                break;
        }
    }
}

/*******************************************************************************
 * Debug and Diagnostics Support
 ******************************************************************************/

static void system_log_task_runtime(void)
{
    #if (configGENERATE_RUN_TIME_STATS == 1)
    static char stats_buffer[1024];
    vTaskGetRunTimeStats(stats_buffer);
    TASK_DEBUG_PRINT("Runtime Statistics:\n%s", stats_buffer);
    #endif
}

static void system_check_stack_usage(void)
{
    #if (configCHECK_FOR_STACK_OVERFLOW > 0)
    TaskHandle_t task_handles[] = {
        xTaskHandleEmergency,
        xTaskHandleServoControl,
        xTaskHandleSensorMonitor,
        xTaskHandleMotorControl,
        xTaskHandleUserInterface,
        xTaskHandleSystemMonitor,
        xTaskHandleUartComm,
        xTaskHandleLedStatus
    };
    
    const char* task_names[] = {
        "Emergency",
        "ServoCtrl",
        "SensorMon",
        "MotorCtrl",
        "UserIF",
        "SysMon",
        "UART",
        "LEDStatus"
    };
    
    for (uint8_t i = 0; i < sizeof(task_handles)/sizeof(task_handles[0]); i++) {
        if (task_handles[i] != NULL) {
            UBaseType_t stack_remaining = uxTaskGetStackHighWaterMark(task_handles[i]);
            if (stack_remaining < 50) { /* Less than 50 words remaining */
                TASK_ERROR_PRINT("Low stack in task %s: %u words remaining", 
                                task_names[i], stack_remaining);
            }
        }
    }
    #endif
}

/*******************************************************************************
 * Interrupt Service Routine Hooks
 ******************************************************************************/

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    TASK_ERROR_PRINT("Stack overflow in task: %s", pcTaskName);
    system_process_emergency_stop();
    
    /* Disable interrupts and halt */
    taskDISABLE_INTERRUPTS();
    for(;;) {
        /* Infinite loop to halt system */
    }
}

void vApplicationMallocFailedHook(void)
{
    TASK_ERROR_PRINT("Memory allocation failed!");
    system_process_emergency_stop();
    
    /* Disable interrupts and halt */
    taskDISABLE_INTERRUPTS();
    for(;;) {
        /* Infinite loop to halt system */
    }
}

void vApplicationIdleHook(void)
{
    /* This function is called when the system is idle */
    /* Can be used for power management or low-priority tasks */
    
    /* Simple power-saving: put CPU to sleep until next interrupt */
    __WFI(); /* Wait For Interrupt */
}

void vApplicationTickHook(void)
{
    /* This function is called on each tick interrupt */
    /* Keep this function lightweight to avoid affecting system performance */
    
    /* Could be used for time-critical operations or system monitoring */
    static uint32_t tick_counter = 0;
    tick_counter++;
    
    /* Every 1000 ticks (1 second at 1kHz tick rate) */
    if ((tick_counter % 1000) == 0) {
        /* Update system uptime or perform periodic checks */
        /* Note: Be careful with operations here as this runs in interrupt context */
    }
}

/*******************************************************************************
 * System Performance Monitoring
 ******************************************************************************/

static void system_monitor_performance(void)
{
    static uint32_t last_monitor_time = 0;
    uint32_t current_time = xTaskGetTickCount();
    
    /* Monitor every 10 seconds */
    if (current_time - last_monitor_time >= pdMS_TO_TICKS(10000)) {
        last_monitor_time = current_time;
        
        /* Check heap usage */
        size_t free_heap = xPortGetFreeHeapSize();
        size_t min_free_heap = xPortGetMinimumEverFreeHeapSize();
        
        if (free_heap < 1024) { /* Less than 1KB free */
            TASK_ERROR_PRINT("Low heap memory: %u bytes free", free_heap);
        }
        
        TASK_DEBUG_PRINT("Heap status - Free: %u, Min free: %u", free_heap, min_free_heap);
        
        /* Check task stack usage */
        system_check_stack_usage();
        
        /* Log runtime statistics periodically */
        #if DEBUG_TASKS_ENABLED
        static uint8_t stats_counter = 0;
        if (++stats_counter >= 6) { /* Every 6 cycles = 1 minute */
            stats_counter = 0;
            system_log_task_runtime();
        }
        #endif
    }
}