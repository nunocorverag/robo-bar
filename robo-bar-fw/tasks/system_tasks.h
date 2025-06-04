/*
 * system_tasks.h
 * 
 * FreeRTOS task definitions and configuration for Robo-Bar project
 * FRDM-KL25Z Development Board
 * 
 * Manages all system tasks including:
 * - Servo control and liquid dispensing
 * - Sensor monitoring and data acquisition
 * - User interface (keypad and LCD)
 * - Motor control (conveyor and mixing)
 * - System monitoring and diagnostics
 * - UART communication and debugging
 */

#ifndef SYSTEM_TASKS_H
#define SYSTEM_TASKS_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"
#include "config/system_config.h"
#include "config/gpio_config.h"
#include "config/pwm_config.h"
#include "config/uart_config.h"

/*******************************************************************************
 * Task Priority Definitions
 ******************************************************************************/
#define TASK_PRIORITY_EMERGENCY         (configMAX_PRIORITIES - 1)  /* Highest priority */
#define TASK_PRIORITY_SERVO_CONTROL     (configMAX_PRIORITIES - 2)  /* High priority */
#define TASK_PRIORITY_SENSOR_MONITOR    (configMAX_PRIORITIES - 3)  /* High priority */
#define TASK_PRIORITY_MOTOR_CONTROL     (configMAX_PRIORITIES - 4)  /* Medium-high */
#define TASK_PRIORITY_USER_INTERFACE    (configMAX_PRIORITIES - 5)  /* Medium */
#define TASK_PRIORITY_SYSTEM_MONITOR    (configMAX_PRIORITIES - 6)  /* Medium-low */
#define TASK_PRIORITY_UART_COMM         (configMAX_PRIORITIES - 7)  /* Low */
#define TASK_PRIORITY_LED_STATUS        (configMAX_PRIORITIES - 8)  /* Lowest */

/*******************************************************************************
 * Task Stack Sizes (in words)
 ******************************************************************************/
#define TASK_STACK_SIZE_EMERGENCY       256
#define TASK_STACK_SIZE_SERVO_CONTROL   512
#define TASK_STACK_SIZE_SENSOR_MONITOR  256
#define TASK_STACK_SIZE_MOTOR_CONTROL   256
#define TASK_STACK_SIZE_USER_INTERFACE  512
#define TASK_STACK_SIZE_SYSTEM_MONITOR  256
#define TASK_STACK_SIZE_UART_COMM       384
#define TASK_STACK_SIZE_LED_STATUS      128

/*******************************************************************************
 * Task Period Definitions (in milliseconds)
 ******************************************************************************/
#define TASK_PERIOD_EMERGENCY_MS        10      /* 100Hz - Critical */
#define TASK_PERIOD_SERVO_CONTROL_MS    20      /* 50Hz - Servo update */
#define TASK_PERIOD_SENSOR_MONITOR_MS   50      /* 20Hz - Sensor reading */
#define TASK_PERIOD_MOTOR_CONTROL_MS    100     /* 10Hz - Motor control */
#define TASK_PERIOD_USER_INTERFACE_MS   100     /* 10Hz - UI updates */
#define TASK_PERIOD_SYSTEM_MONITOR_MS   1000    /* 1Hz - System health */
#define TASK_PERIOD_UART_COMM_MS        100     /* 10Hz - Communication */
#define TASK_PERIOD_LED_STATUS_MS       500     /* 2Hz - Status indication */

/*******************************************************************************
 * Event Group Bit Definitions
 ******************************************************************************/
#define EVENT_EMERGENCY_STOP        (1 << 0)
#define EVENT_SERVO_COMPLETE        (1 << 1)
#define EVENT_SENSOR_ALARM          (1 << 2)
#define EVENT_MOTOR_FAULT           (1 << 3)
#define EVENT_USER_INPUT            (1 << 4)
#define EVENT_SYSTEM_ERROR          (1 << 5)
#define EVENT_CLEANING_REQUIRED     (1 << 6)
#define EVENT_MAINTENANCE_DUE       (1 << 7)


/*******************************************************************************
 * System Configuration Constants
 ******************************************************************************/
#define QUEUE_LENGTH_SERVO_COMMANDS     10
#define QUEUE_LENGTH_MOTOR_COMMANDS     5
#define QUEUE_LENGTH_SENSOR_DATA        20
#define QUEUE_LENGTH_USER_INPUT         5
#define QUEUE_LENGTH_SYSTEM_MESSAGES    15
#define QUEUE_LENGTH_DEBUG_OUTPUT       10

#define MAX_INGREDIENTS_PER_RECIPE      6
#define MAX_RECIPES                     10

#define SYSTEM_HEARTBEAT_INTERVAL_MS    1000
#define CLEANING_REMINDER_INTERVAL_MS   (30 * 60 * 1000)  /* 30 minutes */

#define EMERGENCY_STOP_DEBOUNCE_MS      50
#define SERVO_TIMEOUT_MS                5000
#define SENSOR_ALARM_THRESHOLD          100
#define MOTOR_FAULT_TIMEOUT_MS          2000

/*******************************************************************************
 * Debug and Diagnostics
 ******************************************************************************/
#if DEBUG_TASKS_ENABLED
    #define TASK_DEBUG_PRINT(fmt, ...)     DEBUG_PRINT_DEBUG("[TASK] " fmt, ##__VA_ARGS__)
    #define TASK_ERROR_PRINT(fmt, ...)     DEBUG_PRINT_ERROR("[TASK] " fmt, ##__VA_ARGS__)
    #define TASK_INFO_PRINT(fmt, ...)      DEBUG_PRINT_INFO("[TASK] " fmt, ##__VA_ARGS__)
#else
    #define TASK_DEBUG_PRINT(fmt, ...)     ((void)0)
    #define TASK_ERROR_PRINT(fmt, ...)     ((void)0)
    #define TASK_INFO_PRINT(fmt, ...)      ((void)0)
#endif

/*******************************************************************************
 * Queue Message Types
 ******************************************************************************/
typedef enum {
    MSG_TYPE_SERVO_COMMAND = 0,
    MSG_TYPE_MOTOR_COMMAND,
    MSG_TYPE_SENSOR_DATA,
    MSG_TYPE_USER_INPUT,
    MSG_TYPE_SYSTEM_STATUS,
    MSG_TYPE_DEBUG_OUTPUT,
    MSG_TYPE_ERROR_REPORT
} message_type_t;

typedef struct {
    message_type_t type;
    uint32_t timestamp;
    union {
        struct {
            uint8_t servo_id;
            uint8_t target_angle;
            uint16_t duration_ms;
        } servo_cmd;
        
        struct {
            uint8_t motor_id;
            bool enable;
            bool direction;
            uint8_t speed;
        } motor_cmd;
        
        struct {
            uint8_t sensor_id;
            uint16_t value;
            bool alarm_state;
        } sensor_data;
        
        struct {
            uint8_t key_code;
            bool is_pressed;
        } user_input;
        
        struct {
            system_state_t state;
            system_error_t error;
            uint32_t uptime;
        } system_status;
        
        struct {
            debug_level_t level;
            char message[64];
        } debug_msg;
        
        struct {
            system_error_t error_type;
            uint8_t source_task;
            char description[32];
        } error_report;
    } data;
} system_message_t;

/*******************************************************************************
 * Recipe and Dispensing Structures
 ******************************************************************************/
typedef struct {
    uint8_t ingredient_id;      /* 0-5 for 6 servos */
    uint16_t amount_ml;         /* Amount in milliliters */
    uint16_t dispense_time_ms;  /* Time to dispense */
} recipe_ingredient_t;

typedef struct {
    uint8_t recipe_id;
    char name[16];
    uint8_t ingredient_count;
    recipe_ingredient_t ingredients[MAX_INGREDIENTS_PER_RECIPE];
    uint16_t mixing_time_ms;
    bool requires_cleaning;
} drink_recipe_t;

typedef enum {
    DISPENSE_STATE_IDLE = 0,
    DISPENSE_STATE_POSITIONING,
    DISPENSE_STATE_DISPENSING,
    DISPENSE_STATE_MIXING,
    DISPENSE_STATE_COMPLETE,
    DISPENSE_STATE_ERROR
} dispense_state_t;

typedef struct {
    dispense_state_t state;
    uint8_t current_ingredient;
    uint16_t dispense_progress_ms;
    uint32_t start_time;
    drink_recipe_t *active_recipe;
} dispense_context_t;

/*******************************************************************************
 * Task Handle Declarations
 ******************************************************************************/
extern TaskHandle_t xTaskHandleEmergency;
extern TaskHandle_t xTaskHandleServoControl;
extern TaskHandle_t xTaskHandleSensorMonitor;
extern TaskHandle_t xTaskHandleMotorControl;
extern TaskHandle_t xTaskHandleUserInterface;
extern TaskHandle_t xTaskHandleSystemMonitor;
extern TaskHandle_t xTaskHandleUartComm;
extern TaskHandle_t xTaskHandleLedStatus;

/*******************************************************************************
 * Synchronization Objects
 ******************************************************************************/
extern QueueHandle_t xQueueServoCommands;
extern QueueHandle_t xQueueMotorCommands;
extern QueueHandle_t xQueueSensorData;
extern QueueHandle_t xQueueUserInput;
extern QueueHandle_t xQueueSystemMessages;
extern QueueHandle_t xQueueDebugOutput;

extern SemaphoreHandle_t xSemaphoreI2C;
extern SemaphoreHandle_t xSemaphoreUART;
extern SemaphoreHandle_t xSemaphoreSensorData;

extern EventGroupHandle_t xEventGroupSystem;

extern TimerHandle_t xTimerSystemHeartbeat;
extern TimerHandle_t xTimerCleaningReminder;

/*******************************************************************************
 * Task Function Prototypes
 ******************************************************************************/

/*!
 * @brief Initialize all FreeRTOS tasks and synchronization objects
 * @return true if initialization successful, false otherwise
 */
bool system_tasks_init(void);

/*!
 * @brief Start the FreeRTOS scheduler
 * @note This function does not return if successful
 */
void system_tasks_start_scheduler(void);

/*!
 * @brief Emergency monitoring task
 * Highest priority task that monitors emergency stop and critical errors
 */
void vTaskEmergencyMonitor(void *pvParameters);

/*!
 * @brief Servo control task
 * Manages all servo motors for liquid dispensing and direction control
 */
void vTaskServoControl(void *pvParameters);

/*!
 * @brief Sensor monitoring task
 * Reads all sensors and processes sensor data
 */
void vTaskSensorMonitor(void *pvParameters);

/*!
 * @brief Motor control task
 * Controls conveyor belt and mixing motors
 */
void vTaskMotorControl(void *pvParameters);

/*!
 * @brief User interface task
 * Handles keypad input and LCD display updates
 */
void vTaskUserInterface(void *pvParameters);

/*!
 * @brief System monitoring task
 * Monitors system health, performance, and maintenance requirements
 */
void vTaskSystemMonitor(void *pvParameters);

/*!
 * @brief UART communication task
 * Handles debug output and external communication
 */
void vTaskUartComm(void *pvParameters);

/*!
 * @brief LED status indication task
 * Updates status LEDs based on system state
 */
void vTaskLedStatus(void *pvParameters);

/*******************************************************************************
 * Timer Callback Prototypes
 ******************************************************************************/

/*!
 * @brief System heartbeat timer callback
 */
void vTimerCallbackSystemHeartbeat(TimerHandle_t xTimer);

/*!
 * @brief Cleaning reminder timer callback
 */
void vTimerCallbackCleaningReminder(TimerHandle_t xTimer);

/*******************************************************************************
 * Utility Functions
 ******************************************************************************/

/*!
 * @brief Send message to system queue
 * @param message Pointer to message structure
 * @param timeout_ms Timeout in milliseconds
 * @return true if message sent successfully
 */
bool system_send_message(const system_message_t *message, uint32_t timeout_ms);

/*!
 * @brief Receive message from system queue
 * @param message Pointer to message buffer
 * @param timeout_ms Timeout in milliseconds
 * @return true if message received successfully
 */
bool system_receive_message(system_message_t *message, uint32_t timeout_ms);

/*!
 * @brief Set system event bits
 * @param event_bits Event bits to set
 */
void system_set_event(EventBits_t event_bits);

/*!
 * @brief Clear system event bits
 * @param event_bits Event bits to clear
 */
void system_clear_event(EventBits_t event_bits);

/*!
 * @brief Wait for system events
 * @param event_bits Event bits to wait for
 * @param clear_on_exit Clear bits after waiting
 * @param wait_for_all Wait for all bits or any bit
 * @param timeout_ms Timeout in milliseconds
 * @return Event bits that were set
 */
EventBits_t system_wait_for_event(EventBits_t event_bits, bool clear_on_exit, 
                                  bool wait_for_all, uint32_t timeout_ms);

/*!
 * @brief Execute drink recipe
 * @param recipe Pointer to recipe structure
 * @return true if recipe execution started successfully
 */
bool system_execute_recipe(const drink_recipe_t *recipe);

/*!
 * @brief Get current dispensing context
 * @return Pointer to current dispense context
 */
const dispense_context_t* system_get_dispense_context(void);

/*!
 * @brief Emergency stop all operations
 */
void system_emergency_stop(void);

/*!
 * @brief Reset system after emergency stop
 * @return true if reset successful
 */
bool system_emergency_reset(void);

/*!
 * @brief Get task statistics
 * @param task_handle Task handle to query
 * @param stats Pointer to task status structure
 */
void system_get_task_stats(TaskHandle_t task_handle, TaskStatus_t *stats);

/*!
 * @brief Print system runtime statistics
 */
void system_print_runtime_stats(void);

/*******************************************************************************
 * Recipe Management
 ******************************************************************************/

/*!
 * @brief Initialize default recipes
 */
void system_init_default_recipes(void);

/*!
 * @brief Get recipe by ID
 * @param recipe_id Recipe identifier
 * @return Pointer to recipe or NULL if not found
 */
const drink_recipe_t* system_get_recipe(uint8_t recipe_id);

/*!
 * @brief Get number of available recipes
 * @return Number of recipes
 */
uint8_t system_get_recipe_count(void);

#endif /* SYSTEM_TASKS_H */