/*
 * Robo-Bar System - Main Application
 * FRDM-KL25Z Development Board
 * 
 * This file contains the main application entry point and system initialization
 * for the automated cocktail dispensing system.
 * 
 * Hardware: FRDM-KL25Z (ARM Cortex-M0+, 48MHz)
 * RTOS: FreeRTOS
 */

/* Standard includes */
#include <stdio.h>
#include <stdint.h>

/* MCUXpresso SDK includes */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_gpio.h"

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LED_TASK_PRIORITY       (configMAX_PRIORITIES - 1)
#define DEBUG_TASK_PRIORITY     (configMAX_PRIORITIES - 2)
#define SYSTEM_TASK_PRIORITY    (configMAX_PRIORITIES - 3)

#define LED_TASK_STACK_SIZE     (configMINIMAL_STACK_SIZE + 100)
#define DEBUG_TASK_STACK_SIZE   (configMINIMAL_STACK_SIZE + 200)
#define SYSTEM_TASK_STACK_SIZE  (configMINIMAL_STACK_SIZE + 150)

/* LED configuration - Using onboard RGB LED */
#define BOARD_LED_RED_GPIO      GPIOB
#define BOARD_LED_RED_GPIO_PIN  18U
#define BOARD_LED_GREEN_GPIO    GPIOB
#define BOARD_LED_GREEN_GPIO_PIN 19U
#define BOARD_LED_BLUE_GPIO     GPIOD
#define BOARD_LED_BLUE_GPIO_PIN 1U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void led_task(void *pvParameters);
static void debug_task(void *pvParameters);
static void system_monitor_task(void *pvParameters);
static void hardware_init(void);
static void gpio_init(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static TaskHandle_t led_task_handle = NULL;
static TaskHandle_t debug_task_handle = NULL;
static TaskHandle_t system_task_handle = NULL;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Initialize board hardware */
    hardware_init();
    
    /* Initialize GPIO for LED control */
    gpio_init();
    
    /* Print system information */
    PRINTF("\r\n");
    PRINTF("*************************************************\r\n");
    PRINTF("*        Robo-Bar System Initializing          *\r\n");
    PRINTF("*        FRDM-KL25Z + FreeRTOS                  *\r\n");
    PRINTF("*************************************************\r\n");
    PRINTF("System Clock: %d Hz\r\n", CLOCK_GetFreq(kCLOCK_CoreSysClk));
    PRINTF("FreeRTOS Version: %s\r\n", tskKERNEL_VERSION_NUMBER);
    
    /* Create FreeRTOS tasks */
    if (xTaskCreate(led_task, "LED_Task", LED_TASK_STACK_SIZE, NULL, 
                    LED_TASK_PRIORITY, &led_task_handle) != pdPASS)
    {
        PRINTF("ERROR: Failed to create LED task\r\n");
        for(;;);
    }
    
    if (xTaskCreate(debug_task, "Debug_Task", DEBUG_TASK_STACK_SIZE, NULL, 
                    DEBUG_TASK_PRIORITY, &debug_task_handle) != pdPASS)
    {
        PRINTF("ERROR: Failed to create Debug task\r\n");
        for(;;);
    }
    
    if (xTaskCreate(system_monitor_task, "System_Task", SYSTEM_TASK_STACK_SIZE, NULL, 
                    SYSTEM_TASK_PRIORITY, &system_task_handle) != pdPASS)
    {
        PRINTF("ERROR: Failed to create System Monitor task\r\n");
        for(;;);
    }
    
    PRINTF("All tasks created successfully\r\n");
    PRINTF("Starting FreeRTOS scheduler...\r\n");
    
    /* Start the scheduler */
    vTaskStartScheduler();
    
    /* Should never reach this point */
    PRINTF("ERROR: Scheduler failed to start!\r\n");
    for(;;);
}

/*!
 * @brief Hardware initialization function
 */
static void hardware_init(void)
{
    /* Initialize board hardware */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    
    /* Enable clock for GPIO ports */
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_PortD);
}

/*!
 * @brief GPIO initialization for LEDs
 */
static void gpio_init(void)
{
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput,
        1, /* Initial high state (LED off for active-low) */
    };
    
    /* Configure LED pins as GPIO */
    PORT_SetPinMux(PORTB, BOARD_LED_RED_GPIO_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTB, BOARD_LED_GREEN_GPIO_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTD, BOARD_LED_BLUE_GPIO_PIN, kPORT_MuxAsGpio);
    
    /* Initialize GPIO for LEDs */
    GPIO_PinInit(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PIN, &led_config);
    GPIO_PinInit(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PIN, &led_config);
    GPIO_PinInit(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PIN, &led_config);
    
    /* Turn off all LEDs initially */
    GPIO_PortSet(BOARD_LED_RED_GPIO, 1U << BOARD_LED_RED_GPIO_PIN);
    GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN);
    GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN);
}

/*!
 * @brief LED test task - Cycles through RGB LED colors
 */
static void led_task(void *pvParameters)
{
    const TickType_t delay_ms = pdMS_TO_TICKS(1000);
    uint8_t led_state = 0;
    
    PRINTF("LED Task started\r\n");
    
    for(;;)
    {
        /* Turn off all LEDs first */
        GPIO_PortSet(BOARD_LED_RED_GPIO, 1U << BOARD_LED_RED_GPIO_PIN);
        GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN);
        GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN);
        
        /* Cycle through colors */
        switch(led_state)
        {
            case 0: /* Red */
                GPIO_PortClear(BOARD_LED_RED_GPIO, 1U << BOARD_LED_RED_GPIO_PIN);
                break;
            case 1: /* Green */
                GPIO_PortClear(BOARD_LED_GREEN_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN);
                break;
            case 2: /* Blue */
                GPIO_PortClear(BOARD_LED_BLUE_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN);
                break;
            default:
                led_state = 0;
                continue;
        }
        
        led_state = (led_state + 1) % 3;
        vTaskDelay(delay_ms);
    }
}

/*!
 * @brief Debug task - Prints periodic status messages
 */
static void debug_task(void *pvParameters)
{
    const TickType_t delay_ms = pdMS_TO_TICKS(5000);
    uint32_t counter = 0;
    
    PRINTF("Debug Task started\r\n");
    
    for(;;)
    {
        PRINTF("[DEBUG] System running - Uptime: %lu seconds\r\n", counter * 5);
        PRINTF("[DEBUG] Free heap: %d bytes\r\n", xPortGetFreeHeapSize());
        
        counter++;
        vTaskDelay(delay_ms);
    }
}

/*!
 * @brief System monitor task - Monitors system health
 */
static void system_monitor_task(void *pvParameters)
{
    const TickType_t delay_ms = pdMS_TO_TICKS(2000);
    
    PRINTF("System Monitor Task started\r\n");
    
    for(;;)
    {
        /* Check if other tasks are still running */
        if (eTaskGetState(led_task_handle) == eSuspended)
        {
            PRINTF("[WARNING] LED task suspended!\r\n");
        }
        
        if (eTaskGetState(debug_task_handle) == eSuspended)
        {
            PRINTF("[WARNING] Debug task suspended!\r\n");
        }
        
        /* Monitor stack usage */
        UBaseType_t led_stack_remaining = uxTaskGetStackHighWaterMark(led_task_handle);
        UBaseType_t debug_stack_remaining = uxTaskGetStackHighWaterMark(debug_task_handle);
        
        if (led_stack_remaining < 50)
        {
            PRINTF("[WARNING] LED task low stack: %d words remaining\r\n", led_stack_remaining);
        }
        
        if (debug_stack_remaining < 50)
        {
            PRINTF("[WARNING] Debug task low stack: %d words remaining\r\n", debug_stack_remaining);
        }
        
        vTaskDelay(delay_ms);
    }
}

/*******************************************************************************
 * FreeRTOS hook functions
 ******************************************************************************/

/*!
 * @brief FreeRTOS malloc failed hook
 */
void vApplicationMallocFailedHook(void)
{
    PRINTF("FATAL ERROR: FreeRTOS malloc failed!\r\n");
    taskDISABLE_INTERRUPTS();
    for(;;);
}

/*!
 * @brief FreeRTOS stack overflow hook
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    PRINTF("FATAL ERROR: Stack overflow in task: %s\r\n", pcTaskName);
    taskDISABLE_INTERRUPTS();
    for(;;);
}

/*!
 * @brief FreeRTOS idle hook
 */
void vApplicationIdleHook(void)
{
    /* This function is called during idle time */
    /* Can be used for low-power modes or background tasks */
}

/*!
 * @brief FreeRTOS tick hook
 */
void vApplicationTickHook(void)
{
    /* This function is called on each tick interrupt */
    /* Keep this function short and fast */
}