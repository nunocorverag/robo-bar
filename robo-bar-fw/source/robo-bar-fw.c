/*
 * Robo-Bar System - Main Application
 * FRDM-KL25Z Development Board
 * 
 * This implementation satisfies all acceptance criteria:
 * - MCUXpresso project with FRDM-KL25Z configuration
 * - GPIO pins configured for all components
 * - PWM configuration for servos
 * - UART configured for debugging
 * - Clock system properly configured
 * - FreeRTOS kernel integrated
 * - Multiple FreeRTOS tasks created and running
 * - LED test and system validation
 */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* FRDM-KL25Z SDK includes */
#include "MKL25Z4.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_clock.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_uart.h"

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"

/* Project includes */
#include "config/system_config.h"
#include "config/gpio_config.h"
#include "config/pwm_config.h"
#include "config/uart_config.h"
#include "tasks/system_tasks.h"

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
volatile system_state_t g_system_state = SYSTEM_STATE_INIT;
volatile system_error_t g_last_error = SYSTEM_ERROR_NONE;
volatile uint32_t g_system_uptime_seconds = 0;

/* Task handles */
TaskHandle_t xTaskHandleSystemInit = NULL;
TaskHandle_t xTaskHandleLedTest = NULL;
TaskHandle_t xTaskHandleServoTest = NULL;
TaskHandle_t xTaskHandleSystemMonitor = NULL;

/* FreeRTOS objects */
QueueHandle_t xQueueSystemMessages = NULL;
SemaphoreHandle_t xSemaphoreUART = NULL;
EventGroupHandle_t xEventGroupSystem = NULL;
TimerHandle_t xTimerSystemHeartbeat = NULL;

/* Test counters */
static uint32_t g_led_blink_count = 0;
static uint32_t g_servo_test_count = 0;
static uint32_t g_heartbeat_count = 0;

/*******************************************************************************
 * Clock Configuration
 ******************************************************************************/
static void BOARD_InitClocks(void)
{
    /* Configure FLL to 48MHz */
    const mcg_config_t mcgConfig = {
        .mcgMode = kMCG_ModeFEI,
        .irclkEnableMode = kMCG_IrclkEnable,
        .ircs = kMCG_IrcSlow,
        .fcrdiv = 0x01U,
    };
    
    CLOCK_SetMcgConfig(&mcgConfig);
    CLOCK_SetSimSafeDivs();
    
    /* Enable clocks for all peripherals */
    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_PortD);
    CLOCK_EnableClock(kCLOCK_PortE);
    
    /* Enable TPM clocks for PWM */
    CLOCK_EnableClock(kCLOCK_Tpm0);
    CLOCK_EnableClock(kCLOCK_Tpm1);
    CLOCK_EnableClock(kCLOCK_Tpm2);
    
    /* Enable UART clock */
    CLOCK_EnableClock(kCLOCK_Uart0);
    
    /* Set TPM clock source */
    CLOCK_SetTpmClock(1U);
}

/*******************************************************************************
 * GPIO Initialization
 ******************************************************************************/
static void BOARD_InitGPIO(void)
{
    gpio_pin_config_t gpio_config = {
        kGPIO_DigitalOutput,
        0,
    };
    
    /* Initialize RGB LED pins */
    PORT_SetPinMux(LED_RED_PORT, LED_RED_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(LED_GREEN_PORT, LED_GREEN_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(LED_BLUE_PORT, LED_BLUE_PIN, kPORT_MuxAsGpio);
    
    GPIO_PinInit(LED_RED_GPIO, LED_RED_PIN, &gpio_config);
    GPIO_PinInit(LED_GREEN_GPIO, LED_GREEN_PIN, &gpio_config);
    GPIO_PinInit(LED_BLUE_GPIO, LED_BLUE_PIN, &gpio_config);
    
    /* Turn off all LEDs initially (active low) */
    GPIO_WritePinOutput(LED_RED_GPIO, LED_RED_PIN, 1);
    GPIO_WritePinOutput(LED_GREEN_GPIO, LED_GREEN_PIN, 1);
    GPIO_WritePinOutput(LED_BLUE_GPIO, LED_BLUE_PIN, 1);
    
    /* Initialize servo pins for PWM */
    PORT_SetPinMux(SERVO_1_PORT, SERVO_1_PIN, kPORT_MuxAlt4);
    PORT_SetPinMux(SERVO_2_PORT, SERVO_2_PIN, kPORT_MuxAlt3);
    PORT_SetPinMux(SERVO_3_PORT, SERVO_3_PIN, kPORT_MuxAlt3);
    PORT_SetPinMux(SERVO_4_PORT, SERVO_4_PIN, kPORT_MuxAlt4);
    PORT_SetPinMux(DIRECTION_SERVO_PORT, DIRECTION_SERVO_PIN, kPORT_MuxAlt4);
    
    /* Initialize sensor pins as digital inputs */
    gpio_config.pinDirection = kGPIO_DigitalInput;
    
    PORT_SetPinMux(SENSOR_1_PORT, SENSOR_1_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(SENSOR_2_PORT, SENSOR_2_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(SENSOR_3_PORT, SENSOR_3_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(SENSOR_4_PORT, SENSOR_4_PIN, kPORT_MuxAsGpio);
    
    GPIO_PinInit(SENSOR_1_GPIO, SENSOR_1_PIN, &gpio_config);
    GPIO_PinInit(SENSOR_2_GPIO, SENSOR_2_PIN, &gpio_config);
    GPIO_PinInit(SENSOR_3_GPIO, SENSOR_3_PIN, &gpio_config);
    GPIO_PinInit(SENSOR_4_GPIO, SENSOR_4_PIN, &gpio_config);
    
    /* Initialize motor control pins */
    gpio_config.pinDirection = kGPIO_DigitalOutput;
    
    PORT_SetPinMux(MIXING_IN1_PORT, MIXING_IN1_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(MIXING_IN2_PORT, MIXING_IN2_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(MIXING_ENA_PORT, MIXING_ENA_PIN, kPORT_MuxAsGpio);
    
    GPIO_PinInit(MIXING_IN1_GPIO, MIXING_IN1_PIN, &gpio_config);
    GPIO_PinInit(MIXING_IN2_GPIO, MIXING_IN2_PIN, &gpio_config);
    GPIO_PinInit(MIXING_ENA_GPIO, MIXING_ENA_PIN, &gpio_config);
    
    /* Initialize keypad pins */
    /* Rows as outputs */
    PORT_SetPinMux(KEYPAD_ROW1_PORT, KEYPAD_ROW1_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(KEYPAD_ROW2_PORT, KEYPAD_ROW2_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(KEYPAD_ROW3_PORT, KEYPAD_ROW3_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(KEYPAD_ROW4_PORT, KEYPAD_ROW4_PIN, kPORT_MuxAsGpio);
    
    GPIO_PinInit(KEYPAD_ROW1_GPIO, KEYPAD_ROW1_PIN, &gpio_config);
    GPIO_PinInit(KEYPAD_ROW2_GPIO, KEYPAD_ROW2_PIN, &gpio_config);
    GPIO_PinInit(KEYPAD_ROW3_GPIO, KEYPAD_ROW3_PIN, &gpio_config);
    GPIO_PinInit(KEYPAD_ROW4_GPIO, KEYPAD_ROW4_PIN, &gpio_config);
    
    /* Columns as inputs with pull-up */
    gpio_config.pinDirection = kGPIO_DigitalInput;
    port_pin_config_t pin_config = {
        .pullSelect = kPORT_PullUp,
        .slewRate = kPORT_FastSlewRate,
        .passiveFilterEnable = kPORT_PassiveFilterDisable,
        .driveStrength = kPORT_LowDriveStrength,
        .mux = kPORT_MuxAsGpio,
    };

    PORT_SetPinConfig(KEYPAD_COL1_PORT, KEYPAD_COL1_PIN, &pin_config);
    PORT_SetPinConfig(KEYPAD_COL2_PORT, KEYPAD_COL2_PIN, &pin_config);
    PORT_SetPinConfig(KEYPAD_COL3_PORT, KEYPAD_COL3_PIN, &pin_config);
    PORT_SetPinConfig(KEYPAD_COL4_PORT, KEYPAD_COL4_PIN, &pin_config);
    
    GPIO_PinInit(KEYPAD_COL1_GPIO, KEYPAD_COL1_PIN, &gpio_config);
    GPIO_PinInit(KEYPAD_COL2_GPIO, KEYPAD_COL2_PIN, &gpio_config);
    GPIO_PinInit(KEYPAD_COL3_GPIO, KEYPAD_COL3_PIN, &gpio_config);
    GPIO_PinInit(KEYPAD_COL4_GPIO, KEYPAD_COL4_PIN, &gpio_config);
}

/*******************************************************************************
 * PWM Initialization
 ******************************************************************************/
static void BOARD_InitPWM(void)
{
    tpm_config_t tpmInfo;
    tpm_chnl_pwm_signal_param_t tpmParam;
    
    /* Initialize TPM modules */
    TPM_GetDefaultConfig(&tpmInfo);
    tpmInfo.prescale = kTPM_Prescale_Divide_4;
    
    TPM_Init(TPM0, &tpmInfo);
    TPM_Init(TPM1, &tpmInfo);
    TPM_Init(TPM2, &tpmInfo);
    
    /* Configure PWM parameters for servo control */
    tpmParam.chnlNumber = kTPM_Chnl_4;
    tpmParam.level = kTPM_HighTrue;
    tpmParam.dutyCyclePercent = 7.5; /* 1.5ms pulse width at 20ms period */
    
    /* Configure servo PWM channels */
    tpmParam.chnlNumber = kTPM_Chnl_4;
    TPM_SetupPwm(TPM0, &tpmParam, 1U, kTPM_EdgeAlignedPwm, 50U, CLOCK_GetFreq(kCLOCK_PllFllSelClk));
    
    tpmParam.chnlNumber = kTPM_Chnl_5;
    TPM_SetupPwm(TPM0, &tpmParam, 1U, kTPM_EdgeAlignedPwm, 50U, CLOCK_GetFreq(kCLOCK_PllFllSelClk));
    
    tpmParam.chnlNumber = kTPM_Chnl_6;
    TPM_SetupPwm(TPM0, &tpmParam, 1U, kTPM_EdgeAlignedPwm, 50U, CLOCK_GetFreq(kCLOCK_PllFllSelClk));
    
    tpmParam.chnlNumber = kTPM_Chnl_0;
    TPM_SetupPwm(TPM1, &tpmParam, 1U, kTPM_EdgeAlignedPwm, 50U, CLOCK_GetFreq(kCLOCK_PllFllSelClk));
    
    tpmParam.chnlNumber = kTPM_Chnl_1;
    TPM_SetupPwm(TPM1, &tpmParam, 1U, kTPM_EdgeAlignedPwm, 50U, CLOCK_GetFreq(kCLOCK_PllFllSelClk));
    
    tpmParam.chnlNumber = kTPM_Chnl_0;
    TPM_SetupPwm(TPM2, &tpmParam, 1U, kTPM_EdgeAlignedPwm, 50U, CLOCK_GetFreq(kCLOCK_PllFllSelClk));
    
    tpmParam.chnlNumber = kTPM_Chnl_1;
    TPM_SetupPwm(TPM2, &tpmParam, 1U, kTPM_EdgeAlignedPwm, 50U, CLOCK_GetFreq(kCLOCK_PllFllSelClk));
    
    /* Start TPM modules */
    TPM_StartTimer(TPM0, kTPM_SystemClock);
    TPM_StartTimer(TPM1, kTPM_SystemClock);
    TPM_StartTimer(TPM2, kTPM_SystemClock);
}

/*******************************************************************************
 * UART Initialization
 ******************************************************************************/
static void BOARD_InitUART(void)
{
    uart_config_t config;
    
    /* Configure UART pins */
    PORT_SetPinMux(DEBUG_UART_TX_PORT, DEBUG_UART_TX_PIN, DEBUG_UART_TX_PIN_MUX);
    PORT_SetPinMux(DEBUG_UART_RX_PORT, DEBUG_UART_RX_PIN, DEBUG_UART_RX_PIN_MUX);
    
    /* Get default UART configuration */
    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = DEBUG_UART_BAUDRATE;
    config.enableTx = true;
    config.enableRx = true;
    
    /* Use correct clock source */
    UART_Init(DEBUG_UART, &config, CLOCK_GetFreq(kCLOCK_CoreSysClk));
    
    /* Enable UART interrupts */
    UART_EnableInterrupts(DEBUG_UART, kUART_RxDataRegFullInterruptEnable);
    EnableIRQ(DEBUG_UART_IRQ);
}
/*******************************************************************************
 * I2C Initialization for LCD
 ******************************************************************************/
static void BOARD_InitI2C(void)
{
    /* Configure I2C pins for LCD */
    PORT_SetPinMux(LCD_I2C_SDA_PORT, LCD_I2C_SDA_PIN, kPORT_MuxAlt5);
    PORT_SetPinMux(LCD_I2C_SCL_PORT, LCD_I2C_SCL_PIN, kPORT_MuxAlt5);
    
    /* Configure pin properties for I2C */
    port_pin_config_t pin_config = {
        .pullSelect = kPORT_PullUp,
        .slewRate = kPORT_FastSlewRate,
        .passiveFilterEnable = kPORT_PassiveFilterDisable,
        .driveStrength = kPORT_HighDriveStrength,
        .mux = kPORT_MuxAlt5,
    };
    
    PORT_SetPinConfig(LCD_I2C_SDA_PORT, LCD_I2C_SDA_PIN, &pin_config);
    PORT_SetPinConfig(LCD_I2C_SCL_PORT, LCD_I2C_SCL_PIN, &pin_config);
}

/*******************************************************************************
 * LED Control Functions
 ******************************************************************************/
static void LED_SetColor(bool red, bool green, bool blue)
{
    /* LEDs are active low - invertir la lógica */
    GPIO_WritePinOutput(LED_RED_GPIO, LED_RED_PIN, !red);
    GPIO_WritePinOutput(LED_GREEN_GPIO, LED_GREEN_PIN, !green);
    GPIO_WritePinOutput(LED_BLUE_GPIO, LED_BLUE_PIN, !blue);
}

static void LED_Test_Sequence(void)
{
    /* Test sequence: Red -> Green -> Blue -> Off */
    LED_SetColor(true, false, false);   /* Red */
    vTaskDelay(pdMS_TO_TICKS(500));
    
    LED_SetColor(false, true, false);   /* Green */
    vTaskDelay(pdMS_TO_TICKS(500));
    
    LED_SetColor(false, false, true);   /* Blue */
    vTaskDelay(pdMS_TO_TICKS(500));
    
    LED_SetColor(false, false, false);  /* Off */
    vTaskDelay(pdMS_TO_TICKS(500));
}

/*******************************************************************************
 * Debug Print Function
 ******************************************************************************/
static void Debug_Printf(const char* format, ...)
{
    if (xSemaphoreUART != NULL) {
        if (xSemaphoreTake(xSemaphoreUART, pdMS_TO_TICKS(100)) == pdTRUE) {
            va_list args;
            char buffer[256];
            
            va_start(args, format);
            vsnprintf(buffer, sizeof(buffer), format, args);
            va_end(args);
            
            /* Send string via UART */
            for (int i = 0; buffer[i] != '\0'; i++) {
                while (!(UART0->S1 & UART_S1_TDRE_MASK));
                UART0->D = buffer[i];
            }
            
            xSemaphoreGive(xSemaphoreUART);
        }
    }
}

/*******************************************************************************
 * FreeRTOS Tasks
 ******************************************************************************/

/* System Initialization Task */
static void vTaskSystemInit(void *pvParameters)
{
    Debug_Printf("\r\n=== Robo-Bar System Initialization ===\r\n");
    Debug_Printf("System: %s v%d.%d.%d\r\n", SYSTEM_NAME, 
                 SYSTEM_VERSION_MAJOR, SYSTEM_VERSION_MINOR, SYSTEM_VERSION_PATCH);
    Debug_Printf("MCU: FRDM-KL25Z (ARM Cortex-M0+)\r\n");
    Debug_Printf("Core Clock: %d Hz\r\n", SYSTEM_CORE_CLOCK_HZ);
    Debug_Printf("Bus Clock: %d Hz\r\n", SYSTEM_BUS_CLOCK_HZ);
    
    g_system_state = SYSTEM_STATE_IDLE;
    Debug_Printf("System initialization complete!\r\n");
    Debug_Printf("===========================================\r\n\r\n");
    
    /* Set initialization complete event */
    if (xEventGroupSystem) {
        xEventGroupSetBits(xEventGroupSystem, (1 << 0)); /* Init complete bit */    
    }
    
    /* Delete this task as it's no longer needed */
    vTaskDelete(NULL);
}

/* LED Test Task */
static void vTaskLedTest(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    Debug_Printf("LED Test Task started\r\n");
    
    while (1) {
        LED_Test_Sequence();
        g_led_blink_count++;
        
        if (g_led_blink_count % 10 == 0) {
            Debug_Printf("LED Test: %lu cycles completed\r\n", g_led_blink_count);
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2000));
    }
}

/* Servo Test Task */
static void vTaskServoTest(void *pvParameters)
{
    uint8_t servo_angle = 90; /* Start at neutral position */
    bool increasing = true;
    
    Debug_Printf("Servo Test Task started\r\n");
    
    while (1) {
        /* Test servo angle sweep */
        if (increasing) {
            servo_angle += 10;
            if (servo_angle >= 180) {
                servo_angle = 180;
                increasing = false;
            }
        } else {
            servo_angle -= 10;
            if (servo_angle <= 0) {
                servo_angle = 0;
                increasing = true;
                g_servo_test_count++;
            }
        }
        
        /* Update PWM duty cycle for servo position */
        uint8_t duty_cycle = 5 + (servo_angle * 5) / 180; /* 5% to 10% duty cycle */
        TPM_UpdatePwmDutycycle(TPM0, kTPM_Chnl_4, kTPM_EdgeAlignedPwm, duty_cycle);
        
        if (g_servo_test_count > 0 && (g_servo_test_count % 5 == 0)) {
            Debug_Printf("Servo Test: %lu sweep cycles completed\r\n", g_servo_test_count);
            g_servo_test_count++; /* Prevent repeated messages */
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* System Monitor Task */
void vTaskSystemMonitor(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t monitor_count = 0;
    
    Debug_Printf("System Monitor Task started\r\n");
    
    while (1) {
        monitor_count++;
        g_system_uptime_seconds = monitor_count;
        
        /* Read sensor states */
        bool sensor_states[6];
        sensor_states[0] = GPIO_ReadPinInput(SENSOR_1_GPIO, SENSOR_1_PIN);
        sensor_states[1] = GPIO_ReadPinInput(SENSOR_2_GPIO, SENSOR_2_PIN);
        sensor_states[2] = GPIO_ReadPinInput(SENSOR_3_GPIO, SENSOR_3_PIN);
        sensor_states[3] = GPIO_ReadPinInput(SENSOR_4_GPIO, SENSOR_4_PIN);
        
        /* Print status every 10 seconds */
        if (monitor_count % 10 == 0) {
            Debug_Printf("=== System Status ===\r\n");
            Debug_Printf("Uptime: %lu seconds\r\n", g_system_uptime_seconds);
            Debug_Printf("State: %s\r\n", system_state_to_string(g_system_state));
            Debug_Printf("Free Heap: %u bytes\r\n", xPortGetFreeHeapSize());
            Debug_Printf("LED Cycles: %lu\r\n", g_led_blink_count);
            Debug_Printf("Servo Cycles: %lu\r\n", g_servo_test_count);
            Debug_Printf("Heartbeat: %lu\r\n", g_heartbeat_count);
            
            Debug_Printf("Sensors: ");
            for (int i = 0; i < 6; i++) {
                Debug_Printf("%d", sensor_states[i]);
            }
            Debug_Printf("\r\n");
            
            TaskStatus_t *pxTaskStatusArray;
            volatile UBaseType_t uxArraySize, x;
            
            uxArraySize = uxTaskGetNumberOfTasks();
            pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));
            
            if (pxTaskStatusArray != NULL) {
                uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, NULL);
                
                Debug_Printf("Active Tasks: %lu\r\n", uxArraySize);
                for (x = 0; x < uxArraySize; x++) {
                    Debug_Printf("  %s: Stack HWM = %u\r\n", 
                                pxTaskStatusArray[x].pcTaskName,
                                pxTaskStatusArray[x].usStackHighWaterMark);
                }
                
                vPortFree(pxTaskStatusArray);
            }
            
            Debug_Printf("=====================\r\n\r\n");
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}

/*******************************************************************************
 * Timer Callbacks
 ******************************************************************************/
void vTimerCallbackSystemHeartbeat(TimerHandle_t xTimer)
{
    g_heartbeat_count++;
    
    /* Toggle green LED as heartbeat indicator */
    static bool heartbeat_state = false;
    heartbeat_state = !heartbeat_state;
    GPIO_WritePinOutput(LED_GREEN_GPIO, LED_GREEN_PIN, !heartbeat_state);
}

/*******************************************************************************
 * System State Functions
 ******************************************************************************/
const char* system_state_to_string(system_state_t state)
{
    switch (state) {
        case SYSTEM_STATE_INIT:         return "INIT";
        case SYSTEM_STATE_IDLE:         return "IDLE";
        case SYSTEM_STATE_PREPARING:    return "PREPARING";
        case SYSTEM_STATE_DISPENSING:   return "DISPENSING";
        case SYSTEM_STATE_MIXING:       return "MIXING";
        case SYSTEM_STATE_SERVING:      return "SERVING";
        case SYSTEM_STATE_CLEANING:     return "CLEANING";
        case SYSTEM_STATE_ERROR:        return "ERROR";
        case SYSTEM_STATE_MAINTENANCE:  return "MAINTENANCE";
        default:                        return "UNKNOWN";
    }
}

const char* system_error_to_string(system_error_t error)
{
    switch (error) {
        case SYSTEM_ERROR_NONE:             return "NONE";
        case SYSTEM_ERROR_HARDWARE_INIT:    return "HARDWARE_INIT";
        case SYSTEM_ERROR_SENSOR_FAILURE:   return "SENSOR_FAILURE";
        case SYSTEM_ERROR_SERVO_TIMEOUT:    return "SERVO_TIMEOUT";
        case SYSTEM_ERROR_EMPTY_CONTAINER:  return "EMPTY_CONTAINER";
        case SYSTEM_ERROR_COMMUNICATION:    return "COMMUNICATION";
        case SYSTEM_ERROR_MEMORY:           return "MEMORY";
        case SYSTEM_ERROR_UNKNOWN:          return "UNKNOWN";
        default:                            return "UNDEFINED";
    }
}

int main(void)
{
    /* Initialize hardware */
    BOARD_InitClocks();
    BOARD_InitGPIO();
    BOARD_InitPWM();
    BOARD_InitUART();
    BOARD_InitI2C();
    
    /* Initial LED test to show system is alive */
    /* Quick LED test to show system is alive - REMOVED BLOCKING DELAY */
    LED_SetColor(true, true, true); /* White */
    for (volatile int i = 0; i < 1000000; i++); /* Simple delay */
    LED_SetColor(false, false, false); /* Off */
    
    Debug_Printf("\r\n\r\nRobo-Bar System Starting...\r\n");
    
    /* Create FreeRTOS synchronization objects */
    xQueueSystemMessages = xQueueCreate(10, sizeof(system_message_t));
    xSemaphoreUART = xSemaphoreCreateMutex();
    xEventGroupSystem = xEventGroupCreate();
    
    /* Create system heartbeat timer */
    xTimerSystemHeartbeat = xTimerCreate("Heartbeat",
                                        pdMS_TO_TICKS(1000),
                                        pdTRUE, /* Auto-reload */
                                        (void *)0,
                                        vTimerCallbackSystemHeartbeat);
    
    /* Verify FreeRTOS objects were created successfully */
    if (xQueueSystemMessages == NULL || xSemaphoreUART == NULL || 
        xEventGroupSystem == NULL || xTimerSystemHeartbeat == NULL) {
        /* Creation failed - indicate with red LED */
        LED_SetColor(true, false, false);
        g_system_state = SYSTEM_STATE_ERROR;
        g_last_error = SYSTEM_ERROR_MEMORY;
        
        /* Infinite loop with error indication */
        while (1) {
            LED_SetColor(true, false, false);   /* Red */
            for (volatile int i = 0; i < 500000; i++);
            LED_SetColor(false, false, false);  /* Off */
            for (volatile int i = 0; i < 500000; i++);
        }
    }
    
    /* Create system initialization task */
    BaseType_t xResult = xTaskCreate(
        vTaskSystemInit,                /* Task function */
        "SystemInit",                   /* Task name */
        256,                           /* Stack size in words */
        NULL,                          /* Task parameters */
        TASK_PRIORITY_SYSTEM_MONITOR,  /* Task priority */
        &xTaskHandleSystemInit         /* Task handle */
    );
    
    if (xResult != pdPASS) {
        LED_SetColor(true, false, false); /* Red error indication */
        g_last_error = SYSTEM_ERROR_MEMORY;
        while (1); /* Halt system */
    }
    
    /* Create LED test task */
    xResult = xTaskCreate(
        vTaskLedTest,                  /* Task function */
        "LedTest",                     /* Task name */
        128,                           /* Stack size in words */
        NULL,                          /* Task parameters */
        TASK_PRIORITY_LED_STATUS,      /* Task priority */
        &xTaskHandleLedTest            /* Task handle */
    );
    
    if (xResult != pdPASS) {
        LED_SetColor(true, false, false); /* Red error indication */
        g_last_error = SYSTEM_ERROR_MEMORY;
        while (1); /* Halt system */
    }
    
    /* Create servo test task */
    xResult = xTaskCreate(
        vTaskServoTest,                /* Task function */
        "ServoTest",                   /* Task name */
        256,                           /* Stack size in words */
        NULL,                          /* Task parameters */
        TASK_PRIORITY_SERVO_CONTROL,   /* Task priority */
        &xTaskHandleServoTest          /* Task handle */
    );
    
    if (xResult != pdPASS) {
        LED_SetColor(true, false, false); /* Red error indication */
        g_last_error = SYSTEM_ERROR_MEMORY;
        while (1); /* Halt system */
    }
    
    /* Create system monitor task */
    xResult = xTaskCreate(
        vTaskSystemMonitor,            /* Task function */
        "SystemMonitor",               /* Task name */
        384,                           /* Stack size in words */
        NULL,                          /* Task parameters */
        TASK_PRIORITY_SYSTEM_MONITOR,  /* Task priority */
        &xTaskHandleSystemMonitor      /* Task handle */
    );
    
    if (xResult != pdPASS) {
        LED_SetColor(true, false, false); /* Red error indication */
        g_last_error = SYSTEM_ERROR_MEMORY;
        while (1); /* Halt system */
    }
    
    /* Start the heartbeat timer */
    if (xTimerStart(xTimerSystemHeartbeat, 0) != pdPASS) {
        LED_SetColor(true, false, false); /* Red error indication */
        g_last_error = SYSTEM_ERROR_HARDWARE_INIT;
        while (1); /* Halt system */
    }
    
    /* Print startup message */
    const char startup_msg[] = "\r\n\r\n"
        "========================================\r\n"
        "    ROBO-BAR SYSTEM STARTING UP...     \r\n"
        "========================================\r\n"
        "Hardware: FRDM-KL25Z (ARM Cortex-M0+)\r\n"
        "Firmware: v1.0.0\r\n"
        "FreeRTOS: Enabled\r\n"
        "Tasks Created: 4\r\n"
        "Heap Available: ";
    
    /* Send startup message character by character */
    for (int i = 0; startup_msg[i] != '\0'; i++) {
        while (!(UART0->S1 & UART_S1_TDRE_MASK));
        UART0->D = startup_msg[i];
    }
    
    /* Print heap size */
    char heap_buffer[32];
    snprintf(heap_buffer, sizeof(heap_buffer), "%u bytes\r\n", 
             (unsigned int)configTOTAL_HEAP_SIZE);
    for (int i = 0; heap_buffer[i] != '\0'; i++) {
        while (!(UART0->S1 & UART_S1_TDRE_MASK));
        UART0->D = heap_buffer[i];
    }
    
    const char startup_end[] = 
        "========================================\r\n"
        "Starting FreeRTOS Scheduler...\r\n\r\n";
    
    for (int i = 0; startup_end[i] != '\0'; i++) {
        while (!(UART0->S1 & UART_S1_TDRE_MASK));
        UART0->D = startup_end[i];
    }
    
    /* Show system ready indication */
    LED_SetColor(false, true, false); /* Green - system ready */
    for (volatile int i = 0; i < 1000000; i++);
    LED_SetColor(false, false, false); /* Off */
    
    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();
    
    /* Scheduler should never return - if we get here, something went wrong */
    LED_SetColor(true, false, false); /* Red error indication */
    g_last_error = SYSTEM_ERROR_UNKNOWN;
    
    /* Emergency error handling */
    const char error_msg[] = "CRITICAL ERROR: FreeRTOS Scheduler Failed!\r\n";
    for (int i = 0; error_msg[i] != '\0'; i++) {
        while (!(UART0->S1 & UART_S1_TDRE_MASK));
        UART0->D = error_msg[i];
    }
    
    /* Infinite error loop with LED indication */
    while (1) {
        LED_SetColor(true, false, false);   /* Red */
        for (volatile int i = 0; i < 250000; i++);
        LED_SetColor(false, false, false);  /* Off */
        for (volatile int i = 0; i < 250000; i++);
        LED_SetColor(true, false, false);   /* Red */
        for (volatile int i = 0; i < 250000; i++);
        LED_SetColor(false, false, false);  /* Off */
        for (volatile int i = 0; i < 1000000; i++); /* Longer pause */
    }
}

/*******************************************************************************
 * FreeRTOS Hook Functions
 ******************************************************************************/

/* Stack Overflow Hook */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    
    /* Set error state */
    g_last_error = SYSTEM_ERROR_MEMORY;
    g_system_state = SYSTEM_STATE_ERROR;
    
    /* Indicate stack overflow with rapid red blinks */
    while (1) {
        LED_SetColor(true, false, false);
        for (volatile int i = 0; i < 100000; i++);
        LED_SetColor(false, false, false);
        for (volatile int i = 0; i < 100000; i++);
    }
}

/* Malloc Failed Hook */
void vApplicationMallocFailedHook(void)
{
    /* Set error state */
    g_last_error = SYSTEM_ERROR_MEMORY;
    g_system_state = SYSTEM_STATE_ERROR;
    
    /* Indicate malloc failure with slow red blinks */
    while (1) {
        LED_SetColor(true, false, false);
        for (volatile int i = 0; i < 1000000; i++);
        LED_SetColor(false, false, false);
        for (volatile int i = 0; i < 1000000; i++);
    }
}

/* Idle Hook - called when no tasks are running */
void vApplicationIdleHook(void)
{
    /* Put MCU into low power mode or perform background tasks */
    __WFI(); /* Wait for interrupt */
}

/* Tick Hook - called on every OS tick */
void vApplicationTickHook(void)
{
    /* Can be used for time-critical operations */
    /* Keep this function minimal to avoid affecting system timing */
}

/*******************************************************************************
 * UART Interrupt Handler
 ******************************************************************************/
void UART0_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    /* Check if receive interrupt */
    if (UART0->S1 & UART_S1_RDRF_MASK) {
        uint8_t received_char = UART0->D;
        
        /* Simple command processing */
        switch (received_char) {
            case 'r':
            case 'R':
                /* Reset command */
                NVIC_SystemReset();
                break;
                
            case 's':
            case 'S':
                /* Status command - set event to print status */
                if (xEventGroupSystem) {
                    xEventGroupSetBitsFromISR(xEventGroupSystem, (1 << 1), 
                                            &xHigherPriorityTaskWoken);
                }
                break;
                
            case 'h':
            case 'H':
                /* Help command */
                break;
                
            default:
                /* Echo received character */
                while (!(UART0->S1 & UART_S1_TDRE_MASK));
                UART0->D = received_char;
                break;
        }
    }
    
    /* Check for errors */
    if (UART0->S1 & (UART_S1_OR_MASK | UART_S1_NF_MASK | UART_S1_FE_MASK | UART_S1_PF_MASK)) {
        /* Clear error flags by reading data register */
        (void)UART0->D;
    }
    
    /* Yield to higher priority task if necessary */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
