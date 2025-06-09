#include "freertos_tasks.h"
#include <stdio.h>
#include <stdarg.h>

/* Variables globales */
TaskHandle_t xTaskHandleSystemInit = NULL;
TaskHandle_t xTaskHandleLedTest = NULL;
TaskHandle_t xTaskHandleServoTest = NULL;
TaskHandle_t xTaskHandleSystemMonitor = NULL;

QueueHandle_t xQueueSystemMessages = NULL;
SemaphoreHandle_t xSemaphoreUART = NULL;
EventGroupHandle_t xEventGroupSystem = NULL;
TimerHandle_t xTimerSystemHeartbeat = NULL;

uint32_t g_led_blink_count = 0;
uint32_t g_servo_test_count = 0;
uint32_t g_heartbeat_count = 0;

typedef struct {
    uint8_t type;
    uint32_t data;
} system_message_t;

Debug_Printf(const char* format, ...) {
    if (xSemaphoreUART != NULL) {
        if (xSemaphoreTake(xSemaphoreUART, pdMS_TO_TICKS(100)) == pdTRUE) {
            va_list args;
            char buffer[256];

            va_start(args, format);
            vsnprintf(buffer, sizeof(buffer), format, args);
            va_end(args);

            for (int i = 0; buffer[i] != '\0'; i++) {
                while (!(UART0->S1 & UART_S1_TDRE_MASK));
                UART0->D = buffer[i];
            }

            xSemaphoreGive(xSemaphoreUART);
        }
    }
}

/* Funciones de control LED */
LED_SetColor(bool red, bool green, bool blue) {
    GPIO_WritePinOutput(LED_RED_GPIO, LED_RED_PIN, !red);
    GPIO_WritePinOutput(LED_GREEN_GPIO, LED_GREEN_PIN, !green);
    GPIO_WritePinOutput(LED_BLUE_GPIO, LED_BLUE_PIN, !blue);
}

static void LED_Test_Sequence(void) {
    LED_SetColor(true, false, false);
    vTaskDelay(pdMS_TO_TICKS(500));

    LED_SetColor(false, true, false);
    vTaskDelay(pdMS_TO_TICKS(500));

    LED_SetColor(false, false, true);
    vTaskDelay(pdMS_TO_TICKS(500));

    LED_SetColor(false, false, false);
    vTaskDelay(pdMS_TO_TICKS(500));
}

/* Tarea: Inicialización del sistema */
void vTaskSystemInit(void *pvParameters) {
    Debug_Printf("\r\n=== Robo-Bar System Initialization ===\r\n");
    Debug_Printf("System: %s v%d.%d.%d\r\n", SYSTEM_NAME,
                 SYSTEM_VERSION_MAJOR, SYSTEM_VERSION_MINOR, SYSTEM_VERSION_PATCH);
    Debug_Printf("MCU: FRDM-KL25Z (ARM Cortex-M0+)\r\n");
    Debug_Printf("Core Clock: %d Hz\r\n", SYSTEM_CORE_CLOCK_HZ);
    Debug_Printf("Bus Clock: %d Hz\r\n", SYSTEM_BUS_CLOCK_HZ);

    g_system_state = SYSTEM_STATE_IDLE;

    Debug_Printf("System initialization complete!\r\n");

    if (xEventGroupSystem) {
        xEventGroupSetBits(xEventGroupSystem, (1 << 0));
    }

    vTaskDelete(NULL);
}

/* Tarea: Prueba de LEDs */
void vTaskLedTest(void *pvParameters) {
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

/* Servo Angle → Duty Cycle */
static uint8_t Servo_AngleToDutyCycle(uint16_t angle_degrees) {
    if (angle_degrees > 180) angle_degrees = 180;
    return 5 + (angle_degrees * 5) / 180;
}

/* Establecer ángulo de un servo */
static void Servo_SetAngle(uint8_t servo_id, uint16_t angle_degrees) {
    uint8_t duty = Servo_AngleToDutyCycle(angle_degrees);

    switch (servo_id) {
        case 1: TPM_UpdatePwmDutycycle(TPM0, kTPM_Chnl_4, kTPM_EdgeAlignedPwm, duty); break;
        case 2: TPM_UpdatePwmDutycycle(TPM0, kTPM_Chnl_5, kTPM_EdgeAlignedPwm, duty); break;
        case 3: TPM_UpdatePwmDutycycle(TPM1, kTPM_Chnl_0, kTPM_EdgeAlignedPwm, duty); break;
        case 4: TPM_UpdatePwmDutycycle(TPM1, kTPM_Chnl_1, kTPM_EdgeAlignedPwm, duty); break;
        case 5: TPM_UpdatePwmDutycycle(TPM2, kTPM_Chnl_0, kTPM_EdgeAlignedPwm, duty); break;
        default: Debug_Printf("Invalid servo ID %u\r\n", servo_id); break;
    }
}

/* Tarea: Prueba de Servos */
void vTaskServoTest(void *pvParameters) {
    uint16_t angle = 90;
    bool up = true;
    uint8_t current = 1;
    Debug_Printf("Servo Test Task started\r\n");

    for (uint8_t i = 1; i <= 5; i++) {
        Servo_SetAngle(i, 90);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    while (1) {
        angle = up ? angle + 15 : angle - 15;

        if (angle >= 180) {
            angle = 180;
            up = false;
        } else if (angle <= 0) {
            angle = 0;
            up = true;
            current = current >= 5 ? 1 : current + 1;
            g_servo_test_count++;
            Debug_Printf("Testing Servo %u - Cycle %lu\r\n", current, g_servo_test_count);
        }

        Servo_SetAngle(current, angle);

        if (angle % 45 == 0) {
            Debug_Printf("Servo %u -> %u° (DC: %u%%)\r\n", current, angle, Servo_AngleToDutyCycle(angle));
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* Tarea: Monitor del sistema */
void vTaskSystemMonitor(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    Debug_Printf("System Monitor Task started\r\n");

    while (1) {
        g_system_uptime_seconds++;

        if (g_system_uptime_seconds % 10 == 0) {
            Debug_Printf("=== System Status ===\r\n");
            Debug_Printf("Uptime: %lu s\r\n", g_system_uptime_seconds);
            Debug_Printf("State: %s\r\n", system_state_to_string(g_system_state));
            Debug_Printf("Free Heap: %u bytes\r\n", xPortGetFreeHeapSize());
            Debug_Printf("LED Cycles: %lu\r\n", g_led_blink_count);
            Debug_Printf("Servo Cycles: %lu\r\n", g_servo_test_count);
            Debug_Printf("Heartbeat: %lu\r\n", g_heartbeat_count);

            Debug_Printf("=====================\r\n\r\n");
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}

/* Callback: Heartbeat */
void vTimerCallbackSystemHeartbeat(TimerHandle_t xTimer) {
    g_heartbeat_count++;
    static bool heartbeat_state = false;
    heartbeat_state = !heartbeat_state;
    GPIO_WritePinOutput(LED_GREEN_GPIO, LED_GREEN_PIN, !heartbeat_state);
}

void FREERTOS_StartTasks(void) {
    BaseType_t xResult;

    xQueueSystemMessages = xQueueCreate(10, sizeof(system_message_t));
    xSemaphoreUART = xSemaphoreCreateMutex();
    xEventGroupSystem = xEventGroupCreate();
    xTimerSystemHeartbeat = xTimerCreate("Heartbeat",
                                        pdMS_TO_TICKS(1000),
                                        pdTRUE,
                                        (void *)0,
                                        vTimerCallbackSystemHeartbeat);

    if (!xQueueSystemMessages || !xSemaphoreUART || !xEventGroupSystem || !xTimerSystemHeartbeat) {
        g_last_error = SYSTEM_ERROR_MEMORY;
        g_system_state = SYSTEM_STATE_ERROR;
        LED_SetColor(true, false, false);  // Red
        while (1);
    }

    xResult = xTaskCreate(vTaskSystemInit, "SystemInit", 256, NULL, TASK_PRIORITY_SYSTEM_MONITOR, &xTaskHandleSystemInit);
    if (xResult != pdPASS) while (1);

    xResult = xTaskCreate(vTaskLedTest, "LedTest", 128, NULL, TASK_PRIORITY_LED_STATUS, &xTaskHandleLedTest);
    if (xResult != pdPASS) while (1);

    xResult = xTaskCreate(vTaskServoTest, "ServoTest", 256, NULL, TASK_PRIORITY_SERVO_CONTROL, &xTaskHandleServoTest);
    if (xResult != pdPASS) while (1);

    xResult = xTaskCreate(vTaskSystemMonitor, "SystemMonitor", 384, NULL, TASK_PRIORITY_SYSTEM_MONITOR, &xTaskHandleSystemMonitor);
    if (xResult != pdPASS) while (1);

    if (xTimerStart(xTimerSystemHeartbeat, 0) != pdPASS) while (1);
}
