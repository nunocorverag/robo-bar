#include "freertos_tasks.h"
#include <stdio.h>
#include <stdarg.h>

/* Variables globales */
TaskHandle_t xTaskHandleSystemInit = NULL;
TaskHandle_t xTaskHandleLedTest = NULL;
TaskHandle_t xTaskHandleServoTest = NULL;
TaskHandle_t xTaskHandleSystemMonitor = NULL;
TaskHandle_t xTaskHandleSensorRead = NULL;  // Nueva variable agregada

QueueHandle_t xQueueSystemMessages = NULL;
SemaphoreHandle_t xSemaphoreUART = NULL;
EventGroupHandle_t xEventGroupSystem = NULL;
TimerHandle_t xTimerSystemHeartbeat = NULL;

uint32_t g_led_blink_count = 0;
uint32_t g_servo_test_count = 0;
uint32_t g_heartbeat_count = 0;
uint32_t g_sensor_read_count = 0;  // Nuevo contador agregado

/* Callback: Heartbeat */
void vTimerCallbackSystemHeartbeat(TimerHandle_t xTimer) {
    g_heartbeat_count++;
    static bool heartbeat_state = false;
    heartbeat_state = !heartbeat_state;
    GPIO_WritePinOutput(LED_GREEN_GPIO, LED_GREEN_PIN, !heartbeat_state);
}