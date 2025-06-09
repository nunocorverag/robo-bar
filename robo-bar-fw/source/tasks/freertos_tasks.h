#ifndef FREERTOS_TASKS_H
#define FREERTOS_TASKS_H

#include "FreeRTOS.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>

#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "event_groups.h"
#include "../config/gpio_config.h"
#include "../config/pwm_config.h"
#include "../config/system_config.h"

/* Task handles */
extern TaskHandle_t xTaskHandleSystemInit;
extern TaskHandle_t xTaskHandleLedTest;
extern TaskHandle_t xTaskHandleServoTest;
extern TaskHandle_t xTaskHandleSystemMonitor;

/* RTOS kernel objects */
extern QueueHandle_t xQueueSystemMessages;
extern SemaphoreHandle_t xSemaphoreUART;
extern EventGroupHandle_t xEventGroupSystem;
extern TimerHandle_t xTimerSystemHeartbeat;

/* Counters */
extern uint32_t g_led_blink_count;
extern uint32_t g_servo_test_count;
extern uint32_t g_heartbeat_count;

/* Funciones auxiliares accesibles fuera de freertos_tasks.c */
void Debug_Printf(const char* format, ...);
void LED_SetColor(bool red, bool green, bool blue);

/* Task functions */
void vTaskSystemInit(void *pvParameters);
void vTaskLedTest(void *pvParameters);
void vTaskServoTest(void *pvParameters);
void vTaskSystemMonitor(void *pvParameters);

/* Timer callback */
void vTimerCallbackSystemHeartbeat(TimerHandle_t xTimer);

/* Task creation entry point */
void FREERTOS_StartTasks(void);

#endif // FREERTOS_TASKS_H
