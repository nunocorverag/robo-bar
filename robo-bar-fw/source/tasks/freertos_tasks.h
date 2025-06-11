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
extern TaskHandle_t xTaskHandleSensorRead;  // Nueva tarea agregada

/* RTOS kernel objects */
extern QueueHandle_t xQueueSystemMessages;
extern SemaphoreHandle_t xSemaphoreUART;
extern EventGroupHandle_t xEventGroupSystem;
extern TimerHandle_t xTimerSystemHeartbeat;

/* Counters */
extern uint32_t g_led_blink_count;
extern uint32_t g_servo_test_count;
extern uint32_t g_heartbeat_count;
extern uint32_t g_sensor_read_count;  // Nuevo contador agregado

/* Timer callback */
void vTimerCallbackSystemHeartbeat(TimerHandle_t xTimer);

/* Task creation entry point */
void FREERTOS_StartTasks(void);

#endif // FREERTOS_TASKS_H
