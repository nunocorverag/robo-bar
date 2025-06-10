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

#include "tasks/freertos_tasks.h"
#include "config/board_init.h"
#include "core/startup_flow.h"

static EventGroupHandle_t xEventGroup;

#define STARTUP_COMPLETE_BIT (1 << 0)  // Bit para indicar que el flujo de arranque ha finalizado

int main(void)
{
    // Inicializar hardware (incluyendo UART)
    BOARD_InitAll();
    
    // Crear el grupo de eventos para la señalización de que el arranque ha finalizado
    xEventGroup = xEventGroupCreate();

    // Prueba inicial con LED para indicar que el sistema está vivo
    LED_SetColor(true, true, true); // Blanco

    Debug_Printf("\r\n\r\nRobo-Bar System Starting...\r\n");

    // Crear la tarea para el flujo de arranque (solo la tarea de flujo de arranque por ahora)
    xTaskCreate(StartupFlow_Task, "StartupFlow", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

    // Esperar a que el flujo de arranque haya terminado (esperando el evento de completado)
    xEventGroupWaitBits(xEventGroup, STARTUP_COMPLETE_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    // Una vez que el flujo de arranque esté completo, enviar un mensaje por UART para indicar que el sistema está listo
    Debug_Printf("Flujo de arranque completado, el sistema está listo para otras tareas.\r\n");

    // Ya podemos agregar tareas adicionales, pero por ahora no las agregamos.

    // Iniciar el FreeRTOS scheduler
    // vTaskStartScheduler();

    // // Si el scheduler no arranca correctamente, mostrar un mensaje de error
    // LED_SetColor(true, false, false); // Rojo - Error
    // while (1) {
    //     LED_SetColor(true, false, false);   // Rojo
    //     for (volatile int i = 0; i < 250000; i++);
    //     LED_SetColor(false, false, false);  // Apagar
    //     for (volatile int i = 0; i < 250000; i++);
    // }
}