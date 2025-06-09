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

int main(void)
{
    /* Initialize hardware */
    BOARD_InitAll();
    
    /* Initial LED test to show system is alive */
    LED_SetColor(true, true, true); /* White */

    Debug_Printf("\r\n\r\nRobo-Bar System Starting...\r\n");

    /* Crear todas las tareas, colas, semáforos y timer */
    FREERTOS_StartTasks();

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

    for (int i = 0; startup_msg[i] != '\0'; i++) {
        while (!(UART0->S1 & UART_S1_TDRE_MASK));
        UART0->D = startup_msg[i];
    }

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

    LED_SetColor(false, true, false); /* Green - system ready */
    for (volatile int i = 0; i < 1000000; i++);
    LED_SetColor(false, false, false); /* Off */

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    /* If scheduler returns, something went wrong */
    LED_SetColor(true, false, false); /* Red error indication */
    g_last_error = SYSTEM_ERROR_UNKNOWN;

    const char error_msg[] = "CRITICAL ERROR: FreeRTOS Scheduler Failed!\r\n";
    for (int i = 0; error_msg[i] != '\0'; i++) {
        while (!(UART0->S1 & UART_S1_TDRE_MASK));
        UART0->D = error_msg[i];
    }

    while (1) {
        LED_SetColor(true, false, false);   /* Red */
        for (volatile int i = 0; i < 250000; i++);
        LED_SetColor(false, false, false);  /* Off */
        for (volatile int i = 0; i < 250000; i++);
        LED_SetColor(true, false, false);   /* Red */
        for (volatile int i = 0; i < 250000; i++);
        LED_SetColor(false, false, false);  /* Off */
        for (volatile int i = 0; i < 1000000; i++);
    }
}