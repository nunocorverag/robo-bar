/*
 * Robo-Bar System - Main Application
 * FRDM-KL25Z Development Board
 */
#include "tasks/freertos_tasks.h"
#include "config/board_init.h"
#include "core/startup_flow.h"

// Variables globales necesarias (ya declaradas en freertos_tasks.h)
extern SemaphoreHandle_t xSemaphoreUART;
extern EventGroupHandle_t xEventGroupSystem;
extern TaskHandle_t xTaskHandleSystemMonitor;
extern TaskHandle_t xTaskHandleSensorRead;

#define STARTUP_COMPLETE_BIT (1 << 1)  // Bit para indicar que el flujo de arranque ha finalizado

void vTaskSequenceManager(void *pvParameters)
{
    Debug_Printf("=== Starting Sequence Manager ===\r\n");
    
    // Paso 1: Ejecutar StartupFlow_Task y esperar a que termine
    Debug_Printf("=== Starting Startup Flow ===\r\n");
   
    // Crear y ejecutar la tarea de startup (sin parámetros adicionales)
    TaskHandle_t xStartupTaskHandle = NULL;
    BaseType_t xResult = xTaskCreate(StartupFlow_Task, "StartupFlow", 512, NULL,
                                   tskIDLE_PRIORITY + 3, &xStartupTaskHandle);
    
    if (xResult != pdPASS) {
        Debug_Printf("ERROR: Failed to create StartupFlow task\r\n");
        LED_SetColor(true, false, false); // Rojo - Error
        vTaskDelete(NULL);
        return;
    }
   
    // Esperar a que el startup flow termine (usando el EventGroup del sistema)
    EventBits_t uxBits = xEventGroupWaitBits(
        xEventGroupSystem,
        STARTUP_COMPLETE_BIT,
        pdFALSE,  // No limpiar el bit
        pdTRUE,   // Esperar a que esté set
        pdMS_TO_TICKS(30000)  // Timeout de 30 segundos
    );
   
    if (uxBits & STARTUP_COMPLETE_BIT) {
        Debug_Printf("Startup Flow completed successfully!\r\n");
        Debug_Printf("=== Starting Additional Tasks ===\r\n");
       
        // Paso 2: Crear las tareas adicionales
        
        // Tarea de monitoreo del sistema
        xResult = xTaskCreate(vTaskSystemMonitor, "SystemMonitor", 384, NULL,
                            TASK_PRIORITY_SYSTEM_MONITOR, &xTaskHandleSystemMonitor);
        if (xResult != pdPASS) {
            Debug_Printf("ERROR: Failed to create SystemMonitor task\r\n");
        } else {
            Debug_Printf("SystemMonitor task created\r\n");
        }
       
        // Tarea de lectura de sensores
        xResult = xTaskCreate(vTaskSensorRead, "SensorRead", 256, NULL,
                            TASK_PRIORITY_LED_STATUS, &xTaskHandleSensorRead);
        if (xResult != pdPASS) {
            Debug_Printf("ERROR: Failed to create SensorRead task\r\n");
        } else {
            Debug_Printf("SensorRead task created\r\n");
        }
       
        Debug_Printf("Additional tasks created and running\r\n");
        Debug_Printf("=== System Ready for Operation ===\r\n");
        
        // Indicar que el sistema está listo con LED verde
        LED_SetColor(false, true, false); // Verde - Listo
        
    } else {
        Debug_Printf("ERROR: Startup Flow failed to complete (timeout)!\r\n");
        LED_SetColor(true, false, false); // Rojo - Error
    }
   
    // Esta tarea ya cumplió su propósito, se puede eliminar
    Debug_Printf("Sequence Manager task completing\r\n");
    vTaskDelete(NULL);
}

int main(void)
{
    BaseType_t xResult;
    
    // Inicializar hardware (incluyendo UART)
    BOARD_InitAll();
   
    // Crear los objetos de FreeRTOS necesarios PRIMERO
    xSemaphoreUART = xSemaphoreCreateMutex();
    if (xSemaphoreUART == NULL) {
        printf("ERROR: Failed to create UART semaphore\r\n");
        while(1);
    }
   
    // Crear el grupo de eventos del sistema
    xEventGroupSystem = xEventGroupCreate();
    if (xEventGroupSystem == NULL) {
        printf("ERROR: Failed to create system event group\r\n");
        while(1);
    }
   
    // Crear otros objetos de FreeRTOS necesarios
    // TODO: Definir system_message_t en el header apropiado si se va a usar
    /*
    typedef struct {
        uint8_t type;
        uint32_t data;
    } system_message_t;
    
    xQueueSystemMessages = xQueueCreate(10, sizeof(system_message_t));
    if (xQueueSystemMessages == NULL) {
        printf("ERROR: Failed to create system messages queue\r\n");
        while(1);
    }
    */
    
    // Crear timer de heartbeat
    xTimerSystemHeartbeat = xTimerCreate("Heartbeat",
                                        pdMS_TO_TICKS(1000),
                                        pdTRUE,
                                        (void *)0,
                                        vTimerCallbackSystemHeartbeat);
    if (xTimerSystemHeartbeat == NULL) {
        printf("ERROR: Failed to create heartbeat timer\r\n");
        while(1);
    }
   
    // Prueba inicial con LED
    LED_SetColor(true, true, false); // Amarillo - Inicializando
   
    // Ahora sí podemos usar Debug_Printf porque el semáforo ya existe
    Debug_Printf("\r\n\r\n=== Robo-Bar System Starting ===\r\n");
    Debug_Printf("System: %s v%d.%d.%d\r\n", SYSTEM_NAME,
                 SYSTEM_VERSION_MAJOR, SYSTEM_VERSION_MINOR, SYSTEM_VERSION_PATCH);
    Debug_Printf("MCU: FRDM-KL25Z (ARM Cortex-M0+)\r\n");
    Debug_Printf("Hardware initialization complete\r\n");
   
    // Iniciar el timer de heartbeat
    if (xTimerStart(xTimerSystemHeartbeat, 0) != pdPASS) {
        Debug_Printf("ERROR: Failed to start heartbeat timer\r\n");
        while(1);
    }
   
    // todo: uncomment master task creation when ready
    // Crear la tarea maestra que maneja la secuencia
    // xResult = xTaskCreate(vTaskSequenceManager, "SequenceManager", 512, NULL,
    //                      tskIDLE_PRIORITY + 1, NULL);
    // if (xResult != pdPASS) {
    //     Debug_Printf("ERROR: Failed to create SequenceManager task\r\n");
    //     while(1);
    // }

    // HARDCODED for debugging purposes
    // Inicializar el flujo de inicio sin tareas
    Debug_Printf("\r\n\r\n=== Robo-Bar System Starting ===\r\n");
    if (!StartupFlow_Init()) {
        printf("ERROR: StartupFlow_Init failed\r\n");
        LED_SetColor(true, false, false); // Rojo - Error
        while(1);
    }

    // Ejecutar directamente el flujo de inicio paso a paso
    if (StartupFlow_Execute()) {
        Debug_Printf("StartupFlow completed successfully\r\n");
        LED_SetColor(false, true, false); // Verde - Listo
    } else {
        Debug_Printf("ERROR: StartupFlow failed\r\n");
        LED_SetColor(true, false, false); // Rojo - Error
    }
   
    Debug_Printf("Starting FreeRTOS scheduler...\r\n");
   
    // Iniciar el scheduler
    vTaskStartScheduler();
   
    // Si llegamos aquí, el scheduler falló
    LED_SetColor(true, false, false);
    printf("ERROR: FreeRTOS scheduler failed to start!\r\n");
   
    while (1) {
        LED_SetColor(true, false, false);
        for (volatile int i = 0; i < 250000; i++);
        LED_SetColor(false, false, false);
        for (volatile int i = 0; i < 250000; i++);
    }
}

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

LED_SetColor(bool red, bool green, bool blue) {
    GPIO_WritePinOutput(LED_RED_GPIO, LED_RED_PIN, !red);
    GPIO_WritePinOutput(LED_GREEN_GPIO, LED_GREEN_PIN, !green);
    GPIO_WritePinOutput(LED_BLUE_GPIO, LED_BLUE_PIN, !blue);
}