#include "FreeRTOS.h"
#include "task.h"
#include "source/drivers/lcd_i2c.h"
#include "source/drivers/keypad.h"
#include "source/drivers/servo_control.h"
#include "core/select_beverage_flow.h"
#include <string.h>
#include <stddef.h>  // Para NULL

typedef struct {
    const char* nombre;
    uint8_t servos[4];       // Lista de servos activos (índice base 0)
    uint16_t tiempo_ms;      // Tiempo de apertura
} receta_bebida_t;

static const receta_bebida_t recetas[] = {
    { "Mojito",      {0, 1, 255, 255}, 1500 },
    { "Margarita",   {2, 0, 255, 255}, 1200 },
    { "Cuba Libre",  {3, 1, 2, 255},   1800 },
    { "Vodka Tonic", {0, 2, 3, 1},     2000 },
};
#define NUM_BEBIDAS (sizeof(recetas)/sizeof(recetas[0]))

void vTaskBeverageFlow(void *pvParameters) {
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Selecciona 1-4:");

    keypad_init();
    servo_control_init();  // Inicializa TPM y pines

    while (1) {
        char tecla = keypad_getkey();
        if (tecla >= '1' && tecla <= '4') {
            uint8_t seleccion = tecla - '1';
            const receta_bebida_t* r = &recetas[seleccion];

            lcd_clear();
            lcd_set_cursor(0, 0);
            lcd_print("Preparando:");
            lcd_set_cursor(1, 0);
            lcd_print(r->nombre);

            for (int i = 0; i < 4; i++) {
                if (r->servos[i] < 4) {
                    servo_control_set_angle(r->servos[i], 90); // Abrir
                }
            }

            vTaskDelay(pdMS_TO_TICKS(r->tiempo_ms));

            for (int i = 0; i < 4; i++) {
                if (r->servos[i] < 4) {
                    servo_control_set_angle(r->servos[i], 0); // Cerrar
                }
            }

            lcd_clear();
            lcd_set_cursor(0, 0);
            lcd_print("Bebida lista!");
            vTaskDelay(pdMS_TO_TICKS(2000));
            lcd_clear();
            lcd_set_cursor(0, 0);
            lcd_print("Selecciona 1-4:");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
