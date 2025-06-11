// /*
//  * main_select_beverage_flow.c
//  */

// #include <stddef.h>
// #include "FreeRTOS.h"
// #include "task.h"
// #include "source/drivers/lcd_i2c.h"
// #include "source/drivers/keypad.h"
// #include "source/drivers/servo_control.h"
// #include "core/select_beverage_flow.h"

// int main(void) {
//     BOARD_InitAll();
//     i2c_init();
//     lcd_init();
//     lcd_clear();

//     xTaskCreate(vTaskBeverageFlow, "BebidaFlow", 512, NULL, 2, NULL);
//     vTaskStartScheduler();

//     while (1); // fallback
// }

