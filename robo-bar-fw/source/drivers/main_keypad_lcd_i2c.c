// #include "MKL25Z4.h"
// #include "keypad.h"
// #include "lcd_i2c.h"
// #include <stdio.h>

// void mostrar_menu(void);
// void preparar_bebida(const char *nombre);

// const char* bebidas[9] = {
//     "Mojito", "Margarita", "BloodyMary",
//     "CubaLibre", "Martini", "TequilaSun",
//     "WhiskyCola", "VodkaTonic", "GinLemon"
// };

// int main(void) {
//     keypad_init();
//     i2c_init();
//     lcd_init();
//     lcd_clear();

//     int estado = 0;
//     int seleccion = 0;
//     int menu_mostrado = 0; // bandera para mostrar el menú solo una vez


//     while (1) {
//         char tecla = keypad_getkey();
//         if (estado == 0) {
//                 if (!menu_mostrado) {
//                     mostrar_menu();
//                     menu_mostrado = 1;
//                 }

//                 if (tecla >= '1' && tecla <= '9') {
//                     seleccion = tecla - '1';
//                     lcd_clear();
//                     lcd_set_cursor(0, 0);
//                     lcd_print("Presiona * para");
//                     lcd_set_cursor(1, 0);
//                     lcd_print("hacer bebida");
//                     estado = 1;
//                     menu_mostrado = 0; // resetea bandera para la próxima vez
//                 }
//         } else if (estado == 1) {
//                if (tecla == '*') {
//                    preparar_bebida(bebidas[seleccion]);
//                    estado = 0;
//                } else if (tecla == '#') {
//                    lcd_clear();
//                    estado = 0;
//                }
//            }
//        }
//     }

// void mostrar_menu(void) {
//     lcd_clear();
//     lcd_set_cursor(0, 0);
//     lcd_print("1.Moj 2.Marg 3.BM");
//     lcd_set_cursor(1, 0);
//     lcd_print("4.CL 5.Mar 6.Teq");
//     Delay(2000);
//     lcd_clear();
//     lcd_set_cursor(0, 0);
//     lcd_print("7.WC 8.VT 9.GL");
//     lcd_set_cursor(1, 0);
//     lcd_print("*=OK  #=Menu");
//     Delay(2000);
// }

// void preparar_bebida(const char *nombre) {
//     lcd_clear();
//     lcd_set_cursor(0, 0);
//     lcd_print("Preparando:");
//     lcd_set_cursor(1, 0);
//     lcd_print(nombre);
//     for (int i = 0; i <= 100; i += 10) {
//         char prog[5];
//         sprintf(prog, "%3d%%", i);
//         lcd_set_cursor(1, 12);
//         lcd_print(prog);
//         Delay(500);
//     }
//     lcd_clear();
//     lcd_set_cursor(0, 0);
//     lcd_print("Listo!");
//     Delay(2000);
//     lcd_clear();
// }
