/*
 * gpio_config.h
 * 
 * GPIO pin configuration for Robo-Bar project
 * FRDM-KL25Z Development Board
 * 
 * Pin assignments for all system components:
 * - Servo motors (PWM)
 * - Sensors (Digital/Analog inputs)
 * - Motors (H-bridge control)
 * - User interface (Keypad, LCD)
 * - Status LEDs
 */

#ifndef GPIO_CONFIG_H
#define GPIO_CONFIG_H

#include "fsl_gpio.h"
#include "fsl_port.h"
#include "system_config.h"

/*******************************************************************************
 * Pin Definitions - FRDM-KL25Z
 ******************************************************************************/

/* Onboard RGB LED */
#define LED_RED_PORT        PORTB
#define LED_RED_GPIO        GPIOB
#define LED_RED_PIN         18U

#define LED_GREEN_PORT      PORTB
#define LED_GREEN_GPIO      GPIOB
#define LED_GREEN_PIN       19U

#define LED_BLUE_PORT       PORTD
#define LED_BLUE_GPIO       GPIOD
#define LED_BLUE_PIN        1U

/*******************************************************************************
 * Servo Motor Control (PWM Pins)
 ******************************************************************************/
/* 6 Servo motors for liquid dispensing */
#define SERVO_1_PORT        PORTD
#define SERVO_1_GPIO        GPIOD
#define SERVO_1_PIN         4U
#define SERVO_1_PWM         TPM0_CH4

#define SERVO_2_PORT        PORTA
#define SERVO_2_GPIO        GPIOA
#define SERVO_2_PIN         12U
#define SERVO_2_PWM         TPM1_CH0

#define SERVO_3_PORT        PORTA
#define SERVO_3_GPIO        GPIOA
#define SERVO_3_PIN         13U
#define SERVO_3_PWM         TPM1_CH1

#define SERVO_4_PORT        PORTD
#define SERVO_4_GPIO        GPIOD
#define SERVO_4_PIN         5U
#define SERVO_4_PWM         TPM0_CH5

#define SERVO_5_PORT        PORTA
#define SERVO_5_GPIO        GPIOA
#define SERVO_5_PIN         1U
#define SERVO_5_PWM         TPM2_CH0

#define SERVO_6_PORT        PORTA
#define SERVO_6_GPIO        GPIOA
#define SERVO_6_PIN         2U
#define SERVO_6_PWM         TPM2_CH1

/* 360° Direction control servo */
#define DIRECTION_SERVO_PORT    PORTD
#define DIRECTION_SERVO_GPIO    GPIOD
#define DIRECTION_SERVO_PIN     6U
#define DIRECTION_SERVO_PWM     TPM0_CH6

/*******************************************************************************
 * Water Level Sensors (Digital/Analog Inputs)
 ******************************************************************************/
#define SENSOR_1_PORT       PORTC
#define SENSOR_1_GPIO       GPIOC
#define SENSOR_1_PIN        1U

#define SENSOR_2_PORT       PORTC
#define SENSOR_2_GPIO       GPIOC
#define SENSOR_2_PIN        2U

#define SENSOR_3_PORT       PORTC
#define SENSOR_3_GPIO       GPIOC
#define SENSOR_3_PIN        3U

#define SENSOR_4_PORT       PORTC
#define SENSOR_4_GPIO       GPIOC
#define SENSOR_4_PIN        4U

#define SENSOR_5_PORT       PORTC
#define SENSOR_5_GPIO       GPIOC
#define SENSOR_5_PIN        8U

#define SENSOR_6_PORT       PORTC
#define SENSOR_6_GPIO       GPIOC
#define SENSOR_6_PIN        9U

/*******************************************************************************
 * H-Bridge Motor Control
 ******************************************************************************/
/* Conveyor Belt Motor */
#define CONVEYOR_IN1_PORT   PORTE
#define CONVEYOR_IN1_GPIO   GPIOE
#define CONVEYOR_IN1_PIN    20U

#define CONVEYOR_IN2_PORT   PORTE
#define CONVEYOR_IN2_GPIO   GPIOE
#define CONVEYOR_IN2_PIN    21U

#define CONVEYOR_ENA_PORT   PORTE
#define CONVEYOR_ENA_GPIO   GPIOE
#define CONVEYOR_ENA_PIN    22U

/* Mixing Motor */
#define MIXING_IN1_PORT     PORTE
#define MIXING_IN1_GPIO     GPIOE
#define MIXING_IN1_PIN      23U

#define MIXING_IN2_PORT     PORTE
#define MIXING_IN2_GPIO     GPIOE
#define MIXING_IN2_PIN      29U

#define MIXING_ENA_PORT     PORTE
#define MIXING_ENA_GPIO     GPIOE
#define MIXING_ENA_PIN      30U

/*******************************************************************************
 * 4x4 Matrix Keypad
 ******************************************************************************/
/* Keypad Rows (Outputs) */
#define KEYPAD_ROW1_PORT    PORTB
#define KEYPAD_ROW1_GPIO    GPIOB
#define KEYPAD_ROW1_PIN     0U

#define KEYPAD_ROW2_PORT    PORTB
#define KEYPAD_ROW2_GPIO    GPIOB
#define KEYPAD_ROW2_PIN     1U

#define KEYPAD_ROW3_PORT    PORTB
#define KEYPAD_ROW3_GPIO    GPIOB
#define KEYPAD_ROW3_PIN     2U

#define KEYPAD_ROW4_PORT    PORTB
#define KEYPAD_ROW4_GPIO    GPIOB
#define KEYPAD_ROW4_PIN     3U

/* Keypad Columns (Inputs with pull-up) */
#define KEYPAD_COL1_PORT    PORTB
#define KEYPAD_COL1_GPIO    GPIOB
#define KEYPAD_COL1_PIN     8U

#define KEYPAD_COL2_PORT    PORTB
#define KEYPAD_COL2_GPIO    GPIOB
#define KEYPAD_COL2_PIN     9U

#define KEYPAD_COL3_PORT    PORTB
#define KEYPAD_COL3_GPIO    GPIOB
#define KEYPAD_COL3_PIN     10U

#define KEYPAD_COL4_PORT    PORTB
#define KEYPAD_COL4_GPIO    GPIOB
#define KEYPAD_COL4_PIN     11U

/*******************************************************************************
 * 16x2 LCD Display (I2C Interface)
 ******************************************************************************/
/* I2C0 pins for LCD communication */
#define LCD_I2C_SDA_PORT    PORTE
#define LCD_I2C_SDA_GPIO    GPIOE
#define LCD_I2C_SDA_PIN     25U

#define LCD_I2C_SCL_PORT    PORTE
#define LCD_I2C_SCL_GPIO    GPIOE
#define LCD_I2C_SCL_PIN     24U

/*******************************************************************************
 * Additional Sensors (Optional)
 ******************************************************************************/
/* Position sensors for cup detection */
#define POSITION_SENSOR_1_PORT  PORTC
#define POSITION_SENSOR_1_GPIO  GPIOC
#define POSITION_SENSOR_1_PIN   10U

#define POSITION_SENSOR_2_PORT  PORTC
#define POSITION_SENSOR_2_GPIO  GPIOC
#define POSITION_SENSOR_2_PIN   11U

/* Emergency stop button */
#define EMERGENCY_STOP_PORT     PORTC
#define EMERGENCY_STOP_GPIO     GPIOC
#define EMERGENCY_STOP_PIN      12U

/*******************************************************************************
 * GPIO Configuration Structures
 ******************************************************************************/
typedef struct {
    PORT_Type *port;
    GPIO_Type *gpio;
    uint32_t pin;
    port_mux_t mux;
    gpio_pin_direction_t direction;
    uint8_t initial_value;
} gpio_pin_config_extended_t;

typedef struct {
    PORT_Type *port;
    GPIO_Type *gpio;
    uint32_t pin;
    port_mux_t mux;
    bool enable_pullup;
} gpio_input_config_t;

/*******************************************************************************
 * Pin Arrays for Easy Initialization
 ******************************************************************************/
extern const gpio_pin_config_extended_t servo_pins[SERVO_COUNT];
extern const gpio_input_config_t sensor_pins[WATER_LEVEL_SENSORS_COUNT];
extern const gpio_pin_config_extended_t motor_control_pins[];
extern const gpio_pin_config_extended_t keypad_row_pins[KEYPAD_ROWS];
extern const gpio_input_config_t keypad_col_pins[KEYPAD_COLS];
extern const gpio_pin_config_extended_t status_led_pins[3];

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
void gpio_config_init_all(void);
void gpio_config_init_leds(void);
void gpio_config_init_servos(void);
void gpio_config_init_sensors(void);
void gpio_config_init_motors(void);
void gpio_config_init_keypad(void);
void gpio_config_init_lcd_i2c(void);
void gpio_config_init_additional_sensors(void);

/* LED control functions */
void gpio_led_set(uint8_t led_index, bool state);
void gpio_led_toggle(uint8_t led_index);
void gpio_led_set_rgb(bool red, bool green, bool blue);

/* Sensor reading functions */
bool gpio_sensor_read(uint8_t sensor_index);
uint8_t gpio_sensors_read_all(void);

/* Motor control functions */
void gpio_motor_set_direction(uint8_t motor_index, bool forward);
void gpio_motor_stop(uint8_t motor_index);
void gpio_motor_set_enable(uint8_t motor_index, bool enable);

/* Keypad functions */
uint8_t gpio_keypad_scan(void);
bool gpio_keypad_is_pressed(uint8_t row, uint8_t col);

/* Utility functions */
void gpio_pin_set(PORT_Type *port, GPIO_Type *gpio, uint32_t pin, bool state);
bool gpio_pin_read(GPIO_Type *gpio, uint32_t pin);
void gpio_pin_toggle(GPIO_Type *gpio, uint32_t pin);

/*******************************************************************************
 * Pin Index Definitions
 ******************************************************************************/
/* LED indices */
#define LED_RED_INDEX       0
#define LED_GREEN_INDEX     1
#define LED_BLUE_INDEX      2

/* Servo indices */
#define SERVO_1_INDEX       0
#define SERVO_2_INDEX       1
#define SERVO_3_INDEX       2
#define SERVO_4_INDEX       3
#define SERVO_5_INDEX       4
#define SERVO_6_INDEX       5
#define DIRECTION_SERVO_INDEX 6

/* Motor indices */
#define CONVEYOR_MOTOR_INDEX    0
#define MIXING_MOTOR_INDEX      1

/* Sensor indices */
#define WATER_SENSOR_1_INDEX    0
#define WATER_SENSOR_2_INDEX    1
#define WATER_SENSOR_3_INDEX    2
#define WATER_SENSOR_4_INDEX    3
#define WATER_SENSOR_5_INDEX    4
#define WATER_SENSOR_6_INDEX    5

#endif /* GPIO_CONFIG_H */