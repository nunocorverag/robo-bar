/*
 * gpio_config.c
 * 
 * GPIO configuration implementation for Robo-Bar project
 * FRDM-KL25Z Development Board
 */

#include "gpio_config.h"
#include "system_config.h"
#include "fsl_clock.h"

/*******************************************************************************
 * Pin Configuration Arrays
 ******************************************************************************/

/* Servo motor pins configuration */
const gpio_pin_config_extended_t servo_pins[SERVO_COUNT] = {
    {SERVO_1_PORT, SERVO_1_GPIO, SERVO_1_PIN, kPORT_MuxAlt4, kGPIO_DigitalOutput, 0},
    {SERVO_2_PORT, SERVO_2_GPIO, SERVO_2_PIN, kPORT_MuxAlt3, kGPIO_DigitalOutput, 0},
    {SERVO_3_PORT, SERVO_3_GPIO, SERVO_3_PIN, kPORT_MuxAlt3, kGPIO_DigitalOutput, 0},
    {SERVO_4_PORT, SERVO_4_GPIO, SERVO_4_PIN, kPORT_MuxAlt4, kGPIO_DigitalOutput, 0},
};

/* Water level sensor pins configuration */
const gpio_input_config_t sensor_pins[WATER_LEVEL_SENSORS_COUNT] = {
    {SENSOR_1_PORT, SENSOR_1_GPIO, SENSOR_1_PIN, kPORT_MuxAsGpio, true},
    {SENSOR_2_PORT, SENSOR_2_GPIO, SENSOR_2_PIN, kPORT_MuxAsGpio, true},
    {SENSOR_3_PORT, SENSOR_3_GPIO, SENSOR_3_PIN, kPORT_MuxAsGpio, true},
    {SENSOR_4_PORT, SENSOR_4_GPIO, SENSOR_4_PIN, kPORT_MuxAsGpio, true},
    {SENSOR_5_PORT, SENSOR_5_GPIO, SENSOR_5_PIN, kPORT_MuxAsGpio, true},
    {SENSOR_6_PORT, SENSOR_6_GPIO, SENSOR_6_PIN, kPORT_MuxAsGpio, true}
};

/* Motor control pins configuration */
const gpio_pin_config_extended_t motor_control_pins[] = {
    {MIXING_IN1_PORT, MIXING_IN1_GPIO, MIXING_IN1_PIN, kPORT_MuxAsGpio, kGPIO_DigitalOutput, 0},
    {MIXING_IN2_PORT, MIXING_IN2_GPIO, MIXING_IN2_PIN, kPORT_MuxAsGpio, kGPIO_DigitalOutput, 0},
    {MIXING_ENA_PORT, MIXING_ENA_GPIO, MIXING_ENA_PIN, kPORT_MuxAsGpio, kGPIO_DigitalOutput, 0},
};

/* Keypad row pins configuration */
const gpio_pin_config_extended_t keypad_row_pins[KEYPAD_ROWS] = {
    {KEYPAD_ROW1_PORT, KEYPAD_ROW1_GPIO, KEYPAD_ROW1_PIN, kPORT_MuxAsGpio, kGPIO_DigitalOutput, 1},
    {KEYPAD_ROW2_PORT, KEYPAD_ROW2_GPIO, KEYPAD_ROW2_PIN, kPORT_MuxAsGpio, kGPIO_DigitalOutput, 1},
    {KEYPAD_ROW3_PORT, KEYPAD_ROW3_GPIO, KEYPAD_ROW3_PIN, kPORT_MuxAsGpio, kGPIO_DigitalOutput, 1},
    {KEYPAD_ROW4_PORT, KEYPAD_ROW4_GPIO, KEYPAD_ROW4_PIN, kPORT_MuxAsGpio, kGPIO_DigitalOutput, 1}
};

/* Keypad column pins configuration */
const gpio_input_config_t keypad_col_pins[KEYPAD_COLS] = {
    {KEYPAD_COL1_PORT, KEYPAD_COL1_GPIO, KEYPAD_COL1_PIN, kPORT_MuxAsGpio, true},
    {KEYPAD_COL2_PORT, KEYPAD_COL2_GPIO, KEYPAD_COL2_PIN, kPORT_MuxAsGpio, true},
    {KEYPAD_COL3_PORT, KEYPAD_COL3_GPIO, KEYPAD_COL3_PIN, kPORT_MuxAsGpio, true},
    {KEYPAD_COL4_PORT, KEYPAD_COL4_GPIO, KEYPAD_COL4_PIN, kPORT_MuxAsGpio, true}
};

/* Status LED pins configuration */
const gpio_pin_config_extended_t status_led_pins[3] = {
    {LED_RED_PORT, LED_RED_GPIO, LED_RED_PIN, kPORT_MuxAsGpio, kGPIO_DigitalOutput, 1},     /* Red LED (off initially) */
    {LED_GREEN_PORT, LED_GREEN_GPIO, LED_GREEN_PIN, kPORT_MuxAsGpio, kGPIO_DigitalOutput, 1}, /* Green LED (off initially) */
    {LED_BLUE_PORT, LED_BLUE_GPIO, LED_BLUE_PIN, kPORT_MuxAsGpio, kGPIO_DigitalOutput, 1}    /* Blue LED (off initially) */
};

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/*!
 * @brief Enable clock for a specific port
 * @param port Port to enable clock for
 */
static void gpio_enable_port_clock(PORT_Type *port)
{
    if (port == PORTA)
    {
        CLOCK_EnableClock(kCLOCK_PortA);
    }
    else if (port == PORTB)
    {
        CLOCK_EnableClock(kCLOCK_PortB);
    }
    else if (port == PORTC)
    {
        CLOCK_EnableClock(kCLOCK_PortC);
    }
    else if (port == PORTD)
    {
        CLOCK_EnableClock(kCLOCK_PortD);
    }
    else if (port == PORTE)
    {
        CLOCK_EnableClock(kCLOCK_PortE);
    }
}

/*!
 * @brief Configure a single GPIO pin
 * @param config Pin configuration structure
 */
static void gpio_configure_pin(const gpio_pin_config_extended_t *config)
{
    port_pin_config_t port_config = {0};
    gpio_pin_config_t gpio_config = {0};
    
    /* Enable port clock */
    gpio_enable_port_clock(config->port);
    
    /* Configure port pin */
    port_config.pullSelect = kPORT_PullDisable;
    port_config.slewRate = kPORT_FastSlewRate;
    port_config.passiveFilterEnable = kPORT_PassiveFilterDisable;
    port_config.openDrainEnable = kPORT_OpenDrainDisable;
    port_config.driveStrength = kPORT_LowDriveStrength;
    port_config.mux = config->mux;
    
    PORT_SetPinConfig(config->port, config->pin, &port_config);
    
    /* Configure GPIO pin */
    gpio_config.pinDirection = config->direction;
    gpio_config.outputLogic = config->initial_value;
    
    GPIO_PinInit(config->gpio, config->pin, &gpio_config);
}

/*!
 * @brief Configure a single input GPIO pin with pull-up option
 * @param config Input pin configuration structure
 */
static void gpio_configure_input_pin(const gpio_input_config_t *config)
{
    port_pin_config_t port_config = {0};
    gpio_pin_config_t gpio_config = {0};
    
    /* Enable port clock */
    gpio_enable_port_clock(config->port);
    
    /* Configure port pin */
    port_config.pullSelect = config->enable_pullup ? kPORT_PullUp : kPORT_PullDisable;
    port_config.slewRate = kPORT_FastSlewRate;
    port_config.passiveFilterEnable = kPORT_PassiveFilterDisable;
    port_config.openDrainEnable = kPORT_OpenDrainDisable;
    port_config.driveStrength = kPORT_LowDriveStrength;
    port_config.mux = config->mux;
    
    PORT_SetPinConfig(config->port, config->pin, &port_config);
    
    /* Configure GPIO pin as input */
    gpio_config.pinDirection = kGPIO_DigitalInput;
    gpio_config.outputLogic = 0;
    
    GPIO_PinInit(config->gpio, config->pin, &gpio_config);
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/*!
 * @brief Initialize all GPIO configurations
 */
void gpio_config_init_all(void)
{
    gpio_config_init_leds();
    gpio_config_init_servos();
    gpio_config_init_sensors();
    gpio_config_init_motors();
    gpio_config_init_keypad();
    gpio_config_init_lcd_i2c();
    gpio_config_init_emergency_stop();
}

/*!
 * @brief Initialize status LED pins
 */
void gpio_config_init_leds(void)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gpio_configure_pin(&status_led_pins[i]);
    }
}

/*!
 * @brief Initialize servo motor pins
 */
void gpio_config_init_servos(void)
{
    const gpio_pin_config_extended_t direction_servo_config = {
        DIRECTION_SERVO_PORT, DIRECTION_SERVO_GPIO, DIRECTION_SERVO_PIN, 
        kPORT_MuxAlt4, kGPIO_DigitalOutput, 0
    };
    
    /* Initialize liquid dispensing servos */
    for (uint8_t i = 0; i < SERVO_COUNT; i++)
    {
        gpio_configure_pin(&servo_pins[i]);
    }
    
    /* Initialize direction control servo */
    gpio_configure_pin(&direction_servo_config);
}

/*!
 * @brief Initialize sensor pins
 */
void gpio_config_init_sensors(void)
{
    for (uint8_t i = 0; i < WATER_LEVEL_SENSORS_COUNT; i++)
    {
        gpio_configure_input_pin(&sensor_pins[i]);
    }
}

/*!
 * @brief Initialize motor control pins
 */
void gpio_config_init_motors(void)
{
    uint8_t motor_pins_count = sizeof(motor_control_pins) / sizeof(motor_control_pins[0]);
    
    for (uint8_t i = 0; i < motor_pins_count; i++)
    {
        gpio_configure_pin(&motor_control_pins[i]);
    }
}

/*!
 * @brief Initialize keypad pins
 */
void gpio_config_init_keypad(void)
{
    /* Initialize row pins (outputs) */
    for (uint8_t i = 0; i < KEYPAD_ROWS; i++)
    {
        gpio_configure_pin(&keypad_row_pins[i]);
    }
    
    /* Initialize column pins (inputs with pull-up) */
    for (uint8_t i = 0; i < KEYPAD_COLS; i++)
    {
        gpio_configure_input_pin(&keypad_col_pins[i]);
    }
}

/*!
 * @brief Initialize LCD I2C pins
 */
void gpio_config_init_lcd_i2c(void)
{
    port_pin_config_t port_config = {0};
    
    /* Enable PORTC clock - CAMBIAR de PORTE a PORTC */
    CLOCK_EnableClock(kCLOCK_PortC);
    
    /* Configure I2C pins */
    port_config.pullSelect = kPORT_PullUp;
    port_config.slewRate = kPORT_FastSlewRate;
    port_config.passiveFilterEnable = kPORT_PassiveFilterDisable;
    port_config.openDrainEnable = kPORT_OpenDrainEnable;
    port_config.driveStrength = kPORT_LowDriveStrength;
    port_config.mux = kPORT_MuxAlt2; /* CAMBIAR de kPORT_MuxAlt5 a kPORT_MuxAlt2 para PORTC I2C */
    
    /* Configure SDA pin */
    PORT_SetPinConfig(LCD_I2C_SDA_PORT, LCD_I2C_SDA_PIN, &port_config);
    
    /* Configure SCL pin */
    PORT_SetPinConfig(LCD_I2C_SCL_PORT, LCD_I2C_SCL_PIN, &port_config);
}
/*!
 * @brief Initialize additional sensor pins
 */
void gpio_config_init_emergency_stop(void)
{
    gpio_input_config_t emergency_stop_config = {
        EMERGENCY_STOP_PORT, EMERGENCY_STOP_GPIO, EMERGENCY_STOP_PIN, 
        kPORT_MuxAsGpio, true
    };
    
    /* Configure emergency stop button */
    gpio_configure_input_pin(&emergency_stop_config);
    
    // ELIMINAR position sensors
}
/*******************************************************************************
 * LED Control Functions
 ******************************************************************************/

/*!
 * @brief Set LED state
 * @param led_index LED index (0=Red, 1=Green, 2=Blue)
 * @param state LED state (true=on, false=off)
 */
void gpio_led_set(uint8_t led_index, bool state)
{
    if (led_index < 3)
    {
        const gpio_pin_config_extended_t *led = &status_led_pins[led_index];
        /* Note: FRDM-KL25Z LEDs are active low */
        GPIO_PinWrite(led->gpio, led->pin, state ? 0 : 1);
    }
}

/*!
 * @brief Toggle LED state
 * @param led_index LED index (0=Red, 1=Green, 2=Blue)
 */
void gpio_led_toggle(uint8_t led_index)
{
    if (led_index < 3)
    {
        const gpio_pin_config_extended_t *led = &status_led_pins[led_index];
        GPIO_PortToggle(led->gpio, 1U << led->pin);
    }
}

/*!
 * @brief Set RGB LED combination
 * @param red Red LED state
 * @param green Green LED state
 * @param blue Blue LED state
 */
void gpio_led_set_rgb(bool red, bool green, bool blue)
{
    gpio_led_set(LED_RED_INDEX, red);
    gpio_led_set(LED_GREEN_INDEX, green);
    gpio_led_set(LED_BLUE_INDEX, blue);
}

/*******************************************************************************
 * Sensor Functions
 ******************************************************************************/

/*!
 * @brief Read sensor state
 * @param sensor_index Sensor index (0-5)
 * @return Sensor state (true=active, false=inactive)
 */
bool gpio_sensor_read(uint8_t sensor_index)
{
    if (sensor_index < WATER_LEVEL_SENSORS_COUNT)
    {
        const gpio_input_config_t *sensor = &sensor_pins[sensor_index];
        return (GPIO_PinRead(sensor->gpio, sensor->pin) == 0); /* Active low */
    }
    return false;
}

/*!
 * @brief Read all sensors
 * @return Sensor states as bit field (bit 0 = sensor 0, etc.)
 */
uint8_t gpio_sensors_read_all(void)
{
    uint8_t sensor_states = 0;
    
    for (uint8_t i = 0; i < WATER_LEVEL_SENSORS_COUNT; i++)
    {
        if (gpio_sensor_read(i))
        {
            sensor_states |= (1U << i);
        }
    }
    
    return sensor_states;
}

/*******************************************************************************
 * Motor Control Functions
 ******************************************************************************/

/*!
 * @brief Set motor direction
 * @param motor_index Motor index (0=Conveyor, 1=Mixing)
 * @param forward Direction (true=forward, false=reverse)
 */
void gpio_motor_set_direction(uint8_t motor_index, bool forward)
{
    if (motor_index == CONVEYOR_MOTOR_INDEX)
    {
        GPIO_PinWrite(CONVEYOR_IN1_GPIO, CONVEYOR_IN1_PIN, forward ? 1 : 0);
        GPIO_PinWrite(CONVEYOR_IN2_GPIO, CONVEYOR_IN2_PIN, forward ? 0 : 1);
    }
    else if (motor_index == MIXING_MOTOR_INDEX)
    {
        GPIO_PinWrite(MIXING_IN1_GPIO, MIXING_IN1_PIN, forward ? 1 : 0);
        GPIO_PinWrite(MIXING_IN2_GPIO, MIXING_IN2_PIN, forward ? 0 : 1);
    }
}

/*!
 * @brief Stop motor
 * @param motor_index Motor index (0=Conveyor, 1=Mixing)
 */
void gpio_motor_stop(uint8_t motor_index)
{
    if (motor_index == CONVEYOR_MOTOR_INDEX)
    {
        GPIO_PinWrite(CONVEYOR_IN1_GPIO, CONVEYOR_IN1_PIN, 0);
        GPIO_PinWrite(CONVEYOR_IN2_GPIO, CONVEYOR_IN2_PIN, 0);
    }
    else if (motor_index == MIXING_MOTOR_INDEX)
    {
        GPIO_PinWrite(MIXING_IN1_GPIO, MIXING_IN1_PIN, 0);
        GPIO_PinWrite(MIXING_IN2_GPIO, MIXING_IN2_PIN, 0);
    }
}

/*!
 * @brief Enable/disable motor
 * @param motor_index Motor index (0=Conveyor, 1=Mixing)
 * @param enable Enable state (true=enable, false=disable)
 */
void gpio_motor_set_enable(uint8_t motor_index, bool enable)
{
    if (motor_index == CONVEYOR_MOTOR_INDEX)
    {
        GPIO_PinWrite(CONVEYOR_ENA_GPIO, CONVEYOR_ENA_PIN, enable ? 1 : 0);
    }
    else if (motor_index == MIXING_MOTOR_INDEX)
    {
        GPIO_PinWrite(MIXING_ENA_GPIO, MIXING_ENA_PIN, enable ? 1 : 0);
    }
}

/*******************************************************************************
 * Keypad Functions
 ******************************************************************************/

/*!
 * @brief Scan keypad for pressed keys
 * @return Key code (0-15 for keys, 0xFF for no key pressed)
 */
uint8_t gpio_keypad_scan(void)
{
    uint8_t key_code = 0xFF; /* No key pressed */
    
    for (uint8_t row = 0; row < KEYPAD_ROWS; row++)
    {
        /* Set current row low, others high */
        for (uint8_t r = 0; r < KEYPAD_ROWS; r++)
        {
            const gpio_pin_config_extended_t *row_pin = &keypad_row_pins[r];
            GPIO_PinWrite(row_pin->gpio, row_pin->pin, (r == row) ? 0 : 1);
        }
        
        /* Small delay for signal stabilization */
        for (volatile uint32_t i = 0; i < 1000; i++);
        
        /* Check columns */
        for (uint8_t col = 0; col < KEYPAD_COLS; col++)
        {
            const gpio_input_config_t *col_pin = &keypad_col_pins[col];
            if (GPIO_PinRead(col_pin->gpio, col_pin->pin) == 0) /* Key pressed */
            {
                key_code = (row * KEYPAD_COLS) + col;
                break;
            }
        }
        
        if (key_code != 0xFF)
        {
            break;
        }
    }
    
    /* Set all rows high */
    for (uint8_t r = 0; r < KEYPAD_ROWS; r++)
    {
        const gpio_pin_config_extended_t *row_pin = &keypad_row_pins[r];
        GPIO_PinWrite(row_pin->gpio, row_pin->pin, 1);
    }
    
    return key_code;
}

/*!
 * @brief Check if specific key is pressed
 * @param row Row index (0-3)
 * @param col Column index (0-3)
 * @return true if key is pressed, false otherwise
 */
bool gpio_keypad_is_pressed(uint8_t row, uint8_t col)
{
    if (row >= KEYPAD_ROWS || col >= KEYPAD_COLS)
    {
        return false;
    }
    
    /* Set target row low, others high */
    for (uint8_t r = 0; r < KEYPAD_ROWS; r++)
    {
        const gpio_pin_config_extended_t *row_pin = &keypad_row_pins[r];
        GPIO_PinWrite(row_pin->gpio, row_pin->pin, (r == row) ? 0 : 1);
    }
    
    /* Small delay for signal stabilization */
    for (volatile uint32_t i = 0; i < 1000; i++);
    
    /* Check target column */
    const gpio_input_config_t *col_pin = &keypad_col_pins[col];
    bool is_pressed = (GPIO_PinRead(col_pin->gpio, col_pin->pin) == 0);
    
    /* Set all rows high */
    for (uint8_t r = 0; r < KEYPAD_ROWS; r++)
    {
        const gpio_pin_config_extended_t *row_pin = &keypad_row_pins[r];
        GPIO_PinWrite(row_pin->gpio, row_pin->pin, 1);
    }
    
    return is_pressed;
}

/*******************************************************************************
 * Utility Functions
 ******************************************************************************/

/*!
 * @brief Set GPIO pin state
 * @param port Port type
 * @param gpio GPIO type
 * @param pin Pin number
 * @param state Pin state (true=high, false=low)
 */
void gpio_pin_set(PORT_Type *port, GPIO_Type *gpio, uint32_t pin, bool state)
{
    (void)port; /* Unused parameter */
    GPIO_PinWrite(gpio, pin, state ? 1 : 0);
}

/*!
 * @brief Read GPIO pin state
 * @param gpio GPIO type
 * @param pin Pin number
 * @return Pin state (true=high, false=low)
 */
bool gpio_pin_read(GPIO_Type *gpio, uint32_t pin)
{
    return (GPIO_PinRead(gpio, pin) != 0);
}

/*!
 * @brief Toggle GPIO pin state
 * @param gpio GPIO type
 * @param pin Pin number
 */
void gpio_pin_toggle(GPIO_Type *gpio, uint32_t pin)
{
    GPIO_PortToggle(gpio, 1U << pin);
}