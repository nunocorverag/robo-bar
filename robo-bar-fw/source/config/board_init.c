/* board_init.c - Robo-Bar System Initialization Functions */

#include "board_init.h"

void BOARD_InitClocks(void)
{
    const mcg_config_t mcgConfig = {
        .mcgMode = kMCG_ModeFEI,
        .irclkEnableMode = kMCG_IrclkEnable,
        .ircs = kMCG_IrcSlow,
        .fcrdiv = 0x01U,
    };

    CLOCK_SetMcgConfig(&mcgConfig);
    CLOCK_SetSimSafeDivs();

    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_PortD);
    CLOCK_EnableClock(kCLOCK_PortE);

    CLOCK_EnableClock(kCLOCK_Uart0);
}

void BOARD_InitGPIO(void)
{
    gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 0};

    PORT_SetPinMux(LED_RED_PORT, LED_RED_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(LED_GREEN_PORT, LED_GREEN_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(LED_BLUE_PORT, LED_BLUE_PIN, kPORT_MuxAsGpio);

    GPIO_PinInit(LED_RED_GPIO, LED_RED_PIN, &gpio_config);
    GPIO_PinInit(LED_GREEN_GPIO, LED_GREEN_PIN, &gpio_config);
    GPIO_PinInit(LED_BLUE_GPIO, LED_BLUE_PIN, &gpio_config);

    GPIO_WritePinOutput(LED_RED_GPIO, LED_RED_PIN, 1);
    GPIO_WritePinOutput(LED_GREEN_GPIO, LED_GREEN_PIN, 1);
    GPIO_WritePinOutput(LED_BLUE_GPIO, LED_BLUE_PIN, 1);

    PORT_SetPinMux(PUMP_RELAY_1_PORT, PUMP_RELAY_1_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(PUMP_RELAY_2_PORT, PUMP_RELAY_2_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(PUMP_RELAY_3_PORT, PUMP_RELAY_3_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(DISPENSER_RELAY_PORT, DISPENSER_RELAY_PIN, kPORT_MuxAsGpio);

    GPIO_PinInit(PUMP_RELAY_1_GPIO, PUMP_RELAY_1_PIN, &gpio_config);
    GPIO_PinInit(PUMP_RELAY_2_GPIO, PUMP_RELAY_2_PIN, &gpio_config);
    GPIO_PinInit(PUMP_RELAY_3_GPIO, PUMP_RELAY_3_PIN, &gpio_config);

    gpio_config.pinDirection = kGPIO_DigitalInput;
    gpio_config.outputLogic = 0;

    PORT_SetPinMux(SENSOR_1_PORT, SENSOR_1_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(SENSOR_2_PORT, SENSOR_2_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(SENSOR_3_PORT, SENSOR_3_PIN, kPORT_MuxAsGpio);

    GPIO_PinInit(SENSOR_1_GPIO, SENSOR_1_PIN, &gpio_config);
    GPIO_PinInit(SENSOR_2_GPIO, SENSOR_2_PIN, &gpio_config);
    GPIO_PinInit(SENSOR_3_GPIO, SENSOR_3_PIN, &gpio_config);

    gpio_config.pinDirection = kGPIO_DigitalOutput;

    PORT_SetPinMux(MIXING_IN1_PORT, MIXING_IN1_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(MIXING_IN2_PORT, MIXING_IN2_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(MIXING_ENA_PORT, MIXING_ENA_PIN, kPORT_MuxAsGpio);

    GPIO_PinInit(MIXING_IN1_GPIO, MIXING_IN1_PIN, &gpio_config);
    GPIO_PinInit(MIXING_IN2_GPIO, MIXING_IN2_PIN, &gpio_config);
    GPIO_PinInit(MIXING_ENA_GPIO, MIXING_ENA_PIN, &gpio_config);

    PORT_SetPinMux(KEYPAD_ROW1_PORT, KEYPAD_ROW1_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(KEYPAD_ROW2_PORT, KEYPAD_ROW2_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(KEYPAD_ROW3_PORT, KEYPAD_ROW3_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(KEYPAD_ROW4_PORT, KEYPAD_ROW4_PIN, kPORT_MuxAsGpio);

    GPIO_PinInit(KEYPAD_ROW1_GPIO, KEYPAD_ROW1_PIN, &gpio_config);
    GPIO_PinInit(KEYPAD_ROW2_GPIO, KEYPAD_ROW2_PIN, &gpio_config);
    GPIO_PinInit(KEYPAD_ROW3_GPIO, KEYPAD_ROW3_PIN, &gpio_config);
    GPIO_PinInit(KEYPAD_ROW4_GPIO, KEYPAD_ROW4_PIN, &gpio_config);

    gpio_config.pinDirection = kGPIO_DigitalInput;
    port_pin_config_t pin_config = {
        .pullSelect = kPORT_PullUp,
        .slewRate = kPORT_FastSlewRate,
        .passiveFilterEnable = kPORT_PassiveFilterDisable,
        .driveStrength = kPORT_LowDriveStrength,
        .mux = kPORT_MuxAsGpio,
    };

    PORT_SetPinConfig(KEYPAD_COL1_PORT, KEYPAD_COL1_PIN, &pin_config);
    PORT_SetPinConfig(KEYPAD_COL2_PORT, KEYPAD_COL2_PIN, &pin_config);
    PORT_SetPinConfig(KEYPAD_COL3_PORT, KEYPAD_COL3_PIN, &pin_config);
    PORT_SetPinConfig(KEYPAD_COL4_PORT, KEYPAD_COL4_PIN, &pin_config);

    GPIO_PinInit(KEYPAD_COL1_GPIO, KEYPAD_COL1_PIN, &gpio_config);
    GPIO_PinInit(KEYPAD_COL2_GPIO, KEYPAD_COL2_PIN, &gpio_config);
    GPIO_PinInit(KEYPAD_COL3_GPIO, KEYPAD_COL3_PIN, &gpio_config);
    GPIO_PinInit(KEYPAD_COL4_GPIO, KEYPAD_COL4_PIN, &gpio_config);
}

void BOARD_InitUART(void)
{
    SIM->SCGC4 |= 0x0400;
    SIM->SCGC5 |= 0x0200;
    SIM->SOPT2 |= 0x04000000;

    PORTA->PCR[1] = 0x0200;
    PORTA->PCR[2] = 0x0200;

    UART0->C2 = 0;
    UART0->BDH = 0x00;
    UART0->BDL = 0x17;
    UART0->C4 = 0x0F;
    UART0->C1 = 0x00;
    UART0->C2 = 0x2C;

    NVIC_EnableIRQ(UART0_IRQn);
}

void BOARD_InitI2C(void)
{
    PORT_SetPinMux(LCD_I2C_SDA_PORT, LCD_I2C_SDA_PIN, kPORT_MuxAlt5);
    PORT_SetPinMux(LCD_I2C_SCL_PORT, LCD_I2C_SCL_PIN, kPORT_MuxAlt5);

    port_pin_config_t pin_config = {
        .pullSelect = kPORT_PullUp,
        .slewRate = kPORT_FastSlewRate,
        .passiveFilterEnable = kPORT_PassiveFilterDisable,
        .driveStrength = kPORT_HighDriveStrength,
        .mux = kPORT_MuxAlt5,
    };

    PORT_SetPinConfig(LCD_I2C_SDA_PORT, LCD_I2C_SDA_PIN, &pin_config);
    PORT_SetPinConfig(LCD_I2C_SCL_PORT, LCD_I2C_SCL_PIN, &pin_config);
}

void BOARD_InitAll(void)
{
    BOARD_InitClocks();
    BOARD_InitGPIO();
    BOARD_InitUART();
    BOARD_InitI2C();
}
