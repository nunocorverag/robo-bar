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

    CLOCK_EnableClock(kCLOCK_Tpm0);
    CLOCK_EnableClock(kCLOCK_Tpm1);
    CLOCK_EnableClock(kCLOCK_Tpm2);

    CLOCK_EnableClock(kCLOCK_Uart0);
    CLOCK_SetTpmClock(1U);
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

    PORT_SetPinMux(SERVO_1_PORT, SERVO_1_PIN, kPORT_MuxAlt4);
    PORT_SetPinMux(SERVO_2_PORT, SERVO_2_PIN, kPORT_MuxAlt3);
    PORT_SetPinMux(SERVO_3_PORT, SERVO_3_PIN, kPORT_MuxAlt3);
    PORT_SetPinMux(SERVO_4_PORT, SERVO_4_PIN, kPORT_MuxAlt4);
    PORT_SetPinMux(DIRECTION_SERVO_PORT, DIRECTION_SERVO_PIN, kPORT_MuxAlt4);

    gpio_config.pinDirection = kGPIO_DigitalInput;

    PORT_SetPinMux(SENSOR_1_PORT, SENSOR_1_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(SENSOR_2_PORT, SENSOR_2_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(SENSOR_3_PORT, SENSOR_3_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(SENSOR_4_PORT, SENSOR_4_PIN, kPORT_MuxAsGpio);

    GPIO_PinInit(SENSOR_1_GPIO, SENSOR_1_PIN, &gpio_config);
    GPIO_PinInit(SENSOR_2_GPIO, SENSOR_2_PIN, &gpio_config);
    GPIO_PinInit(SENSOR_3_GPIO, SENSOR_3_PIN, &gpio_config);
    GPIO_PinInit(SENSOR_4_GPIO, SENSOR_4_PIN, &gpio_config);

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

void BOARD_InitPWM(void)
{
    tpm_config_t tpmInfo;
    tpm_chnl_pwm_signal_param_t tpmParam[6]; // Array para todos los servos
    
    /* Obtener configuración por defecto */
    TPM_GetDefaultConfig(&tpmInfo);
    
    /* CRÍTICO: Configurar prescaler para obtener exactamente 50Hz */
    /* Con clock de 48MHz y prescaler /16: 48MHz/16 = 3MHz */
    /* Para 50Hz necesitamos: 3MHz/50Hz = 60000 counts */
    tpmInfo.prescale = kTPM_Prescale_Divide_16;
    
    /* Inicializar módulos TPM */
    TPM_Init(TPM0, &tpmInfo);
    TPM_Init(TPM1, &tpmInfo);
    TPM_Init(TPM2, &tpmInfo);
    
    /* Configurar parámetros PWM para servos */
    /* Servo 1 - TPM0 Channel 4 (PTD4) */
    tpmParam[0].chnlNumber = kTPM_Chnl_4;
    tpmParam[0].level = kTPM_HighTrue;
    tpmParam[0].dutyCyclePercent = 7.5; /* 1.5ms pulse = posición central */
    
    /* Servo 2 - TPM0 Channel 5 (PTD5) */
    tpmParam[1].chnlNumber = kTPM_Chnl_5;
    tpmParam[1].level = kTPM_HighTrue;
    tpmParam[1].dutyCyclePercent = 7.5;
    
    /* Servo 3 - TPM1 Channel 0 (PTA12) */
    tpmParam[2].chnlNumber = kTPM_Chnl_0;
    tpmParam[2].level = kTPM_HighTrue;
    tpmParam[2].dutyCyclePercent = 7.5;
    
    /* Servo 4 - TPM1 Channel 1 (PTA13) */
    tpmParam[3].chnlNumber = kTPM_Chnl_1;
    tpmParam[3].level = kTPM_HighTrue;
    tpmParam[3].dutyCyclePercent = 7.5;
    
    /* Direction Servo - TPM2 Channel 0 (PTE20) */
    tpmParam[4].chnlNumber = kTPM_Chnl_0;
    tpmParam[4].level = kTPM_HighTrue;
    tpmParam[4].dutyCyclePercent = 7.5;
    
    /* Setup PWM con frecuencia exacta de 50Hz */
    uint32_t pwm_freq = CLOCK_GetFreq(kCLOCK_PllFllSelClk);
    
    /* TPM0 - Servos 1 y 2 */
    TPM_SetupPwm(TPM0, &tpmParam[0], 2U, kTPM_EdgeAlignedPwm, 50U, pwm_freq);
    
    /* TPM1 - Servos 3 y 4 */  
    TPM_SetupPwm(TPM1, &tpmParam[2], 2U, kTPM_EdgeAlignedPwm, 50U, pwm_freq);
    
    /* TPM2 - Direction Servo */
    TPM_SetupPwm(TPM2, &tpmParam[4], 1U, kTPM_EdgeAlignedPwm, 50U, pwm_freq);
    
    /* IMPORTANTE: Iniciar los módulos TPM */
    TPM_StartTimer(TPM0, kTPM_SystemClock);
    TPM_StartTimer(TPM1, kTPM_SystemClock);
    TPM_StartTimer(TPM2, kTPM_SystemClock);
}

void BOARD_InitAll(void)
{
    BOARD_InitClocks();
    BOARD_InitGPIO();
    BOARD_InitPWM();
    BOARD_InitUART();
    BOARD_InitI2C();
}
