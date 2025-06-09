/*
 * pwm_config.c
 * 
 * PWM configuration implementation for servo motor control
 * FRDM-KL25Z Development Board - Robo-Bar project
 */

#include "pwm_config.h"
#include "gpio_config.h"
#include <string.h>

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
servo_config_t g_servo_configs[SERVO_COUNT + 1]; /* +1 for direction servo */

/*******************************************************************************
 * Private Variables
 ******************************************************************************/
static const pwm_channel_config_t pwm_channels[] = {
    /* Servo 1 - TPM0 CH4 */
    {SERVO_1_TPM, SERVO_1_CHANNEL, SERVO_1_CLOCK_NAME, SERVO_1_CLOCK_IP},
    /* Servo 2 - TPM1 CH0 */
    {SERVO_2_TPM, SERVO_2_CHANNEL, SERVO_2_CLOCK_NAME, SERVO_2_CLOCK_IP},
    /* Servo 3 - TPM1 CH1 */
    {SERVO_3_TPM, SERVO_3_CHANNEL, SERVO_3_CLOCK_NAME, SERVO_3_CLOCK_IP},
    /* Servo 4 - TPM0 CH5 */
    {SERVO_4_TPM, SERVO_4_CHANNEL, SERVO_4_CLOCK_NAME, SERVO_4_CLOCK_IP},
    /* Servo 5 - TPM2 CH0 */
    /* Direction Servo - TPM0 CH6 */
    {DIRECTION_SERVO_TPM, DIRECTION_SERVO_CHANNEL, DIRECTION_SERVO_CLOCK_NAME, DIRECTION_SERVO_CLOCK_IP}
};

static bool tpm_modules_initialized[3] = {false, false, false}; /* TPM0, TPM1, TPM2 */

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/*!
 * @brief Get TPM module index for initialization tracking
 * @param base TPM base address
 * @return TPM module index (0-2) or 0xFF if invalid
 */
static uint8_t get_tpm_module_index(TPM_Type *base)
{
    if (base == TPM0) return 0;
    if (base == TPM1) return 1;
    if (base == TPM2) return 2;
    return 0xFF;
}

/*!
 * @brief Convert angle to pulse width in microseconds
 * @param angle Servo angle (0-180 degrees)
 * @return Pulse width in microseconds
 */
static uint16_t angle_to_pulse_us(uint8_t angle)
{
    if (angle > SERVO_ANGLE_MAX)
        angle = SERVO_ANGLE_MAX;
    
    /* Linear interpolation: pulse = min + (angle / 180) * (max - min) */
    return SERVO_PULSE_MIN_US + 
           ((uint32_t)angle * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US)) / SERVO_ANGLE_MAX;
}

/*!
 * @brief Convert pulse width to angle
 * @param pulse_us Pulse width in microseconds
 * @return Servo angle (0-180 degrees)
 */
static uint8_t pulse_us_to_angle(uint16_t pulse_us)
{
    if (pulse_us < SERVO_PULSE_MIN_US)
        pulse_us = SERVO_PULSE_MIN_US;
    if (pulse_us > SERVO_PULSE_MAX_US)
        pulse_us = SERVO_PULSE_MAX_US;
    
    /* Linear interpolation: angle = (pulse - min) * 180 / (max - min) */
    return ((uint32_t)(pulse_us - SERVO_PULSE_MIN_US) * SERVO_ANGLE_MAX) / 
           (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);
}

/*!
 * @brief Update TPM channel duty cycle
 * @param base TPM base address
 * @param channel TPM channel number
 * @param pulse_us Desired pulse width in microseconds
 */
static void update_tpm_duty_cycle(TPM_Type *base, uint8_t channel, uint16_t pulse_us)
{
    uint32_t duty_ticks = US_TO_TICKS(pulse_us);
    TPM_UpdatePwmDutycycle(base, (tpm_chnl_t)channel, kTPM_EdgeAlignedPwm, duty_ticks);
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/*!
 * @brief Initialize all PWM configurations
 */
void pwm_config_init_all(void)
{
    /* Initialize servo configurations */
    memset(g_servo_configs, 0, sizeof(g_servo_configs));
    
    /* Configure each servo */
    for (uint8_t i = 0; i < SERVO_COUNT; i++)
    {
        g_servo_configs[i].servo_id = i;
        g_servo_configs[i].tpm_base = pwm_channels[i].tpm_base;
        g_servo_configs[i].channel = pwm_channels[i].channel;
        g_servo_configs[i].current_pulse_us = SERVO_PULSE_NEUTRAL_US;
        g_servo_configs[i].current_angle = SERVO_ANGLE_NEUTRAL;
        g_servo_configs[i].is_360_servo = false;
        
        pwm_config_init_servo_channel(i);
    }
    
    /* Configure direction servo (360° servo) */
    g_servo_configs[SERVO_COUNT].servo_id = SERVO_COUNT;
    g_servo_configs[SERVO_COUNT].tpm_base = pwm_channels[SERVO_COUNT].tpm_base;
    g_servo_configs[SERVO_COUNT].channel = pwm_channels[SERVO_COUNT].channel;
    g_servo_configs[SERVO_COUNT].current_pulse_us = DIRECTION_SERVO_STOP;
    g_servo_configs[SERVO_COUNT].current_angle = 0;
    g_servo_configs[SERVO_COUNT].is_360_servo = true;
    
    pwm_config_init_servo_channel(SERVO_COUNT);
    
    /* Set all servos to neutral position */
    pwm_servo_enable_all();
    for (uint8_t i = 0; i < SERVO_COUNT; i++)
    {
        pwm_servo_set_neutral_position(i);
    }
    pwm_direction_servo_stop();
}

/*!
 * @brief Initialize TPM module
 * @param base TPM base address
 * @param clock_name Clock name for the TPM module
 * @param clock_ip Clock IP name for the TPM module
 */
void pwm_config_init_tpm_module(TPM_Type *base, clock_name_t clock_name, clock_ip_name_t clock_ip)
{
    uint8_t tpm_index = get_tpm_module_index(base);
    
    if (tpm_index == 0xFF || tpm_modules_initialized[tpm_index])
        return;
    
    tpm_config_t tpm_config;
    
    /* Enable TPM clock */
    CLOCK_EnableClock(clock_ip);
    CLOCK_SetTpmClock(1U); /* Use MCGIRCLK as TPM clock source */
    
    /* Initialize TPM module */
    TPM_GetDefaultConfig(&tpm_config);
    tpm_config.prescale = kTPM_Prescale_Divide_4; /* Adjust for desired frequency */
    TPM_Init(base, &tpm_config);
    
    /* Set PWM frequency */
    TPM_SetTimerPeriod(base, PWM_PERIOD_TICKS);
    
    tpm_modules_initialized[tpm_index] = true;
}

/*!
 * @brief Initialize specific servo channel
 * @param servo_id Servo ID (0-6)
 */
void pwm_config_init_servo_channel(uint8_t servo_id)
{
    if (!pwm_servo_is_valid_id(servo_id))
        return;
    
    const pwm_channel_config_t *channel_config = &pwm_channels[servo_id];
    tpm_chnl_pwm_signal_param_t pwm_param;
    
    /* Initialize TPM module if not already done */
    pwm_config_init_tpm_module(channel_config->tpm_base, 
                               channel_config->clock_name, 
                               channel_config->clock_name);
    
    /* Configure PWM channel */
    pwm_param.chnlNumber = (tpm_chnl_t)channel_config->channel;
    pwm_param.level = kTPM_HighTrue;
    pwm_param.dutyCyclePercent = 0; /* Will be set later */
    
    TPM_SetupPwm(channel_config->tpm_base, &pwm_param, 1U, kTPM_EdgeAlignedPwm, 
                 PWM_FREQUENCY_HZ, CLOCK_GetFreq(channel_config->clock_name));
}

/*!
 * @brief Set servo angle
 * @param servo_id Servo ID (0-5)
 * @param angle Desired angle (0-180 degrees)
 */
void pwm_servo_set_angle(uint8_t servo_id, uint8_t angle)
{
    if (!pwm_servo_is_valid_id(servo_id) || servo_id >= SERVO_COUNT)
        return;
    
    if (angle > SERVO_ANGLE_MAX)
        angle = SERVO_ANGLE_MAX;
    
    uint16_t pulse_us = angle_to_pulse_us(angle);
    pwm_servo_set_pulse_width(servo_id, pulse_us);
    
    g_servo_configs[servo_id].current_angle = angle;
}

/*!
 * @brief Set servo pulse width
 * @param servo_id Servo ID (0-6)
 * @param pulse_us Pulse width in microseconds
 */
void pwm_servo_set_pulse_width(uint8_t servo_id, uint16_t pulse_us)
{
    if (!pwm_servo_is_valid_id(servo_id))
        return;
    
    servo_config_t *servo = &g_servo_configs[servo_id];
    
    /* Clamp pulse width to valid range */
    if (pulse_us < SERVO_PULSE_MIN_US)
        pulse_us = SERVO_PULSE_MIN_US;
    if (pulse_us > SERVO_PULSE_MAX_US)
        pulse_us = SERVO_PULSE_MAX_US;
    
    /* Update TPM duty cycle */
    update_tpm_duty_cycle(servo->tpm_base, servo->channel, pulse_us);
    
    /* Update servo configuration */
    servo->current_pulse_us = pulse_us;
    if (!servo->is_360_servo)
    {
        servo->current_angle = pulse_us_to_angle(pulse_us);
    }
}

/*!
 * @brief Get current servo angle
 * @param servo_id Servo ID (0-5)
 * @return Current angle (0-180 degrees)
 */
uint8_t pwm_servo_get_angle(uint8_t servo_id)
{
    if (!pwm_servo_is_valid_id(servo_id) || servo_id >= SERVO_COUNT)
        return 0;
    
    return g_servo_configs[servo_id].current_angle;
}

/*!
 * @brief Get current servo pulse width
 * @param servo_id Servo ID (0-6)
 * @return Current pulse width in microseconds
 */
uint16_t pwm_servo_get_pulse_width(uint8_t servo_id)
{
    if (!pwm_servo_is_valid_id(servo_id))
        return 0;
    
    return g_servo_configs[servo_id].current_pulse_us;
}

/*!
 * @brief Stop direction servo
 */
void pwm_direction_servo_stop(void)
{
    pwm_servo_set_pulse_width(PWM_DIRECTION_SERVO_INDEX, DIRECTION_SERVO_STOP);
}

/*!
 * @brief Rotate direction servo clockwise
 * @param speed Speed (0-100%)
 */
void pwm_direction_servo_rotate_cw(uint8_t speed)
{
    if (speed > 100)
        speed = 100;
    
    uint16_t pulse_us = DIRECTION_SERVO_STOP + 
                       ((uint32_t)speed * (DIRECTION_SERVO_CW_FAST - DIRECTION_SERVO_STOP)) / 100;
    
    pwm_servo_set_pulse_width(PWM_DIRECTION_SERVO_INDEX, pulse_us);
}

/*!
 * @brief Rotate direction servo counter-clockwise
 * @param speed Speed (0-100%)
 */
void pwm_direction_servo_rotate_ccw(uint8_t speed)
{
    if (speed > 100)
        speed = 100;
    
    uint16_t pulse_us = DIRECTION_SERVO_STOP - 
                       ((uint32_t)speed * (DIRECTION_SERVO_STOP - DIRECTION_SERVO_CCW_FAST)) / 100;
    
    pwm_servo_set_pulse_width(PWM_DIRECTION_SERVO_INDEX, pulse_us);
}

/*!
 * @brief Enable servo PWM output
 * @param servo_id Servo ID (0-6)
 */
void pwm_servo_enable(uint8_t servo_id)
{
    if (!pwm_servo_is_valid_id(servo_id))
        return;
    
    servo_config_t *servo = &g_servo_configs[servo_id];
    TPM_StartTimer(servo->tpm_base, kTPM_SystemClock);
}

/*!
 * @brief Disable servo PWM output
 * @param servo_id Servo ID (0-6)
 */
void pwm_servo_disable(uint8_t servo_id)
{
    if (!pwm_servo_is_valid_id(servo_id))
        return;
    
    servo_config_t *servo = &g_servo_configs[servo_id];
    update_tpm_duty_cycle(servo->tpm_base, servo->channel, 0);
}

/*!
 * @brief Enable all servo PWM outputs
 */
void pwm_servo_enable_all(void)
{
    for (uint8_t i = 0; i <= SERVO_COUNT; i++)
    {
        pwm_servo_enable(i);
    }
}

/*!
 * @brief Disable all servo PWM outputs
 */
void pwm_servo_disable_all(void)
{
    for (uint8_t i = 0; i <= SERVO_COUNT; i++)
    {
        pwm_servo_disable(i);
    }
}

/*!
 * @brief Check if servo ID is valid
 * @param servo_id Servo ID to check
 * @return true if valid, false otherwise
 */
bool pwm_servo_is_valid_id(uint8_t servo_id)
{
    return (servo_id <= SERVO_COUNT); /* 0-6 are valid (including direction servo) */
}

/*!
 * @brief Calibrate servo pulse range
 * @param servo_id Servo ID (0-5)
 * @param min_pulse_us Minimum pulse width
 * @param max_pulse_us Maximum pulse width
 */
void pwm_servo_calibrate(uint8_t servo_id, uint16_t min_pulse_us, uint16_t max_pulse_us)
{
    /* This is a placeholder for servo calibration */
    /* In a full implementation, this would store calibration values */
    /* and use them in angle_to_pulse_us() calculations */
    (void)servo_id;
    (void)min_pulse_us;
    (void)max_pulse_us;
}

/*!
 * @brief Set servo to neutral position
 * @param servo_id Servo ID (0-5)
 */
void pwm_servo_set_neutral_position(uint8_t servo_id)
{
    if (servo_id < SERVO_COUNT)
    {
        pwm_servo_set_angle(servo_id, SERVO_ANGLE_NEUTRAL);
    }
}

/*!
 * @brief Perform servo sweep test
 * @param servo_id Servo ID (0-5)
 */
void pwm_servo_sweep_test(uint8_t servo_id)
{
    if (servo_id >= SERVO_COUNT)
        return;
    
    /* Simple sweep test - in practice this would be more sophisticated */
    pwm_servo_set_angle(servo_id, 0);
    /* Add delay here in actual implementation */
    pwm_servo_set_angle(servo_id, 90);
    /* Add delay here in actual implementation */
    pwm_servo_set_angle(servo_id, 180);
    /* Add delay here in actual implementation */
    pwm_servo_set_angle(servo_id, 90);
}
