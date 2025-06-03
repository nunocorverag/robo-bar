/*
 * pwm_config.h
 * 
 * PWM configuration for servo motor control
 * FRDM-KL25Z Development Board - Robo-Bar project
 * 
 * Configures TPM (Timer/PWM Module) channels for:
 * - 6 liquid dispensing servos
 * - 1 direction control servo (360°)
 */

#ifndef PWM_CONFIG_H
#define PWM_CONFIG_H

#include "fsl_tpm.h"
#include "fsl_clock.h"
#include "system_config.h"

/*******************************************************************************
 * PWM Configuration Constants
 ******************************************************************************/
#define PWM_FREQUENCY_HZ            SERVO_PWM_FREQUENCY_HZ    /* 50 Hz for servos */
#define PWM_PERIOD_TICKS            (SYSTEM_BUS_CLOCK_HZ / PWM_FREQUENCY_HZ)

/* Servo pulse widths in microseconds */
#define SERVO_PULSE_MIN_US          1000U    /* 1ms - 0° position */
#define SERVO_PULSE_MAX_US          2000U    /* 2ms - 180° position */
#define SERVO_PULSE_NEUTRAL_US      1500U    /* 1.5ms - 90° position */

/* Convert microseconds to timer ticks */
#define US_TO_TICKS(us)             ((us * SYSTEM_BUS_CLOCK_HZ) / 1000000U)

/* Servo angle limits */
#define SERVO_ANGLE_MIN             0U
#define SERVO_ANGLE_MAX             180U
#define SERVO_ANGLE_NEUTRAL         90U

/* Direction servo special values (360° servo) */
#define DIRECTION_SERVO_STOP        SERVO_PULSE_NEUTRAL_US
#define DIRECTION_SERVO_CW_SLOW     1600U    /* Clockwise slow */
#define DIRECTION_SERVO_CW_FAST     2000U    /* Clockwise fast */
#define DIRECTION_SERVO_CCW_SLOW    1400U    /* Counter-clockwise slow */
#define DIRECTION_SERVO_CCW_FAST    1000U    /* Counter-clockwise fast */

/*******************************************************************************
 * TPM Module Configuration
 ******************************************************************************/
typedef struct {
    TPM_Type *tpm_base;
    uint8_t channel;
    clock_name_t clock_name;
    clock_ip_name_t clock_ip_name;
} pwm_channel_config_t;

/*******************************************************************************
 * Servo Configuration Structure
 ******************************************************************************/
typedef struct {
    uint8_t servo_id;
    TPM_Type *tpm_base;
    uint8_t channel;
    uint16_t current_pulse_us;
    uint8_t current_angle;
    bool is_360_servo;
} servo_config_t;

/*******************************************************************************
 * PWM Channel Definitions
 ******************************************************************************/
/* TPM0 Channels */
#define SERVO_1_TPM                 TPM0
#define SERVO_1_CHANNEL             4U
#define SERVO_1_CLOCK_NAME          kCLOCK_Tpm0
#define SERVO_1_CLOCK_IP            kCLOCK_Tpm0

#define SERVO_4_TPM                 TPM0
#define SERVO_4_CHANNEL             5U
#define SERVO_4_CLOCK_NAME          kCLOCK_Tpm0
#define SERVO_4_CLOCK_IP            kCLOCK_Tpm0

#define DIRECTION_SERVO_TPM         TPM0
#define DIRECTION_SERVO_CHANNEL     6U
#define DIRECTION_SERVO_CLOCK_NAME  kCLOCK_Tpm0
#define DIRECTION_SERVO_CLOCK_IP    kCLOCK_Tpm0

/* TPM1 Channels */
#define SERVO_2_TPM                 TPM1
#define SERVO_2_CHANNEL             0U
#define SERVO_2_CLOCK_NAME          kCLOCK_Tpm1
#define SERVO_2_CLOCK_IP            kCLOCK_Tpm1

#define SERVO_3_TPM                 TPM1
#define SERVO_3_CHANNEL             1U
#define SERVO_3_CLOCK_NAME          kCLOCK_Tpm1
#define SERVO_3_CLOCK_IP            kCLOCK_Tpm1

/* TPM2 Channels */
#define SERVO_5_TPM                 TPM2
#define SERVO_5_CHANNEL             0U
#define SERVO_5_CLOCK_NAME          kCLOCK_Tpm2
#define SERVO_5_CLOCK_IP            kCLOCK_Tpm2

#define SERVO_6_TPM                 TPM2
#define SERVO_6_CHANNEL             1U
#define SERVO_6_CLOCK_NAME          kCLOCK_Tpm2
#define SERVO_6_CLOCK_IP            kCLOCK_Tpm2

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

/* Initialization functions */
void pwm_config_init_all(void);
void pwm_config_init_tpm_module(TPM_Type *base, clock_name_t clock_name, clock_ip_name_t clock_ip);
void pwm_config_init_servo_channel(uint8_t servo_id);

/* Servo control functions */
void pwm_servo_set_angle(uint8_t servo_id, uint8_t angle);
void pwm_servo_set_pulse_width(uint8_t servo_id, uint16_t pulse_us);
uint8_t pwm_servo_get_angle(uint8_t servo_id);
uint16_t pwm_servo_get_pulse_width(uint8_t servo_id);

/* Direction servo functions (360° servo) */
void pwm_direction_servo_stop(void);
void pwm_direction_servo_rotate_cw(uint8_t speed);  /* speed: 0-100 */
void pwm_direction_servo_rotate_ccw(uint8_t speed); /* speed: 0-100 */

/* Utility functions */
void pwm_servo_enable(uint8_t servo_id);
void pwm_servo_disable(uint8_t servo_id);
void pwm_servo_enable_all(void);
void pwm_servo_disable_all(void);
bool pwm_servo_is_valid_id(uint8_t servo_id);

/* Calibration functions */
void pwm_servo_calibrate(uint8_t servo_id, uint16_t min_pulse_us, uint16_t max_pulse_us);
void pwm_servo_set_neutral_position(uint8_t servo_id);
void pwm_servo_sweep_test(uint8_t servo_id);

/*******************************************************************************
 * Servo Index Definitions
 ******************************************************************************/
#define PWM_SERVO_1_INDEX           0
#define PWM_SERVO_2_INDEX           1
#define PWM_SERVO_3_INDEX           2
#define PWM_SERVO_4_INDEX           3
#define PWM_SERVO_5_INDEX           4
#define PWM_SERVO_6_INDEX           5
#define PWM_DIRECTION_SERVO_INDEX   6

/*******************************************************************************
 * External Variables
 ******************************************************************************/
extern servo_config_t g_servo_configs[SERVO_COUNT + 1]; /* +1 for direction servo */

#endif /* PWM_CONFIG_H */