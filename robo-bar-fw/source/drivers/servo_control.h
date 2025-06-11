#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdint.h>

void servo_control_init(void);
void servo_control_set_angle(uint8_t servo_id, uint8_t angle);

#endif
