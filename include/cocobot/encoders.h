#ifndef COCOBOT_ENCODERS_H
#define COCOBOT_ENCODERS_H

#include <stdint.h>

/* Get the position of robot's motors
 * Argument:
 *  - motor_position: pointer to store motor position, where
 *     motor_position[0] corresponds to the left motor and
 *     motor_position[1] corresponds to the right motor
 */
void cocobot_encoders_get_motor_position(int32_t motor_position[2]);

#endif // COCOBOT_ENCODERS_H
