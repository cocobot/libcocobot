#ifndef COCOBOT_VREP_H
#define COCOBOT_VREP_H

#include <stdint.h>

/* Initialization of vrep communication module. Has to be called before any other action
 */
void cocobot_vrep_init(void);

/* Get the position of virtual robot's motors
 * Argument:
 *  - motor_position: pointer to store motor position, where
 *     motor_position[0] corresponds to the left motor and
 *     motor_position[1] corresponds to the right motor
 */
void cocobot_vrep_get_motor_position(int32_t motor_position[2]);

/* Set the speed of virtual robot's motors
 * Argument:
 *  - left_motor_speed: VREP joint velocity for the left motor
 *  - right_motor_speed: VREP joint velocity for the right motor
 */
void cocobot_vrep_set_motor_command(float left_motor_speed, float right_motor_speed);

#endif // COCOBOT_VREP_H
