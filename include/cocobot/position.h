#ifndef COCOBOT_POSITION_H
#define COCOBOT_POSITION_H

/* Initialization of the position module. Need to be called before any other action 
 * Argument:
 *  - task_priority: FreeRTOS task priority for position thread
 */
void cocobot_position_init(unsigned int task_priority);

/* Get the X coord
 * Return:
 *  X coord of the robot in mm
 */
float cocobot_position_get_x(void);

/* Get the Y coord
 * Return:
 *  Y coord of the robot in mm
 */
float cocobot_position_get_y(void);

/* Get the distance value
 * Return:
 *  Curvilinear distance of the robot in mm
 */
float cocobot_position_get_distance(void);

/* Get the angle value
 * Return
 *  Absolute (no modulo) angle of the robot in deg
 */
float cocobot_position_get_angle(void);

/* Get the distance speed value
 * Return:
 *  Distance speed of the robot in mm / 10ms
 */
float cocobot_position_get_speed_distance(void);

/* Get the anglar speed value
 * Return
 *  Absolute angular speed of the robot in deg / 10ms
 */
float cocobot_position_get_speed_angle(void);

/* Set the speed of virtual robot's motors
 * Argument:
 *  - left_motor_speed: VREP joint velocity for the left motor
 *  - right_motor_speed: VREP joint velocity for the right motor
 */
void cocobot_position_set_motor_command(float left_motor_speed, float right_motor_speed);

/* Set the speed of the virtual robot's motors using distance and angle commmand
 * Argument:
 *  - linear_speed: command for the virtual robot linear speed
 *  - angular_velocity: command for the virtual robot angular velocity
 */
void cocobot_position_set_speed_distance_angle(float linear_speed, float angular_velocity);

/* Handle console user command related to position module
 * Argument:
 *  - command: requested command
 * Return:
 *  0 : if command is not reconized
 *  1 : if command has been successfully handled
 */
int cocobot_position_handle_console(char * command);

/* Send asynchronously debug informations if user has requested them
 */
void cocobot_position_handle_async_console(void);

/* Set the X coord
 * Argument:
 *  x: new X coord of the robot in mm
 */
void cocobot_position_set_x(float x);

/* Set the Y coord
 * Argument:
 *  y: new Y coord of the robot in mm
 */
void cocobot_position_set_y(float y);

/* Set the angle value
 * Argument:
 *  angle: new angle value of the robot in deg
 */
void cocobot_position_set_angle(float angle);


#endif// COCOBOT_POSITION_H
