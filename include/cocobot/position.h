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

/* Get the angle (modulo 360) value
 * Return
 *  Angle (modulo 360) of the robot in deg
 */
float cocobot_position_get_angle_mod360(void);

#endif// COCOBOT_POSITION_H
