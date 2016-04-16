#ifndef COCOBOT_ASSERV_RAMP_H
#define COCOBOT_ASSERV_RAMP_H

//internal structure
typedef struct
{
  float max_speed;
  float max_accel;

  float position_target;
  float position_current;
  float position_feedback;
  float speed_target;
} cocobot_asserv_ramp_t;


/* Initialization of the ramp module. Need to be called before any other action 
 * Argument:
 *  - ramp: a valid cocobot_asserv_ramp_t pointer
 */
void cocobot_asserv_ramp_init(cocobot_asserv_ramp_t * ramp);

/* Reset ramp values to current position (will set speed to 0) for hard stop (opponent detected)
 * Argument:
 *  - ramp: a valid cocobot_asserv_ramp_t pointer
 *  - current_position: current position (in mm or deg) of the robot
 */
void cocobot_asserv_ramp_reset(cocobot_asserv_ramp_t * ramp, float current_position);

/* Set the max speed for the trapeze speed generator
 * Argument:
 *  - ramp: a valid cocobot_asserv_ramp_t pointer
 *  - max_speed: maximum speed allowed
 */
void cocobot_asserv_ramp_set_max_speed(cocobot_asserv_ramp_t * ramp, float max_speed);

/* Set the acceleration for the trapeze speed generator
 * Argument:
 *  - ramp: a valid cocobot_asserv_ramp_t pointer
 *  - max_accel: maximum acceleration allowed
 */
void cocobot_asserv_ramp_set_max_accel(cocobot_asserv_ramp_t * ramp, float max_accel);

/* Get the max speed for the trapeze speed generator
 * Argument:
 *  - ramp: a valid cocobot_asserv_ramp_t pointer
 * Return:
 *  maximum speed allowed
 */
float cocobot_asserv_ramp_get_max_speed(cocobot_asserv_ramp_t * ramp);

/* Get the acceleration for the trapeze speed generator
 * Argument:
 *  - ramp: a valid cocobot_asserv_ramp_t pointer
 * Return:
 *  maximum acceleration allowed
 */
float cocobot_asserv_ramp_get_max_accel(cocobot_asserv_ramp_t * ramp);

/* Run the computation
 * Argument:
 *  - ramp: a valid cocobot_asserv_ramp_t pointer
 */
void cocobot_asserv_ramp_compute(cocobot_asserv_ramp_t * ramp);

/* Get the output of the ramp
 * Argument:
 *  - ramp: a valid cocobot_asserv_ramp_t pointer
 * Return:
 *  Output of the ramp
 */
float cocobot_asserv_ramp_get_output(cocobot_asserv_ramp_t * ramp);

/* Set the ramp target
 * Argument:
 *  - ramp: a valid cocobot_asserv_ramp_t pointer
 *  - target: requested target
 */
void cocobot_asserv_ramp_set_position_target(cocobot_asserv_ramp_t * ramp, float target);

/* Get the ramp position target
 * Argument:
 *  - ramp: a valid cocobot_asserv_ramp_t pointer
 * Return:
 *  actual position target
 */
float cocobot_asserv_ramp_get_position_target(cocobot_asserv_ramp_t * ramp);

/* Get the ramp speed target
 * Argument:
 *  - ramp: a valid cocobot_asserv_ramp_t pointer
 * Return:
 *  actual speed target
 */
float cocobot_asserv_ramp_get_speed_target(cocobot_asserv_ramp_t * ramp);

void cocobot_asserv_ramp_set_feedback(cocobot_asserv_ramp_t * ramp, float feedback);

#endif// COCOBOT_ASSERV_RAMP_H
