#ifndef COCOBOT_ASSERV_RAMP_H
#define COCOBOT_ASSERV_RAMP_H

//internal structure
typedef struct
{
  float max_speed;
  float max_accel;

  float position_target;
  float position_current;
  float speed_target;

  float output;
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

#endif// COCOBOT_ASSERV_RAMP_H
