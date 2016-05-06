#ifndef COCOBOT_ASSERV_H
#define COCOBOT_ASSERV_H

typedef enum
{
  COCOBOT_ASSERV_DISABLE,
  COCOBOT_ASSERV_ENABLE,
} cocobot_asserv_state_t;

/* Initialization of the asserv module. Need to be called before any other action 
 */
void cocobot_asserv_init(void);

/* Run the asserv computation
 */
void cocobot_asserv_compute(void);

/* Set the curvilinear distance set point
 * Argument:
 *  - distance: requested set point (mm)
 */
void cocobot_asserv_set_distance_set_point(float distance);

/* Set the angular set point
 * Argument:
 *  - distance: requested set point (deg)
 */
void cocobot_asserv_set_angular_set_point(float angular);

/* Enable/disable the asserv
 * Argument:
 *  - state: requested cocobot_asserv_state_t value (COCOBOT_ASSERV_DISABLE or COCOBOT_ASSERV_ENABLE)
 */
void cocobot_asserv_set_state(cocobot_asserv_state_t state);

/* Get the asserv of the asserv
 * Return:
 *  - cocobot_asserv_state_t value (COCOBOT_ASSERV_DISABLE or COCOBOT_ASSERV_ENABLE)
 */
cocobot_asserv_state_t cocobot_asserv_get_state(void);

/* Handle console user command related to asserv module
 * Argument:
 *  - command: requested command
 * Return:
 *  0 : if command is not reconized
 *  1 : if command has been successfully handled
 */
int cocobot_asserv_handle_console(char * command);

/* Send asynchronously debug informations if user has requested them
 */
void cocobot_asserv_handle_async_console(void);

/* Get the linear speed of the robot (assumpe acceleration is inifite)
 * Return:
 *  - the linear speed in m/s
 */
float cocobot_asserv_get_linear_speed(void);

void cocobot_asserv_slow(void);

#endif// COCOBOT_ASSERV_H
