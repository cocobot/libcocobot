#ifndef COCOBOT_TRAJECTORY_H
#define COCOBOT_TRAJECTORY_H

#define COCOBOT_TRAJECTORY_UNLIMITED_TIME (-1.0f)
#define COCOBOT_TRAJECTORY_INVALID_HANDLE (-1)

/* Type for storage of order unique handle */
typedef int32_t cocobot_trajectory_handle_t;

/* Return values of cocobot_trajectory_wait and cocobot_trajectory_wait_until */
typedef enum
{
  COCOBOT_TRAJECTORY_SUCCESS,             /* Orders have been proccessed without error */
  COCOBOT_TRAJECTORY_STOPPED_BEFORE_END,  /* cocobot_trajectory_stop has been called before the end of the order list */
  COCOBOT_TRAJECTORY_TIMEOUT_BEFORE_END,  /* One (or more) order has timouted before reaching the target */
  COCOBOT_TRAJECTORY_RUNNING,
} cocobot_trajectory_result_t;

typedef enum
{
  COCOBOT_TRAJECTORY_FORWARD,
  COCOBOT_TRAJECTORY_BACKWARD
} cocobot_trajectory_xy_default_t;


/* Initialization of the trajectory module. Need to be called before any other action 
 * Argument:
 *  - task_priority: FreeRTOS task priority for trajectory thread
 */
void cocobot_trajectory_init(unsigned int task_priority);


/* Goto d order. 
 * Arguments:
 *  - distance: The target distance in mm
 *  - time: Maximum allowed time in s for the order (or COCOBOT_TRAJECTORY_UNLIMITED_TIME for no timeout)
 *  Return:
 *    An unique id for order identification
 */
cocobot_trajectory_handle_t cocobot_trajectory_goto_d(float distance, float time);

/* Goto a order. 
 * Arguments:
 *  - angle: The target angle in deg
 *  - time: Maximum allowed time in s for the order (or COCOBOT_TRAJECTORY_UNLIMITED_TIME for no timeout)
 *  Return:
 *    An unique id for order identification
 */
cocobot_trajectory_handle_t cocobot_trajectory_goto_a(float angle, float time);

/* Goto xy order. 
 * Arguments:
 *  - x: The target x coordonate in mm
 *  - y: The target y coordonate in mm
 *  - time: Maximum allowed time in s for the order (or COCOBOT_TRAJECTORY_UNLIMITED_TIME for no timeout)
 *  Return:
 *    An unique id for order identification
 */
cocobot_trajectory_handle_t cocobot_trajectory_goto_xy(float x, float y, float time);

cocobot_trajectory_handle_t cocobot_trajectory_goto_xy_forward(float x, float y, float time);

/* Goto xy order but in backward. 
 * Arguments:
 *  - x: The target x coordonate in mm
 *  - y: The target y coordonate in mm
 *  - time: Maximum allowed time in s for the order (or COCOBOT_TRAJECTORY_UNLIMITED_TIME for no timeout)
 *  Return:
 *    An unique id for order identification
 */
cocobot_trajectory_handle_t cocobot_trajectory_goto_xy_backward(float x, float y, float time);

/* Goto xy circle order
 * Arguments:
 *  - xi: x coord of the center point
 *  - yi: y coord of the center point
 *  - xe: x coord of the ending point
 *  - ye: y coord of the ending point
 *  - time: Maximum allowed time in s for the order (or COCOBOT_TRAJECTORY_UNLIMITED_TIME for no timeout)
 * Return:
 *    An unique id for order identification
 *
 * Info:
 *                _intermediate
 *               /             \
 *              /               ending point
 *              |
 *              \
 *               current pos
 */
cocobot_trajectory_handle_t cocobot_trajectory_goto_xy_circle(float xc, float yc, float xe, float ye, float time);

/* Goto xy circle order but in backward
 * Arguments:
 *  - xi: x coord of the intermediate point
 *  - yi: y coord of the intermediate point
 *  - xe: x coord of the ending point
 *  - ye: y coord of the ending point
 *  - time: Maximum allowed time in s for the order (or COCOBOT_TRAJECTORY_UNLIMITED_TIME for no timeout)
 * Return:
 *    An unique id for order identification
 *
 * Info:
 *                _intermediate
 *               /             \
 *              /               ending point
 *              |
 *              \
 *               current pos
 */
cocobot_trajectory_handle_t cocobot_trajectory_goto_xy_circle_backward(float xi, float yi, float xe, float ye, float time);


/* Block the task until all the orders has been processed or cocobot_trajectory_stop has been called 
 * Return:
 *  cocobot_trajectory_result_t value
 */
cocobot_trajectory_result_t cocobot_trajectory_wait(void);

/* Block the task until the order in argument has been processed or cocobot_trajectory_stop has been called 
 * Argument:
 *  - order: an handle to the order
 * Return:
 *  cocobot_trajectory_result_t value
 */
cocobot_trajectory_result_t cocobot_trajectory_wait_until(cocobot_trajectory_handle_t order);


/* Stop all orders
 */
void cocobot_trajectory_stop(void);

/* Pause orders (can be resume by calling cocobot_trajectory_resume). Robot will wait without moving
 */
void cocobot_trajectory_pause(void);

/* Resume paused orders
 */
void cocobot_trajectory_resume(void);


/* Handle console user command related to trajectory module
 * Argument:
 *  - command: requested command
 * Return:
 *  0 : if command is not reconized
 *  1 : if command has been successfully handled
 */
int cocobot_trajectory_handle_console(char * command);

void cocobot_trajetory_set_xy_default(cocobot_trajectory_xy_default_t pref);

void cocobot_trajectory_set_opponent_detection(int enable);

#endif// COCOBOT_TRAJECTORY_H
