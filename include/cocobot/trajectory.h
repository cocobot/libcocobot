#ifndef cocobot_trajectory_H
#define cocobot_trajectory_H

#define COCOBOT_TRAJECTORY_UNLIMITED_TIME (-1f)

/* Type for storage of order unique handle */
typedef uint32_t cocobot_trajectory_handle_t;

/* Return values of cocobot_trajectory_wait and cocobot_trajectory_wait_until */
typedef enum
{
  COCOBOT_TRAJECTORY_SUCCESS,             /* Orders have been proccessed without error */
  COCOBOT_TRAJECTORY_STOPPED_BEFORE_END,  /* cocobot_trajectory_stop has been called before the end of the order list */
  COCOBOT_TRAJECTORY_TIMEOUT_BEFORE_END,  /* One (or more) order has timouted before reaching the target */
} cocobot_trajectory_result_t;


/* Initialization of the trajectory module. Need to be called before any other action */
void cocobot_trajectory_init(void);


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

/* Goto xy order but in backward. 
 * Arguments:
 *  - x: The target x coordonate in mm
 *  - y: The target y coordonate in mm
 *  - time: Maximum allowed time in s for the order (or COCOBOT_TRAJECTORY_UNLIMITED_TIME for no timeout)
 *  Return:
 *    An unique id for order identification
 */
cocobot_trajectory_handle_t cocobot_trajectory_goto_xy_backward(float x, float y, float time);


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
cocobot_trajectory_result_t cocobot_trajectory_wait(cocobot_trajectory_handle_t order);


/* Stop all orders
 */
void cocobot_trajectory_stop(void);

/* Pause orders (can be resume by calling cocobot_trajectory_resume). Robot will wait without moving
 */
void cocobot_trajectory_pause(void);

/* Resume paused orders
 */
void cocobot_trajectory_resume(void);


#endif// cocobot_trajectory_H
