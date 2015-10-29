#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#define TRAJECTORY_UNLIMITED_TIME (-1f)

/* Type for storage of order unique handle */
typedef uint32_t trajectory_handle_t;

/* Return values of trajectory_wait and trajectory_wait_until */
typedef enum
{
  TRAJECTORY_SUCCESS,             /* Orders have been proccessed without error */
  TRAJECTORY_STOPPED_BEFORE_END,  /* trajectory_stop has been called before the end of the order list */
  TRAJECTORY_TIMEOUT_BEFORE_END,  /* One (or more) order has timouted before reaching the target */
} trajectory_result_t;


/* Initialization of the trajectory module. Need to be called before any other action */
void trajectory_init(void);


/* Goto d order. 
 * Arguments:
 *  - distance: The target distance in mm
 *  - time: Maximum allowed time for the order (or TRAJECTORY_UNLIMITED_TIME for no timeout)
 *  Return:
 *    An unique id for order identification
 */
trajectory_handle_t trajectory_goto_d(float distance, float time);

/* Goto a order. 
 * Arguments:
 *  - angle: The target angle in deg
 *  - time: Maximum allowed time for the order (or TRAJECTORY_UNLIMITED_TIME for no timeout)
 *  Return:
 *    An unique id for order identification
 */
trajectory_handle_t trajectory_goto_a(float angle, float time);

/* Goto xy order. 
 * Arguments:
 *  - x: The target x coordonate in mm
 *  - y: The target y coordonate in mm
 *  - time: Maximum allowed time for the order (or TRAJECTORY_UNLIMITED_TIME for no timeout)
 *  Return:
 *    An unique id for order identification
 */
trajectory_handle_t trajectory_goto_xy(float x, float y, float time);

/* Goto xy order but in backward. 
 * Arguments:
 *  - x: The target x coordonate in mm
 *  - y: The target y coordonate in mm
 *  - time: Maximum allowed time for the order (or TRAJECTORY_UNLIMITED_TIME for no timeout)
 *  Return:
 *    An unique id for order identification
 */
trajectory_handle_t trajectory_goto_xy_backward(float x, float y, float time);


/* Block the task until all the orders has been processed or trajectory_stop has been called 
 * Return:
 *  trajectory_result_t value
 */
trajectory_result_t trajectory_wait(void);

/* Block the task until the order in argument has been processed or trajectory_stop has been called 
 * Argument:
 *  - order: an handle to the order
 * Return:
 *  trajectory_result_t value
 */
trajectory_result_t trajectory_wait(trajectory_handle_t order);


/* Stop all orders
 */
void trajectory_stop(void):

#endif// TRAJECTORY_H
