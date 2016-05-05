#ifndef COCOBOT_ACTION_SCHEDULER_H
#define COCOBOT_ACTION_SCHEDULER_H

#define ACTION_NAME_LENGTH 32

/**** User action callback return values ****/
/* For positive return values, action will be marked as done and never tried
 * again.
 * For negative values, action failed and can be tried later.
 */
typedef enum
{
  COCOBOT_RETURN_NO_ACTION_WITH_THIS_NAME       = -6,
  COCOBOT_RETURN_NO_ACTION_TO_EXEC              = -5,

  COCOBOT_RETURN_ACTION_SUCCESS_BUT_DO_IT_AGAIN = -4,
  COCOBOT_RETURN_ACTION_NOT_REACHED             = -3,
  COCOBOT_RETURN_ACTION_ADVERSARY_BLOCK_ACTION  = -2,
  COCOBOT_RETURN_ACTION_UNKNOWN_FAILURE         = -1,

  COCOBOT_RETURN_ACTION_SUCCESS                 = 1,
  COCOBOT_RETURN_ACTION_CORRUPTED               = 2,
  COCOBOT_RETURN_ACTION_SUCCESS_BUT_IM_LOST     = 3,

  // Action can be done later but it's value decreases
  COCOBOT_RETURN_ACTION_PARTIAL_SUCCESS         = 0,
} cocobot_action_callback_result_t;


typedef enum
{
  COCOBOT_STRATEGY_DEFENSIVE,
  COCOBOT_STRATEGY_OFFENSIVE,
} cocobot_strategy_t;

// Should never return 0 (reserved).
// Should return a strictly positive value if execution was done correctly.
// Should return a strictly negative value when action could not be fully
// executed and should be done later.
// Argument is callback_arg
typedef cocobot_action_callback_result_t (*action_callback)(void *);

typedef int (*action_unlocked)(void);

// Function for the user to give dynamics coordinates to actions
// First argument is callback_arg
typedef void (*action_pos)(void *, float *x, float *y, float *a);

/* Init scheduler and game's state. Must be called once before any other function.
 */
void cocobot_action_scheduler_init(void);

/* Set the strategy used by the scheduler when evaluating actions value
 * (default is COCOBOT_STRATEGY_DEFENSIVE)
 * Argument:
 *  - strat: strategy mode
 */
void cocobot_action_scheduler_set_strategy(cocobot_strategy_t strat);

/* Set the average linear speed of the robot during a match. This data is
 * used to approximate the time needed to reach the action's starting point.
 * By default the scheduler will consider that the robot needs no time to
 * reach the action's starting point, which might bring unwanted results
 * (e.g. action not completed before the end of the match).
 */
void cocobot_action_scheduler_set_average_linear_speed(float speed);

/* Set scheduler pause
 * Argument:
 * - paused: pause game if 1, unpause if 0
 */
void cocobot_action_scheduler_set_pause(int paused);

/* Set scheduler goto function
 * Argument:
 * - use_pathfinder: use pathfinder if 1 or use simple goto_xy if 0
 */
void cocobot_action_scheduler_use_pathfinder(int use_pathfinder);

/* Start internal variables computation and starts unfinite loop to handle
 * actions execution.
 */
void cocobot_action_scheduler_start(void);

/* Add a new action to the scheduler
 * Argument:
 *  - name:             action's name
 *  - score:            points given when action succeed
 *  - coordinates:      function returning x, y position (mm) and angle (deg) through its args.
 *  Coordinates to start the action are relative to the table. The angle value
 *  is optional and can be replaced by NAN defined in math.h if it should not
 *  be used.
 *  - execution_time:   time needed to guarantee the action's full execution (in ms)
 *  - success_proba:    probability that the action succeed when doing it (between 0 and 1)
 *  - preexec_callback: function to call when action before the robot moves to
 *  the action (can be set to NULL if not needed)
 *  - exec_callback:    function to call when action should be executed
 *  - cleanup_callback: function always called at the end, even if a problem
 *  occured and action could not be executed (can be set to NULL if not needed)
 *  - callback_arg:     argument to be used with all callback functions
 *  - unlocked:         function called to know if action is unlocked and can be
 *  executed (can be set to NULL if action is always unlocked)
 */
void cocobot_action_scheduler_add_action(char name[ACTION_NAME_LENGTH],
    unsigned int score, action_pos pos, int32_t execution_time, float success_proba,
    action_callback preexec_callback, action_callback exec_callback,
    action_callback cleanup_callback, void *callback_arg, action_unlocked unlocked);

/* Execute the action with the given name
 * Argument:
 *  - name: name of the action to execute
 * Return:
 *  - COCOBOT_RETURN_NO_ACTION_WITH_THIS_NAME if there is no action with this
 *  name 
 *  - else action's callback return value (negative value for error during
 *  action's execution)
 */
cocobot_action_callback_result_t
cocobot_action_scheduler_execute_action_by_name(char name[ACTION_NAME_LENGTH]);

/* Execute best remaining action based on game's state (selected strategy,
 * remaining time, distance, obstacles, ...)
 * Return:
 *  - COCOBOT_RETURN_NO_ACTION_TO_EXEC if no action could be executed with
 *  current game's state
 *  - else action's callback return value (negative value for error during
 *  action's execution)
 */
cocobot_action_callback_result_t cocobot_action_scheduler_execute_best_action(void);

/* Handle console user command related to action scheduler module
 * Argument:
 *  - command: requested command
 * Return:
 *  0 : if command is not reconized
 *  1 : if command has been successfully handled
 */
int cocobot_action_scheduler_handle_console(char * command);

#endif // COCOBOT_ACTION_SCHEDULER_H
