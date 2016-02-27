#ifndef COCOBOT_ACTION_SCHEDULER_H
#define COCOBOT_ACTION_SCHEDULER_H

typedef enum
{
  COCOBOT_STRATEGY_DEFENSIVE,
  COCOBOT_STRATEGY_OFFENSIVE,
} cocobot_strategy_t;

// Should never return 0 (reserved).
// Should return a strictly positive value if execution was done correctly.
// Should return a strictly negative value when action could not be fully
// executed and should be done later.
typedef int (*action_callback)(void);

/* Init scheduler and game's state
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

/* Add a new action to the scheduler
 * Argument:
 *  - score:          points given when action succeed
 *  - x, y, a:        position (mm) and angle (deg), relatively to the table, to start the action
 *  - execution_time: time needed to guarantee the action's full execution
 *  - success_proba:  probability that the action succeed when doing it (between 0 and 1)
 *  - callback:       function to call when action should be executed
 */
void cocobot_action_scheduler_add_action(unsigned int score, float x, float y, float a, float execution_time, float risk, action_callback callback);

/* Execute best remaining action based on game's state (selected strategy,
 * remaining time, distance, obstacles, ...)
 * Return:
 *  - 0 if no action could be executed with current game's state
 *  - else action's callback return value (negative value for error during
 *  action's execution)
 */
int cocobot_action_scheduler_execute_best_action(void);

#endif // COCOBOT_ACTION_SCHEDULER_H
