#ifndef COCOBOT_ACTION_SCHEDULER_H
#define COCOBOT_ACTION_SCHEDULER_H

typedef enum
{
  COCOBOT_STRATEGY_DEFENSIVE,
  COCOBOT_STRATEGY_OFFENSIVE,
} cocobot_strategy_t;

/* Init scheduler and game's state
 */
void cocobot_action_scheduler_init(void);

/* Set the strategy used by the scheduler when evaluating actions value
 * (default is COCOBOT_STRATEGY_DEFENSIVE)
 * Argument:
 *  - strat: strategy mode
 */
void cocobot_action_scheduler_set_strategy(cocobot_strategy_t strat);

/* Add a new action to the scheduler
 * Argument:
 *  - score:          points given when action succeed
 *  - x, y, a:        position (mm) and angle (deg), relatively to the table, to start the action
 *  - execution_time: time needed to guarantee the action's full execution
 *  - success_proba:  probability that the action succeed when doing it (between 0 and 1)
 * Return:
 *  A unique ID for the new action
 */
void * cocobot_action_scheduler_add_action(unsigned int score, float x, float y, float a, float execution_time, float risk);

/* Compute action's value based on current game's state (selected strategy,
 * remaining time, distance, obstacles, ...)
 * Argument:
 *  - action_id: id of the action to evaluate (see cocobot_action_scheduler_add_action)
 * Return:
 *  The action's value (bigger is better). If negative, action can't be done
 *  in time.
 */
float cocobot_action_scheduler_eval(void * action_id);


#endif // COCOBOT_ACTION_SCHEDULER_H
