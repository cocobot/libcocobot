#include <cocobot.h>
#include <stdio.h>

#define SCHEDULER_MAX_ACTIONS 16

typedef struct
{
  float x, y; // in mm
  float a;    // in deg
} cocobot_pos_t;

typedef struct
{
  unsigned int    score;
  cocobot_pos_t   pos;
  float           execution_time;
  float           risk;
  action_callback callback;
  uint8_t         done;
} cocobot_action_t;

static cocobot_action_t action_list[SCHEDULER_MAX_ACTIONS];
static unsigned int action_list_end;

static struct cocobot_game_state_t
{
  cocobot_strategy_t  strat;
  cocobot_pos_t       robot_pos;
  float               remaining_time; // in ms
} current_game_state;


void cocobot_action_scheduler_init(void)
{
  current_game_state.strat = COCOBOT_STRATEGY_DEFENSIVE;
  current_game_state.robot_pos.x = 0;
  current_game_state.robot_pos.y = 0;
  current_game_state.robot_pos.a = 0;
  current_game_state.remaining_time = 90000;

  action_list_end = 0;
}

void cocobot_action_scheduler_set_strategy(cocobot_strategy_t strat)
{
  current_game_state.strat = strat;
}

static void cocobot_action_scheduler_update_game_state(void)
{
  // TODO: Update remaining time
  //current_game_state.remaining_time = ;

  current_game_state.robot_pos.x = cocobot_position_get_x();
  current_game_state.robot_pos.y = cocobot_position_get_y();
  current_game_state.robot_pos.a = cocobot_position_get_angle();
}

void cocobot_action_scheduler_add_action(unsigned int score, float x, float y, float a, float execution_time, float risk, action_callback callback)
{
  action_list[action_list_end].score = score;
  action_list[action_list_end].pos.x = x;
  action_list[action_list_end].pos.y = y;
  action_list[action_list_end].pos.a = a;
  action_list[action_list_end].execution_time = execution_time;
  action_list[action_list_end].risk = risk;
  action_list[action_list_end].callback = callback;
  action_list[action_list_end].done = 0;

  if (action_list_end < SCHEDULER_MAX_ACTIONS-1)
  {
    action_list_end++;
  }
  else
  {
    printf("[Warning] cocobot_action_scheduler: Action list full.\n");
  }
}

/* Compute action's value based on current game's state
 * Argument:
 *  - action_id: id of the action to evaluate (see cocobot_action_scheduler_add_action)
 * Return:
 *  The action's value (bigger is better). If negative, action can't be done
 *  in time or as already been done.
 */
static float cocobot_action_scheduler_eval(void * action_id)
{
  cocobot_action_t *action = (cocobot_action_t *)action_id;

  cocobot_action_scheduler_update_game_state();

  if (action->execution_time > current_game_state.remaining_time)
  {
    return -1;
  }
  if (action->done)
  {
    return -0.5f;
  }

  // TODO: Set evaluation formula based on strategy, time, distance, ...

  return 0;
}

int cocobot_action_scheduler_execute_best_action(void)
{
  unsigned int action_current_index = 0;
  int action_best_index = -1;
  float action_current_eval = 0;
  float action_best_eval = -1;

  for (; action_current_index < action_list_end; action_current_index++)
  {
    action_current_eval = cocobot_action_scheduler_eval(&action_list[action_current_index]);
    if (action_current_eval > action_best_eval)
    {
      action_best_eval = action_current_eval;
      action_best_index = action_current_index;
    }
  }

  if (action_best_index < 0 || action_best_eval < 0)
  {
    return 0;
  }

  int action_return_value = (*action_list[action_best_index].callback)();

  if (action_return_value > 0)
  {
    action_list[action_best_index].done = 1;
  }

  return action_return_value;
}
