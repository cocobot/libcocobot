#include <cocobot.h>

typedef struct
{
  float x, y; // in mm
  float a;    // in deg
} cocobot_pos_t;

typedef struct
{
  unsigned int  score;
  cocobot_pos_t pos;
  float         execution_time;
  float         risk;
} cocobot_action_t;

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
}

void cocobot_action_scheduler_set_strategy(cocobot_strategy_t strat)
{
  current_game_state.strat = strat;
}

void * cocobot_action_scheduler_add_action(unsigned int score, float x, float y, float a, float execution_time, float risk)
{
  (void)score;
  (void)x;
  (void)y;
  (void)a;
  (void)execution_time;
  (void)risk;

  return 0;
}

float cocobot_action_scheduler_eval(void * action_id)
{
  cocobot_action_t *action = (cocobot_action_t *)action_id;

  if (action->execution_time > current_game_state.remaining_time)
  {
    return -1;
  }

  return 0;
}
