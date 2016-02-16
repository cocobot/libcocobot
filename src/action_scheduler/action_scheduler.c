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
  unsigned int  score;
  cocobot_pos_t pos;
  float         execution_time;
  float         risk;
  uint8_t       done;
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

void * cocobot_action_scheduler_add_action(unsigned int score, float x, float y, float a, float execution_time, float risk)
{
  action_list[action_list_end].score = score;
  action_list[action_list_end].pos.x = x;
  action_list[action_list_end].pos.y = y;
  action_list[action_list_end].pos.a = a;
  action_list[action_list_end].execution_time = execution_time;
  action_list[action_list_end].risk = risk;
  action_list[action_list_end].done = 0;

  if (action_list_end < SCHEDULER_MAX_ACTIONS-1)
  {
    action_list_end++;
  }
  else
  {
    printf("[Warning] cocobot_action_scheduler: Action list full.\n");
  }

  return &action_list[action_list_end];
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
