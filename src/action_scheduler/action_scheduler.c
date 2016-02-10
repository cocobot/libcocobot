#include <cocobot.h>

typedef struct
{
  unsigned int score;
  float x, y, a;
  float execution_time;
  float risk;
} cocobot_action_t;

void cocobot_action_scheduler_set_strategy(cocobot_strategy_t strat)
{
  (void)strat;
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
  (void)action_id;

  return 0;
}
