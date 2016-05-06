#include <cocobot.h>
#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <math.h>

#define SCHEDULER_MAX_ACTIONS 32

#define EUCLIDEAN_DISTANCE(x, y) (sqrt((x)*(x) + (y)*(y)))

typedef enum
{
  COCOBOT_ACTION_REACHED,
  COCOBOT_ACTION_NOT_REACHED,
  COCOBOT_ADVERSARY_DETECTED,
} cocobot_action_goto_return_value_t;

typedef enum
{
  COCOBOT_ACTION_ALREADY_DONE    = -3,
  COCOBOT_ACTION_LOCKED          = -2,
  COCOBOT_ACTION_NOT_ENOUGH_TIME = -1,
} cocobot_action_eval_return_value_t;

typedef struct
{
  float x, y; // in mm
  float a;    // in deg
} cocobot_pos_t;

typedef struct
{
  char            name[ACTION_NAME_LENGTH];
  unsigned int    score;
  action_pos      pos;
  int32_t         execution_time;
  float           success_proba;
  action_callback preexec_callback;
  action_callback exec_callback;
  action_callback cleanup_callback;
  void *          callback_arg;
  action_unlocked unlocked;
  uint8_t         done;
} cocobot_action_t;

static cocobot_action_t action_list[SCHEDULER_MAX_ACTIONS];
static unsigned int action_list_end;

#define INITIAL_REMAINING_TIME 90000

static struct cocobot_game_state_t
{
  cocobot_strategy_t  strat;
  cocobot_pos_t       robot_pos;
  float               robot_average_linear_speed; // in m/s (or mm/ms)
  int32_t             remaining_time; // in ms
  int                 paused;
  int                 use_pathfinder;
  int                 time_gets_short;
} current_game_state;


void cocobot_action_scheduler_init(void)
{
  current_game_state.strat = COCOBOT_STRATEGY_DEFENSIVE;
  current_game_state.robot_pos.x = 0;
  current_game_state.robot_pos.y = 0;
  current_game_state.robot_pos.a = 0;
  current_game_state.robot_average_linear_speed = 0;
  current_game_state.remaining_time = INITIAL_REMAINING_TIME;
  current_game_state.paused = 0;
  current_game_state.use_pathfinder = 0;
  current_game_state.time_gets_short = 0;

  action_list_end = 0;
}

void cocobot_action_scheduler_set_strategy(cocobot_strategy_t strat)
{
  current_game_state.strat = strat;
}

void cocobot_action_scheduler_set_average_linear_speed(float speed)
{
  current_game_state.robot_average_linear_speed = speed;
}

void cocobot_action_scheduler_set_pause(int paused)
{
  current_game_state.paused = paused;
}

void cocobot_action_scheduler_use_pathfinder(int use_pathfinder)
{
  current_game_state.use_pathfinder = use_pathfinder;
}

void cocobot_action_scheduler_start(void)
{
  while(1)
  {
    if (current_game_state.paused)
    {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    else if(!cocobot_action_scheduler_execute_best_action())
    {
      //wait small delay if no action is available (which is a bad thing)
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    else
    {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

static void cocobot_action_scheduler_update_game_state(void)
{
  current_game_state.remaining_time = INITIAL_REMAINING_TIME - cocobot_game_state_get_elapsed_time();

  current_game_state.robot_pos.x = cocobot_position_get_x();
  current_game_state.robot_pos.y = cocobot_position_get_y();
  current_game_state.robot_pos.a = cocobot_position_get_angle();
}

void cocobot_action_scheduler_add_action(char name[ACTION_NAME_LENGTH],
    unsigned int score, action_pos pos, int32_t execution_time, float success_proba,
    action_callback preexec_callback, action_callback exec_callback,
    action_callback cleanup_callback, void * callback_arg, action_unlocked unlocked)
{
  cocobot_action_t * new_action = &action_list[action_list_end];

  strncpy(new_action->name, name, ACTION_NAME_LENGTH);
  new_action->score = score;
  new_action->pos = pos;
  new_action->execution_time = execution_time;
  new_action->success_proba = success_proba;
  new_action->preexec_callback = preexec_callback;
  new_action->exec_callback = exec_callback;
  new_action->cleanup_callback = cleanup_callback;
  new_action->callback_arg = callback_arg;
  new_action->unlocked = unlocked;
  new_action->done = 0;

  if (action_list_end < SCHEDULER_MAX_ACTIONS-1)
  {
    action_list_end++;
  }
  else
  {
    printf("[Warning] cocobot_action_scheduler: Action list full.\n");
  }
}

static void cocobot_action_scheduler_print_action(cocobot_action_t * action)
{
  cocobot_console_send_answer("\t score: %d", action->score);
  float x, y, a;
  (*action->pos)(action->callback_arg, &x, &y, &a);
  cocobot_console_send_answer("\t pos: (x = %.1f mm, y = %.1f, a = %.2f deg)",
      (double)x, (double)y, (double)a);
  cocobot_console_send_answer("\t execution time: %ld", action->execution_time);
  cocobot_console_send_answer("\t success proba: %.2f", (double)action->success_proba);
  cocobot_console_send_answer("\t done: %d", action->done);
}

/* Returns the approximate time (in ms) needed by the robot to reach the action's
 * starting point, based on the average robot linear speed set by the user.
 * By default, this time is set to 0 ms.
 * TODO: this function should be replaced by a more precise time given by the
 * pathfinder.
 */
static float cocobot_action_scheduler_time_to_reach(cocobot_action_t * action)
{
  if (current_game_state.robot_average_linear_speed == 0)
  {
    return 0;
  }

  float action_x, action_y, action_a;
  (*action->pos)(action->callback_arg, &action_x, &action_y, &action_a);
  float x = current_game_state.robot_pos.x - action_x;
  float y = current_game_state.robot_pos.y - action_y;
  // 2 is a factor to approximate the length of the real path from the
  // straight-line distance between the two points
  float approximate_linear_distance = 2 * EUCLIDEAN_DISTANCE(x, y);

  return (approximate_linear_distance)/(current_game_state.robot_average_linear_speed);
}

/* Compute action's value based on current game's state
 * Argument:
 *  - action_id: id of the action to evaluate (see cocobot_action_scheduler_add_action)
 * Return:
 *  The action's value (bigger is better). If negative, action can't be done
 *  in time or as already been done.
 */
static float cocobot_action_scheduler_eval(cocobot_action_t * action)
{
  cocobot_action_scheduler_update_game_state();

  if (action->done)
  {
    return COCOBOT_ACTION_ALREADY_DONE;
  }
  if (action->unlocked != NULL && !action->unlocked())
  {
    return COCOBOT_ACTION_LOCKED;
  }

  float time_value = 1000 / (action->execution_time + cocobot_action_scheduler_time_to_reach(action));

  if (current_game_state.time_gets_short)
  {
    return time_value;
  }
  else
  {
    if (action->execution_time > current_game_state.remaining_time)
    {
      return COCOBOT_ACTION_NOT_ENOUGH_TIME;
    }

    // TODO: Take strategy and remaining time to execute other actions into account
    float potential_elementary_value = action->score * time_value;
    float effective_elementary_value = action->success_proba * potential_elementary_value;

    return effective_elementary_value;
  }
}

// TODO: should be replaced by a pathfinder function, with more return code
static cocobot_action_goto_return_value_t cocobot_action_scheduler_goto(cocobot_action_t * action)
{
  float x, y, a;
  (*action->pos)(action->callback_arg, &x, &y, &a);

  if (current_game_state.use_pathfinder)
  {
    cocobot_pathfinder_execute_trajectory(cocobot_position_get_x(), cocobot_position_get_y(), x, y);
  }
  else
  {
    cocobot_trajectory_goto_xy(x , y, -1);
  }

  if (!isnan(a)) {
    cocobot_trajectory_goto_a(a, -1);
  }
  cocobot_trajectory_wait();

  return COCOBOT_ACTION_REACHED;
}

static void cocobot_action_scheduler_handle_action_result(cocobot_action_t *action,
    cocobot_action_callback_result_t action_result)
{
  // If action correctly executed, mark action as done
  if (action_result > 0)
  {
    action->done = 1;
  }
  else if (action_result == COCOBOT_RETURN_ACTION_PARTIAL_SUCCESS)
  {
    action->score = action->score / 2;
  }

  if (action_result == COCOBOT_RETURN_ACTION_SUCCESS_BUT_IM_LOST)
  {
    // TODO: do something smart when robot is lost
  }
}

static cocobot_action_callback_result_t cocobot_action_scheduler_execute_action(cocobot_action_t *action)
{
  if (action->preexec_callback != NULL)
  {
    (*action->preexec_callback)(action->callback_arg);
  }

  cocobot_action_callback_result_t action_return_value = COCOBOT_RETURN_ACTION_NOT_REACHED;
  cocobot_action_goto_return_value_t goto_return_value = cocobot_action_scheduler_goto(action);

  if (goto_return_value == COCOBOT_ACTION_REACHED)
  {
    action_return_value = (*action->exec_callback)(action->callback_arg);
  }

  cocobot_action_scheduler_handle_action_result(action, action_return_value);

  if (action->cleanup_callback != NULL)
  {
    (*action->cleanup_callback)(action->callback_arg);
  }

  return action_return_value;
}

cocobot_action_callback_result_t
cocobot_action_scheduler_execute_action_by_name(char name[ACTION_NAME_LENGTH])
{
  unsigned int action_current_index = 0;
  cocobot_action_callback_result_t action_result = COCOBOT_RETURN_NO_ACTION_WITH_THIS_NAME;

  for (; action_current_index < action_list_end; action_current_index++)
  {
    if (strncmp(name, action_list[action_current_index].name, ACTION_NAME_LENGTH) == 0)
    {
      action_result = cocobot_action_scheduler_execute_action(&action_list[action_current_index]);
      break;
    }
  }

  return action_result;
}

cocobot_action_callback_result_t cocobot_action_scheduler_execute_best_action(void)
{
  unsigned int action_current_index = 0;
  int action_best_index = -1;
  float action_current_eval = 0;
  float action_best_eval = -3;

  // Compute best action
  for (; action_current_index < action_list_end; action_current_index++)
  {
    action_current_eval = cocobot_action_scheduler_eval(&action_list[action_current_index]);
    if (action_current_eval > action_best_eval)
    {
      action_best_eval = action_current_eval;
      action_best_index = action_current_index;
    }
  }

  if (action_best_eval == COCOBOT_ACTION_NOT_ENOUGH_TIME)
  {
    // Only remaining action take too much time, we still try the quickest one
    current_game_state.time_gets_short = 1;
    return cocobot_action_scheduler_execute_best_action();
  }
  else if (action_best_eval <= 0)
  {
    return COCOBOT_RETURN_NO_ACTION_TO_EXEC;
  }

  cocobot_action_t * best_action = &action_list[action_best_index];

  return cocobot_action_scheduler_execute_action(best_action);
}


static void cocobot_action_scheduler_print_action_by_name(char name[ACTION_NAME_LENGTH])
{
  unsigned int action_current_index = 0;

  for (; action_current_index < action_list_end; action_current_index++)
  {
    if (strncmp(name, action_list[action_current_index].name, ACTION_NAME_LENGTH) == 0)
    {
      cocobot_action_scheduler_print_action(&action_list[action_current_index]);
      return;
    }
  }

  cocobot_console_send_answer("No action with name: %s", name);
}

static void cocobot_action_scheduler_list_actions(void)
{
  unsigned int i = 0;
  float action_value;

  if (action_list_end == 0)
  {
      cocobot_console_send_answer("No action in the list");
      return;
  }

  for (; i < action_list_end; i++)
  {
    action_value = cocobot_action_scheduler_eval(&action_list[i]);

    if (action_value == COCOBOT_ACTION_ALREADY_DONE)
    {
      cocobot_console_send_answer("%s: done!", action_list[i].name);
    }
    else if (action_value == COCOBOT_ACTION_LOCKED)
    {
      cocobot_console_send_answer("%s: locked!", action_list[i].name);
    }
    else if (action_value == COCOBOT_ACTION_NOT_ENOUGH_TIME)
    {
      cocobot_console_send_answer("%s: not enough time", action_list[i].name);
    }
    else
    {
      cocobot_console_send_answer("%s: %.3f", action_list[i].name, (double)action_value);
    }
  }
}

static void cocobot_action_scheduler_debug_actions(void)
{
  unsigned int i = 0;
  float action_value;
  float x, y, a;

  for (; i < action_list_end; i++)
  {
    cocobot_action_t * action = &action_list[i];

    action_value = cocobot_action_scheduler_eval(action);

    (*action->pos)(action->callback_arg, &x, &y, &a);

    cocobot_console_send_answer("%s,%.0f,%.0f,%0.3f", action->name, (double)x, (double)y, (double)action_value);
  }
}

int cocobot_action_scheduler_handle_console(char * command)
{
  if(strcmp(command,"exec_action") == 0)
  {
    char action_name[ACTION_NAME_LENGTH];
    cocobot_action_callback_result_t res;

    if(cocobot_console_get_sargument(0, action_name, sizeof(action_name)))
    {
      // TODO: Warning cannot be used inside this task
      res = cocobot_action_scheduler_execute_action_by_name(action_name);
      switch(res)
      {
      case COCOBOT_RETURN_NO_ACTION_WITH_THIS_NAME:
        cocobot_console_send_answer("No action with name: %s", action_name);
        break;

      case COCOBOT_RETURN_ACTION_NOT_REACHED:
        cocobot_console_send_answer("Action could not be reached");
        break;

      case COCOBOT_RETURN_ACTION_SUCCESS:
        cocobot_console_send_answer("Action succeed");
        break;

      case COCOBOT_RETURN_ACTION_UNKNOWN_FAILURE:
        cocobot_console_send_answer("Action failed");
        break;

      default:
        cocobot_console_send_answer("Something happened");
      }
    }
    return 1;
  }
  if(strcmp(command,"list_actions") == 0)
  {
    cocobot_action_scheduler_list_actions();
    return 1;
  }
  if(strcmp(command,"print_action") == 0)
  {
    char action_name[ACTION_NAME_LENGTH];
    if(cocobot_console_get_sargument(0, action_name, sizeof(action_name)))
    {
      cocobot_action_scheduler_print_action_by_name(action_name);
    }
    return 1;
  }
  if(strcmp(command,"action_pause") == 0)
  {
    int paused;
    if(cocobot_console_get_iargument(0, &paused))
    {
      cocobot_action_scheduler_set_pause(paused);
    }
    cocobot_console_send_answer("%d", current_game_state.paused);
    return 1;
  }
  if(strcmp(command,"debug_actions") == 0)
  {
    cocobot_action_scheduler_debug_actions();
    return 1;
  }

  return 0;
}
