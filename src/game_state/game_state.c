#include <stdlib.h>
#include <cocobot.h>
#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#ifdef AUSBEE_SIM
# include <time.h> //for random seed
#endif

#define USER_DATA_SIZE 16

static cocobot_game_state_funny_action_t _funny_action;
static TimerHandle_t _end_match_timer;
static cocobot_game_state_color_t _color;
static void * _userdata[USER_DATA_SIZE];

static void cocobot_game_state_match_ended_event(TimerHandle_t xTimer)
{
  (void)xTimer;

  cocobot_asserv_set_state(COCOBOT_ASSERV_DISABLE);

  if(_funny_action != NULL)
  {
    _funny_action();
  }
}

void cocobot_game_state_init(cocobot_game_state_funny_action_t funny_action)
{
  _funny_action = funny_action;

#ifdef AUSBEE_SIM
  //random color in simu
  srand(time(NULL));
  _color = rand() % 2; 
#else
  //TODO: impl me
  _color = COCOBOT_GAME_STATE_COLOR_NEG;
#endif

  //create 90s timer
  _end_match_timer = xTimerCreate("end_match", 90000 / portTICK_PERIOD_MS, pdFALSE, NULL, cocobot_game_state_match_ended_event);
}

void cocobot_game_state_wait_for_starter_removed(void)
{
#ifdef AUSBEE_SIM
  //no starter on simu. Just wait 3s
  vTaskDelay(3000 / portTICK_PERIOD_MS);
#else
  //TODO: impl me
#endif

  xTimerStart(_end_match_timer, 0);

  cocobot_asserv_set_state(COCOBOT_ASSERV_ENABLE);
}

cocobot_game_state_color_t cocobot_game_state_get_color(void)
{
  return _color;
}


void cocobot_game_state_set_userdata(unsigned int id, void * data)
{
  _userdata[id] = data;
}

void * cocobot_game_state_get_userdata(unsigned int id)
{
  return _userdata[id];
}
