#include <stdlib.h>
#include <cocobot.h>
#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#ifdef AUSBEE_SIM
# include <time.h> //for random seed
#endif
#include <platform.h>

#define USER_DATA_SIZE 16

static cocobot_game_state_funny_action_t _funny_action;
static TimerHandle_t _end_match_timer;
static cocobot_game_state_color_t _color;
static void * _userdata[USER_DATA_SIZE];
static uint8_t _starter_removed;
static TickType_t _start_time = 0;

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

  _starter_removed = 0;

#ifdef AUSBEE_SIM
  //random color in simu
  srand(time(NULL));
  _color = rand() % 2; 
#else
  if(platform_gpio_get(PLATFORM_GPIO_COLOR))
  {
    _color = COCOBOT_GAME_STATE_COLOR_POS;
  }
  else
  {
    _color = COCOBOT_GAME_STATE_COLOR_NEG;
  }
#endif

  //create 90s timer
  _end_match_timer = xTimerCreate("end_match", 90000 / portTICK_PERIOD_MS, pdFALSE, NULL, cocobot_game_state_match_ended_event);
}

uint8_t cocobot_game_state_is_starter_removed(void)
{
  return _starter_removed;
}

void cocobot_game_state_wait_for_starter_removed(void)
{
#ifdef AUSBEE_SIM
  //no starter on simu. Just wait 3s
  vTaskDelay(3000 / portTICK_PERIOD_MS);
#else
  while(!platform_gpio_get(PLATFORM_GPIO_STARTER))
  {
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
  _starter_removed = 1;
#endif

  _start_time = xTaskGetTickCount();

  xTimerStart(_end_match_timer, 0);

  cocobot_asserv_set_state(COCOBOT_ASSERV_ENABLE);
}

cocobot_game_state_color_t cocobot_game_state_get_color(void)
{
  return _color;
}

int32_t cocobot_game_state_get_elapsed_time(void)
{
  if(_start_time == 0)
  {
    return 0;
  }

  return (xTaskGetTickCount() - _start_time) * portTICK_PERIOD_MS;
}

void cocobot_game_state_set_userdata(unsigned int id, void * data)
{
  _userdata[id] = data;
}

void * cocobot_game_state_get_userdata(unsigned int id)
{
  return _userdata[id];
}
