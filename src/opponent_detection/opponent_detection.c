#include <cocobot.h>

#define COCOBOT_OPPONENT_DETECTION_USIR_FRONT_LEFT 0
#define COCOBOT_OPPONENT_DETECTION_USIR_FRONT_RIGHT 1
#define COCOBOT_OPPONENT_DETECTION_USIR_BACK_LEFT 2
#define COCOBOT_OPPONENT_DETECTION_USIR_BACK_RIGHT 3

typedef struct
{
  int us;
  int ir;
  int alert_activated;
  int alert;
} cocobot_opponent_detection_usir_t;

typedef struct
{
  float x;
  float y;
  int activated;
} cocobot_opponent_detection_fake_robot_t;

static cocobot_opponent_detection_usir_t _usirs[4];
static cocobot_opponent_detection_fake_robot_t _fakebot;

void cocobot_opponent_detection_init(unsigned int task_priority)
{
  //(void)task_priority;
  int i;
  for(i = 0; i < 4; i += 1)
  {
    _usirs[i].us = 42000;
    _usirs[i].ir = 42000;
    _usirs[i].alert_activated = COCOBOT_OPPONENT_DETECTION_DEACTIVATED;
    _usirs[i].alert = 0;
  }

  _fakebot.x = 0;
  _fakebot.y = 0;
  _fakebot.activated = COCOBOT_OPPONENT_DETECTION_DEACTIVATED;
}

int cocobot_opponent_detection_handle_console(char * command)
{
  if(strcmp(command,"opponent_usir") == 0)
  {
    int i;
    for(i = 0; i < 4; i += 1)
    {
      cocobot_console_send_answer("%d %d %d %d", _usirs[i].us, _usirs[i].ir, _usirs[i].alert_activated, _usirs[i].alert);
    }

    return 1;
  }

  if(strcmp(command, "opponent_fakebot") == 0)
  {
    int activate;
    float x;
    float y;

    if(cocobot_console_get_iargument(0, &activate))
    {
      if(cocobot_console_get_fargument(1, &x))
      {
        if(cocobot_console_get_fargument(2, &y))
        {
          _fakebot.x = x;
          _fakebot.y = y;
          _fakebot.activated = (activate > 0) ? COCOBOT_OPPONENT_DETECTION_ACTIVATED : COCOBOT_OPPONENT_DETECTION_DEACTIVATED;
        }
      }
    }

    cocobot_console_send_answer("%d %f %f", _fakebot.activated, _fakebot.x, _fakebot.y);
    return 1;
  }

  return 0;
}
