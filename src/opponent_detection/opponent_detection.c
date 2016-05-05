#include <FreeRTOS.h>
#include <task.h>
#include <cocobot.h>
#include <platform.h>
#include <math.h>

#define COCOBOT_OPPONENT_MIN_CORRELATION  150

typedef struct
{
  int us;
  int ir;
  int alert_activated;
  int alert;
  int force_on;
} cocobot_opponent_detection_usir_t;

typedef struct
{
  float x;
  float y;
  int activated;
} cocobot_opponent_detection_fake_robot_t;

static cocobot_opponent_detection_usir_t _usirs[4];
static cocobot_opponent_detection_fake_robot_t _fakebot;

const int ir_lookup_table[] = {2894, 2480, 1680, 1310, 1060, 890, 790, 700, 650, 590, 500, 450, 400};

static int alert_threshold;

int cocobot_opponent_detection_ir_mV_to_mm(int32_t value)
{
  unsigned int i;
  for(i = 0; i < sizeof(ir_lookup_table)/sizeof(int); i += 1)
  {
    if(ir_lookup_table[i] < value)
    {
      if(i == 0)
      {
        return 1;
      }
      else
      {
        float f1 = ir_lookup_table[i -1];
        float f2 = ir_lookup_table[i];

        float res = (value - f1) / (f2 - f1) * 50 + (i) * 50;

        return res;
      }
    }
  }
  return 10000;
}

void cocobot_opponent_detection_task(void * arg)
{
  //arg is always NULL. Prevent "variable unused" warning
  (void)arg;

  while(1)
  {
    int force_wait = 1;
    int clear = 1;
    int i;
    for(i = 3; i >= 0; i -= 1)
    {
      if(_usirs[i].alert_activated || _usirs[i].force_on)
      {
        platform_us_send_trig(i);
        vTaskDelay(1 / portTICK_PERIOD_MS);
        platform_us_reset_trig(i);
        vTaskDelay(75 / portTICK_PERIOD_MS);
        float adc_mV = platform_adc_get_mV(PLATFORM_ADC_IR0 + i);
        adc_mV = platform_adc_get_mV(PLATFORM_ADC_IR0 + i);
        _usirs[i].ir = cocobot_opponent_detection_ir_mV_to_mm(adc_mV);
        _usirs[i].us = platform_us_get_value(i);

        if((fabs(_usirs[i].us - _usirs[i].ir) < COCOBOT_OPPONENT_MIN_CORRELATION) && (_usirs[i].us < alert_threshold))
        {
#ifndef AUSBEE_SIM
          float x = cocobot_position_get_x();
          float y = cocobot_position_get_y();
          float a = cocobot_position_get_angle();

          float dtec = _usirs[i].us + COCOBOT_ROBOT_DEPTH;

          //back
          if((i == COCOBOT_OPPONENT_DETECTION_USIR_BACK_LEFT) || (i == COCOBOT_OPPONENT_DETECTION_USIR_BACK_RIGHT))
          {
            dtec = -dtec;
          }
          float nx = x + dtec * cosf(a * M_PI / 180.0);
          float ny = y + dtec * sinf(a * M_PI / 180.0);

          if((nx < COCOBOT_OPPONENT_DETECTION_MAX_X) && 
             (nx > -COCOBOT_OPPONENT_DETECTION_MAX_X) &&
             (ny < COCOBOT_OPPONENT_DETECTION_MAX_Y) &&
             (ny > -COCOBOT_OPPONENT_DETECTION_MAX_Y))
          {
            _usirs[i].alert = 1;
          }
          else
          {
            _usirs[i].alert = 2;
          }
#endif
          clear = 0;
 //         platform_gpio_set(PLATFORM_GPIO0);
        }
        else
        {
          _usirs[i].alert = 0;
        }

        force_wait = 0;
      }
      else
      {
        _usirs[i].ir = 0;
        _usirs[i].us = 0;
        _usirs[i].alert = 0;
      }
    }

    if(clear)
    {
 //     platform_gpio_clear(PLATFORM_GPIO0);
    }
    if(force_wait)
    {
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    
  }
}
int cocobot_opponent_detection_is_in_alert(void)
{
  int i;

  for(i = 0; i < 4; i += 1)
  {
    if(_usirs[i].alert == 1)
    {
      return 1;
    }
  }

  return 0;
}

void cocobot_opponent_detection_set_enable(int id, int status)
{
  _usirs[id].alert_activated = status;
}

void cocobot_opponent_detection_init(unsigned int task_priority)
{
  (void)task_priority;

  int i;
  for(i = 0; i < 4; i += 1)
  {
    _usirs[i].us = 0;
    _usirs[i].ir = 0;
    _usirs[i].alert_activated = COCOBOT_OPPONENT_DETECTION_DEACTIVATED;
    _usirs[i].alert = 0;
    _usirs[i].force_on = 0;
  }

  alert_threshold = 250;
  _fakebot.x = 0;
  _fakebot.y = 0;
  _fakebot.activated = COCOBOT_OPPONENT_DETECTION_DEACTIVATED;

  //start task
  xTaskCreate(cocobot_opponent_detection_task, "opponent", 200, NULL, task_priority, NULL);
}

int cocobot_opponent_detection_handle_console(char * command)
{
  if(strcmp(command,"opponent_usir_force") == 0)
  {
    int id;
    if(cocobot_console_get_iargument(0, &id))
    {
      if((id >= 0) && (id < 4))
      {
        int set;
        if(cocobot_console_get_iargument(1, &set))
        {
          _usirs[id].force_on = set;
        }

        cocobot_console_send_answer("%d", _usirs[id].force_on);
        return 1;
      }              
    }
    cocobot_console_send_answer("?");
    return 1;
  }

  if(strcmp(command,"opponent_usir") == 0)
  {
    int i;
    for(i = 0; i < 4; i += 1)
    {
      cocobot_console_send_answer("%d %d %d %d %d", _usirs[i].us, _usirs[i].ir, _usirs[i].alert_activated, _usirs[i].alert, _usirs[i].force_on);
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

    cocobot_console_send_answer("%d %f %f", _fakebot.activated, (double)_fakebot.x, (double)_fakebot.y);
    return 1;
  }

  return 0;
}
