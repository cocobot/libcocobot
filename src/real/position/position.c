#include <cocobot.h>
#include <math.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "generated/autoconf.h"

#ifndef AUSBEE_SIM
#include <cocobot/encoders.h>
#endif //AUSBEE_SIM

//useful macros
#define TICK2RAD(tick)  ((((float)tick) * M_PI) / ((float)CONFIG_LIBCOCOBOT_POSITION_TICK_PER_180DEG))
#define TICK2DEG(tick)  ((((float)tick) * 180.0) / ((float)CONFIG_LIBCOCOBOT_POSITION_TICK_PER_180DEG))
#define TICK2MM(tick)  ((((float)tick) * 1000.0) / ((float)CONFIG_LIBCOCOBOT_POSITION_TICK_PER_METER))

//mutex for internal value access
static SemaphoreHandle_t mutex;

//internal values (in tick)
static float _enc_x;
static float _enc_y;
static int32_t _enc_angle;
static int32_t _enc_speed_angle;
static int32_t _enc_angle_offset;
static int32_t _enc_distance;
static int32_t _enc_speed_distance;

static void cocobot_position_task(void * arg)
{
  //arg is always NULL. Prevent "variable unused" warning
  (void)arg;
  int32_t motor_position[2] = {0, 0}; // {left, right}

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
    //update encoder values
#ifndef AUSBEE_SIM
    cocobot_encoders_get_motor_position(motor_position);
#endif //AUSBEE_SIM

    xSemaphoreTake(mutex, portMAX_DELAY);

    //compute new curvilinear distance
    int32_t new_distance = motor_position[0] + motor_position[1];
    int32_t delta_distance = new_distance - _enc_distance;

    //compute new angle value
    int32_t new_angle = motor_position[0] - motor_position[1] + _enc_angle_offset;
    int32_t delta_angle = new_angle - _enc_angle;

    //compute X/Y coordonate
    float mid_angle = TICK2RAD(_enc_angle + delta_angle / 2);
    float dx = delta_distance * cos(mid_angle);
    float dy = delta_distance * sin(mid_angle);
    _enc_x += dx;
    _enc_y += dy;

    _enc_angle = new_angle;
    _enc_distance = new_distance;
    _enc_speed_distance = delta_distance;
    _enc_speed_angle = delta_angle;


    xSemaphoreGive(mutex);

    //run the asserv
    cocobot_asserv_compute();

    //wait 10ms
    vTaskDelayUntil( &xLastWakeTime, 10 / portTICK_PERIOD_MS);
  }
}


void cocobot_position_init(unsigned int task_priority)
{
  //create mutex
  mutex = xSemaphoreCreateMutex();

  //start task
  xTaskCreate(cocobot_position_task, "position", 200, NULL, task_priority, NULL);
}

float cocobot_position_get_x(void)
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  float x = TICK2MM(_enc_x);
  xSemaphoreGive(mutex);

  return x;
}

float cocobot_position_get_y(void)
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  float y = TICK2MM(_enc_y);
  xSemaphoreGive(mutex);

  return y;
}

float cocobot_position_get_distance(void)
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  float d = TICK2MM(_enc_distance);
  xSemaphoreGive(mutex);

  return d;
}

float cocobot_position_get_angle(void)
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  double a = TICK2DEG(_enc_angle);
  xSemaphoreGive(mutex);

  return a;
}

float cocobot_position_get_speed_distance(void)
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  float d = TICK2MM(_enc_speed_distance);
  xSemaphoreGive(mutex);

  return d;
}

float cocobot_position_get_speed_angle(void)
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  double a = TICK2DEG(_enc_speed_angle);
  xSemaphoreGive(mutex);

  return a;
}
