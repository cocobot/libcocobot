#include <cocobot.h>
#include <math.h>
#include <FreeRTOS.h>
#include <task.h>
#include <platform.h>
#include "generated/autoconf.h"

static void cocobot_position_task(void * arg)
{
  //arg is always NULL. Prevent "variable unused" warning
  (void)arg;

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
    // update motors values
    // TODO using Vrep functions

    //compute new curvilinear distance


    //compute new angle value


    //compute X/Y coordonate


    //run the asserv
    cocobot_asserv_compute();

    //wait 10ms
    vTaskDelayUntil( &xLastWakeTime, 10 / portTICK_PERIOD_MS);
  }
}


void cocobot_position_init(unsigned int task_priority)
{
  // TODO: Start Vrep server

  //start task
  xTaskCreate(cocobot_position_task, "position", 200, NULL, task_priority, NULL);
}

float cocobot_position_get_x(void)
{
  //return x;
}

float cocobot_position_get_y(void)
{
  //return y;
}

float cocobot_position_get_distance(void)
{
  //return d;
}

float cocobot_position_get_angle(void)
{
  //return a;
}

float cocobot_position_get_speed_distance(void)
{
  //return d;
}

float cocobot_position_get_speed_angle(void)
{
  //return a;
}
