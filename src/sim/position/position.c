#include <cocobot.h>
#include <math.h>
#include <FreeRTOS.h>
#include <task.h>
#include <platform.h>
#include "generated/autoconf.h"
#include <stdio.h>
#include "extApi.h"

int clientID=-1;

static void cocobot_position_task(void * arg)
{
  //arg is always NULL. Prevent "variable unused" warning
  (void)arg;

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(simxGetConnectionId(clientID)!=-1)
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


  printf("cocobot_position_task: Connection with VREP remote API server lost\n");
}


void cocobot_position_init(unsigned int task_priority)
{
  // Start Vrep server
  int vrep_server_port=1234;
  clientID = simxStart((simxChar*)"127.0.0.1", vrep_server_port, 1, 1, 2000, 5);
  if (clientID!=-1)
  {
    printf("Connected to VREP remote API server with client ID: %d\n", clientID);

    // Start task
    xTaskCreate(cocobot_position_task, "position", 200, NULL, task_priority, NULL);
  }
  else
  {
    printf("cocobot_position_init: Failed connecting to VREP remote API server with port %d\n", vrep_server_port);
  }
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
