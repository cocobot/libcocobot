#include <cocobot.h>
#include <math.h>
#include <FreeRTOS.h>
#include <task.h>
#include <platform.h>
#include "generated/autoconf.h"
#include <stdio.h>
#include "extApi.h"

int clientID=-1;
int left_motor_handle, right_motor_handle;

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
  int ret=0;
  clientID = simxStart((simxChar*)"127.0.0.1", vrep_server_port, 1, 1, 2000, 5);
  if (clientID!=-1)
  {
    printf("Connected to VREP remote API server with client ID: %d\n", clientID);

    ret = simxGetObjectHandle(clientID, "cocobotLeftMotor#", &left_motor_handle, simx_opmode_oneshot_wait);
    if (ret != 0)
    {
      printf("Error %d during remote function call. Could not get object cocobotLeftMotor#\n", ret);
    }

    ret = simxGetObjectHandle(clientID, "cocobotRightMotor#", &right_motor_handle, simx_opmode_oneshot_wait);
    if (ret != 0)
    {
      printf("Error %d during remote function call. Could not get object cocobotRightMotor#\n", ret);
    }

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

void cocobot_position_set_motor_command(float left_motor_speed, float right_motor_speed)
{
  if (simxGetConnectionId(clientID)!=-1)
  {
    simxSetJointTargetVelocity(clientID, left_motor_handle, left_motor_speed, simx_opmode_oneshot);
    simxSetJointTargetVelocity(clientID, right_motor_handle, right_motor_speed, simx_opmode_oneshot);
  }
  else
  {
      printf("cocobot_position_set_motor_command: No connection with VREP remote API server");
  }
}

void cocobot_position_set_speed_distance_angle(float linear_speed, float angular_velocity)
{
  if (simxGetConnectionId(clientID)!=-1)
  {
    cocobot_position_set_motor_command(linear_speed+angular_velocity, linear_speed-angular_velocity);
  }
  else
  {
      printf("cocobot_position_set_speed_distance_angle: No connection with VREP remote API server");
  }
}
