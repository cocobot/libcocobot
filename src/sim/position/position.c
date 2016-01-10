#include <cocobot.h>
#include <math.h>
#include <FreeRTOS.h>
#include <task.h>
#include <platform.h>
#include "generated/autoconf.h"
#include <stdio.h>
#include "extApi.h"

#define DEG2RAD(angle)  ((((float)angle) * M_PI) / ((float)180))

int clientID=-1;

int   left_motor_handle=-1,         right_motor_handle=-1;
float left_motor_position=0,        right_motor_position=0;
float left_motor_prev_position=0,   right_motor_prev_position=0;

float robot_x=0, robot_y=0;

float robot_distance=0,     robot_angle=0;
float robot_linear_speed=0, robot_angular_velocity=0;

static void cocobot_position_task(void * arg)
{
  //arg is always NULL. Prevent "variable unused" warning
  (void)arg;

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
    if (simxGetConnectionId(clientID)!=-1)
    {
      // update motors values
      simxGetJointPosition(clientID, left_motor_handle, &left_motor_position, simx_opmode_buffer);
      simxGetJointPosition(clientID, right_motor_handle, &right_motor_position, simx_opmode_buffer);

      //compute new curvilinear distance
      float new_distance = left_motor_position + right_motor_position;
      float delta_distance = new_distance - robot_distance;

      //compute new angle value
      float new_angle = left_motor_position - right_motor_position;
      float delta_angle = new_angle - robot_angle;

      //compute X/Y coordonate
      float mid_angle = DEG2RAD(robot_angle + delta_angle / 2);
      float dx = delta_distance * cos(mid_angle);
      float dy = delta_distance * sin(mid_angle);
      robot_x += dx;
      robot_y += dy;

      robot_angle = new_angle;
      robot_distance = new_distance;
      robot_linear_speed = delta_distance;
      robot_angular_velocity = delta_angle;

      //run the asserv
      cocobot_asserv_compute();

      //wait 10ms
      vTaskDelayUntil( &xLastWakeTime, 10 / portTICK_PERIOD_MS);
    }
    else
    {
      printf("cocobot_position_task: Connection with VREP remote API server lost\n");
      //wait 1s
      vTaskDelayUntil( &xLastWakeTime, 1000 / portTICK_PERIOD_MS);
    }
  }

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
    else
    {
      simxGetJointPosition(clientID, left_motor_handle, &left_motor_position, simx_opmode_streaming);
    }

    ret = simxGetObjectHandle(clientID, "cocobotRightMotor#", &right_motor_handle, simx_opmode_oneshot_wait);
    if (ret != 0)
    {
      printf("Error %d during remote function call. Could not get object cocobotRightMotor#\n", ret);
    }
    else
    {
      simxGetJointPosition(clientID, right_motor_handle, &right_motor_position, simx_opmode_streaming);
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
  return robot_x;
}

float cocobot_position_get_y(void)
{
  return robot_y;
}

float cocobot_position_get_distance(void)
{
  return robot_distance;
}

float cocobot_position_get_angle(void)
{
  return robot_angle;
}

float cocobot_position_get_speed_distance(void)
{
  return robot_linear_speed;
}

float cocobot_position_get_speed_angle(void)
{
  return robot_angular_velocity;
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
