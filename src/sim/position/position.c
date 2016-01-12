#include <cocobot.h>
#include <math.h>
#include <FreeRTOS.h>
#include <task.h>
#include <platform.h>
#include "generated/autoconf.h"
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include "extApi.h"

#define DEG2RAD(angle)  ((((float)angle) * M_PI) / ((float)180))


typedef struct 
{
  int initialized;
  float left_motor_position;
  float right_motor_position;
} cocobot_from_vrep_t;

typedef struct 
{
  int initialized;
  float left_motor_speed;
  float right_motor_speed;
} cocobot_to_vrep_t;

static cocobot_from_vrep_t buffer_from_vrep;
static cocobot_to_vrep_t buffer_to_vrep;

static pthread_mutex_t lock;

float robot_x=0, robot_y=0;

float robot_distance=0,     robot_angle=0;
float robot_linear_speed=0, robot_angular_velocity=0;

static void cocobot_position_task(void * arg)
{
  //arg is always NULL. Prevent "variable unused" warning
  (void)arg;

  cocobot_from_vrep_t buffer_from_vrep_sync;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
    pthread_mutex_lock(&lock);
    memcpy(&buffer_from_vrep_sync, &buffer_from_vrep, sizeof(cocobot_from_vrep_t));
    pthread_mutex_unlock(&lock);

    if(buffer_from_vrep_sync.initialized) 
    {
      //compute new curvilinear distance
      float new_distance = buffer_from_vrep_sync.left_motor_position + buffer_from_vrep_sync.right_motor_position;
      float delta_distance = new_distance - robot_distance;

      //compute new angle value
      float new_angle = buffer_from_vrep_sync.left_motor_position - buffer_from_vrep_sync.right_motor_position;
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
    }

    //wait 10ms
    vTaskDelayUntil( &xLastWakeTime, 10 / portTICK_PERIOD_MS);
  }

}

static void * cocobot_position_vrep(void * args)
{
  (void)args;

  // Start Vrep server
  int vrep_server_port = 1234;
  int ret = 0;
  int clientID = -1;
  int left_motor_handle = -1;
  int right_motor_handle = -1;

  cocobot_to_vrep_t buffer_to_vrep_sync;
  cocobot_from_vrep_t buffer_from_vrep_unsync;
  buffer_from_vrep_unsync.initialized = 1;

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


    while (simxGetConnectionId(clientID)!=-1)
    {
      simxGetJointPosition(clientID, left_motor_handle, &buffer_from_vrep_unsync.left_motor_position, simx_opmode_buffer);
      simxGetJointPosition(clientID, right_motor_handle, &buffer_from_vrep_unsync.right_motor_position, simx_opmode_buffer);

      pthread_mutex_lock(&lock);
      memcpy(&buffer_from_vrep, &buffer_from_vrep_unsync, sizeof(cocobot_from_vrep_t));
      memcpy(&buffer_to_vrep_sync, &buffer_to_vrep, sizeof(cocobot_to_vrep_t));
      pthread_mutex_unlock(&lock);

      if(buffer_to_vrep_sync.initialized)
      {
        simxSetJointTargetVelocity(clientID, left_motor_handle, buffer_to_vrep_sync.left_motor_speed, simx_opmode_oneshot);
        simxSetJointTargetVelocity(clientID, right_motor_handle, buffer_to_vrep_sync.right_motor_speed, simx_opmode_oneshot);
      }

      extApi_sleepMs(5);
    }
  }
  else
  {
    printf("cocobot_position_init: Failed connecting to VREP remote API server with port %d\n", vrep_server_port);
    exit(EXIT_FAILURE);
  }
}


void cocobot_position_init(unsigned int task_priority)
{
  buffer_from_vrep.initialized = 0;
  buffer_to_vrep.initialized = 0;

  if (pthread_mutex_init(&lock, NULL) != 0)
  {
    perror("Unable to create mutex for V-REP");
    exit(EXIT_FAILURE);
  }

  pthread_t vrep_thread;
  if(pthread_create(&vrep_thread, NULL, cocobot_position_vrep, NULL) != 0)
  {
    perror("Unable to create thread for V-REP");
    exit(EXIT_FAILURE);
  }

  // Start task
  xTaskCreate(cocobot_position_task, "position", 200, NULL, task_priority, NULL); 
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
   cocobot_to_vrep_t buffer_to_vrep_unsync;
   
   buffer_to_vrep_unsync.initialized = 1;
   buffer_to_vrep_unsync.left_motor_speed = left_motor_speed;
   buffer_to_vrep_unsync.right_motor_speed = right_motor_speed;

   pthread_mutex_lock(&lock);
   memcpy(&buffer_to_vrep, &buffer_to_vrep_unsync, sizeof(cocobot_to_vrep_t));
   pthread_mutex_unlock(&lock);
}

void cocobot_position_set_speed_distance_angle(float linear_speed, float angular_velocity)
{
  cocobot_position_set_motor_command(linear_speed+angular_velocity, linear_speed-angular_velocity);
}
