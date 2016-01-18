#include <cocobot/vrep.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include "extApi.h"

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

static void * cocobot_vrep_task(void * args)
{
  (void)args;

  int vrep_server_port = 1234;
  int ret = 0;
  int clientID = -1;
  int left_motor_handle = -1;
  int right_motor_handle = -1;

  cocobot_to_vrep_t buffer_to_vrep_sync;
  cocobot_from_vrep_t buffer_from_vrep_unsync;
  buffer_from_vrep_unsync.initialized = 1;

  // Start Vrep server
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
      simxGetJointPosition(clientID, left_motor_handle, &buffer_from_vrep_unsync.left_motor_position, simx_opmode_streaming);
    }

    ret = simxGetObjectHandle(clientID, "cocobotRightMotor#", &right_motor_handle, simx_opmode_oneshot_wait);
    if (ret != 0)
    {
      printf("Error %d during remote function call. Could not get object cocobotRightMotor#\n", ret);
    }
    else
    {
      simxGetJointPosition(clientID, right_motor_handle, &buffer_from_vrep_unsync.right_motor_position, simx_opmode_streaming);
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

void cocobot_vrep_init(void)
{
  buffer_from_vrep.initialized = 0;
  buffer_to_vrep.initialized = 0;

  if (pthread_mutex_init(&lock, NULL) != 0)
  {
    perror("Unable to create mutex for V-REP");
    exit(EXIT_FAILURE);
  }

  pthread_t vrep_thread;
  if(pthread_create(&vrep_thread, NULL, cocobot_vrep_task, NULL) != 0)
  {
    perror("Unable to create thread for V-REP");
    exit(EXIT_FAILURE);
  }
}

void cocobot_vrep_get_motor_position(int32_t motor_position[2])
{
  cocobot_from_vrep_t buffer_from_vrep_sync;

  pthread_mutex_lock(&lock);
  memcpy(&buffer_from_vrep_sync, &buffer_from_vrep, sizeof(cocobot_from_vrep_t));
  pthread_mutex_unlock(&lock);

  if(buffer_from_vrep_sync.initialized)
  {
    motor_position[0] = (int32_t)buffer_from_vrep_sync.left_motor_position;
    motor_position[1] = (int32_t)buffer_from_vrep_sync.right_motor_position;
  }
}

void cocobot_vrep_set_motor_command(float left_motor_speed, float right_motor_speed)
{
  cocobot_to_vrep_t buffer_to_vrep_unsync;

  buffer_to_vrep_unsync.initialized = 1;
  buffer_to_vrep_unsync.left_motor_speed = left_motor_speed;
  buffer_to_vrep_unsync.right_motor_speed = right_motor_speed;

  pthread_mutex_lock(&lock);
  memcpy(&buffer_to_vrep, &buffer_to_vrep_unsync, sizeof(cocobot_to_vrep_t));
  pthread_mutex_unlock(&lock);
}
