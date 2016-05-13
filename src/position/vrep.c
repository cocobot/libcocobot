#include <cocobot/vrep.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <math.h>
#include "extApi.h"
#include "generated/autoconf.h"

#define WHEEL_RADIUS_IN_METER (0.0325)

#define VREPPOS2M(pos)     (pos * WHEEL_RADIUS_IN_METER)
#define VREPPOS2TICK(pos)  (VREPPOS2M(pos) * (float)CONFIG_LIBCOCOBOT_POSITION_TICK_PER_METER / 2) // "/ 2" because distance is the sum of left and right motors

#define MAX_SPEED_IN_M_PER_S ((float)CONFIG_LIBCOCOBOT_DIST_RAMP_MAX_SPEED / 10000)
#define MAX_SPEED_IN_RAD_PER_S (MAX_SPEED_IN_M_PER_S / WHEEL_RADIUS_IN_METER)

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
static int clientID = -1;
static int robot_handle = -1;
static int sub_robot_handle[6] = {-1,-1,-1,-1,-1,-1};

static void * cocobot_vrep_task(void * args)
{
  (void)args;

  int ret = 0;
  int left_motor_handle = -1;
  int right_motor_handle = -1;

  cocobot_to_vrep_t buffer_to_vrep_sync;
  cocobot_from_vrep_t buffer_from_vrep_unsync;
  buffer_from_vrep_unsync.initialized = 1;

  ret = simxGetObjectHandle(clientID, "cocobotLeftMotor", &left_motor_handle, simx_opmode_oneshot_wait);
  if (ret != 0)
  {
    printf("Error %d during remote function call. Could not get object cocobotLeftMotor\n", ret);
  }
  else
  {
    simxGetJointPosition(clientID, left_motor_handle, &buffer_from_vrep_unsync.left_motor_position, simx_opmode_streaming);
  }

  ret = simxGetObjectHandle(clientID, "cocobotRightMotor", &right_motor_handle, simx_opmode_oneshot_wait);
  if (ret != 0)
  {
    printf("Error %d during remote function call. Could not get object cocobotRightMotor\n", ret);
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

#define CHECK_REMOTE_CALL(CALL,RETURN_CODE) {\
  RETURN_CODE=CALL; \
  if (RETURN_CODE != 0) { printf("Error %d during remote function call. Could not get sub robot object\n", RETURN_CODE); } }


static void cocobot_vrep_sub_init(void)
{
  int ret = 0;

  CHECK_REMOTE_CALL(simxGetObjectHandle(clientID, "patinGlissant1", &(sub_robot_handle[0]), simx_opmode_oneshot_wait),ret);
  CHECK_REMOTE_CALL(simxGetObjectHandle(clientID, "patinGlissant2", &(sub_robot_handle[1]), simx_opmode_oneshot_wait),ret);
  CHECK_REMOTE_CALL(simxGetObjectHandle(clientID, "patinGlissant3", &(sub_robot_handle[2]), simx_opmode_oneshot_wait),ret);
  CHECK_REMOTE_CALL(simxGetObjectHandle(clientID, "patinGlissant4", &(sub_robot_handle[3]), simx_opmode_oneshot_wait),ret);
  CHECK_REMOTE_CALL(simxGetObjectHandle(clientID, "cocobotLeftWheel"  , &(sub_robot_handle[4]), simx_opmode_oneshot_wait),ret);
  CHECK_REMOTE_CALL(simxGetObjectHandle(clientID, "cocobotRightWheel" , &(sub_robot_handle[5]), simx_opmode_oneshot_wait),ret);
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

  int ret = 0;
  int vrep_server_port = 1234;

  // Start Vrep server
  clientID = simxStart((simxChar*)"127.0.0.1", vrep_server_port, 1, 1, 2000, 5);
  if (clientID!=-1)
  {
    printf("Connected to VREP remote API server with client ID: %d\n", clientID);

    ret = simxLoadModel(clientID, "cocobot.ttm", 1, &robot_handle, simx_opmode_oneshot_wait);
    if (ret != 0)
    {
      printf("Error %d during remote function call. Could not get object cocobot\n", ret);
    }

    cocobot_vrep_sub_init();
  }
  else
  {
    printf("cocobot_position_init: Failed connecting to VREP remote API server with port %d\n", vrep_server_port);
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
    motor_position[0] = (int32_t)VREPPOS2TICK(buffer_from_vrep_sync.right_motor_position);
    motor_position[1] = (int32_t)VREPPOS2TICK(buffer_from_vrep_sync.left_motor_position);
  }
}

void cocobot_vrep_set_motor_command(float left_motor_speed, float right_motor_speed)
{
  cocobot_to_vrep_t buffer_to_vrep_unsync;

  buffer_to_vrep_unsync.initialized = 1;

  // Converting pwm on 16 bits to vrep motor speed
  left_motor_speed  = MAX_SPEED_IN_RAD_PER_S * left_motor_speed  / 0xffff;
  right_motor_speed = MAX_SPEED_IN_RAD_PER_S * right_motor_speed / 0xffff;

  buffer_to_vrep_unsync.left_motor_speed = left_motor_speed;
  buffer_to_vrep_unsync.right_motor_speed = right_motor_speed;

  pthread_mutex_lock(&lock);
  memcpy(&buffer_to_vrep, &buffer_to_vrep_unsync, sizeof(cocobot_to_vrep_t));
  pthread_mutex_unlock(&lock);
}

static void cocobot_vrep_set_sub_position(int axis)
{
  int i = 0;

  for (; i < 6; i++)
  {
    float pos[3] = {0,0,0};
    int ret = 0;

    if (axis == 0 || axis == 1)
    {
      ret = simxGetObjectPosition(clientID, sub_robot_handle[i], sim_handle_parent, pos, simx_opmode_oneshot);
      if (ret != 0 && ret != 1)
      {
        printf("Error %d during remote function call. Could not get sub robot position\n", ret);
      }

      pos[axis] = 0;

      ret = simxSetObjectPosition(clientID, sub_robot_handle[i], sim_handle_parent, pos, simx_opmode_oneshot);
      if (ret != 0 && ret != 1)
      {
        printf("Error %d during remote function call. Could not set sub robot position\n", ret);
      }
    }
    else if (axis == 2)
    {
      ret = simxSetObjectOrientation(clientID, sub_robot_handle[i], sim_handle_parent, pos, simx_opmode_oneshot);
      if (ret != 0 && ret != 1)
      {
        printf("Error %d during remote function call. Could not set sub robot orientation\n", ret);
      }
    }
  }
}

void cocobot_vrep_position_set_x(float x)
{
  float pos[3] = {0,0,0};
  int ret = 0;

  if (clientID!=-1)
  {
    ret = simxGetObjectPosition(clientID, robot_handle, -1, pos, simx_opmode_oneshot_wait);
    if (ret != 0 && ret != 1)
    {
      printf("Error %d during remote function call. Could not get robot x\n", ret);
    }

    pos[0] = x / 1000;

    ret = simxSetObjectPosition(clientID, robot_handle, -1, pos, simx_opmode_oneshot);
    if (ret != 0 && ret != 1)
    {
      printf("Error %d during remote function call. Could not set robot x\n", ret);
    }

    cocobot_vrep_set_sub_position(0);
  }
  else
  {
    printf("cocobot_vrep_position_set_x: Connection to VREP remote API server not yet established\n");
    exit(EXIT_FAILURE);
  }
}

void cocobot_vrep_position_set_y(float y)
{
  float pos[3] = {0,0,0};
  int ret = 0;

  if (clientID!=-1)
  {
    ret = simxGetObjectPosition(clientID, robot_handle, -1, pos, simx_opmode_oneshot_wait);
    if (ret != 0 && ret != 1)
    {
      printf("Error %d during remote function call. Could not get robot y\n", ret);
    }

    pos[1] = y / 1000;

    ret = simxSetObjectPosition(clientID, robot_handle, -1, pos, simx_opmode_oneshot);
    if (ret != 0 && ret != 1)
    {
      printf("Error %d during remote function call. Could not set robot y\n", ret);
    }

    cocobot_vrep_set_sub_position(1);
  }
  else
  {
    printf("cocobot_vrep_position_set_y: Connection to VREP remote API server not yet established\n");
    exit(EXIT_FAILURE);
  }
}

void cocobot_vrep_position_set_angle(float angle)
{
  float or[3] = {0,0,0};
  int ret = 0;

  if (clientID!=-1)
  {
    or[2] = angle * M_PI / 180;

    ret = simxSetObjectOrientation(clientID, robot_handle, -1, or, simx_opmode_oneshot);
    if (ret != 0 && ret != 1)
    {
      printf("Error %d during remote function call. Could not set robot orientation\n", ret);
    }

    cocobot_vrep_set_sub_position(2);
  }
  else
  {
    printf("cocobot_vrep_position_set_angle: Connection to VREP remote API server not yet established\n");
    exit(EXIT_FAILURE);
  }
}
