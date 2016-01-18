#include <cocobot.h>
#include <math.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "generated/autoconf.h"

#ifdef AUSBEE_SIM
#include <cocobot/vrep.h>
#else
#include <cocobot/encoders.h>
#endif //AUSBEE_SIM

//useful macros
#define TICK2RAD(tick)  ((((float)tick) * M_PI) / ((float)CONFIG_LIBCOCOBOT_POSITION_TICK_PER_180DEG))
#define TICK2DEG(tick)  ((((float)tick) * 180.0) / ((float)CONFIG_LIBCOCOBOT_POSITION_TICK_PER_180DEG))
#define TICK2MM(tick)  ((((float)tick) * 1000.0) / ((float)CONFIG_LIBCOCOBOT_POSITION_TICK_PER_METER))

//mutex for internal value access
static SemaphoreHandle_t mutex;

//internal values (in tick)
static float robot_x=0, robot_y=0;
static int32_t robot_distance=0,     robot_angle=0, robot_angle_offset=0;
static int32_t robot_linear_speed=0, robot_angular_velocity=0;

static void cocobot_position_task(void * arg)
{
  //arg is always NULL. Prevent "variable unused" warning
  (void)arg;
#ifdef AUSBEE_SIM
  float motor_position[2] = {0, 0}; // {left, right}
#else
  int32_t motor_position[2] = {0, 0}; // {left, right}
#endif //AUSBEE_SIM

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
    //update encoder values
#ifdef AUSBEE_SIM
    cocobot_vrep_get_motor_position(motor_position);
#else
    cocobot_encoders_get_motor_position(motor_position);
#endif //AUSBEE_SIM

    xSemaphoreTake(mutex, portMAX_DELAY);

    //compute new curvilinear distance
    int32_t new_distance = (int32_t)motor_position[0] + (int32_t)motor_position[1];
    int32_t delta_distance = new_distance - robot_distance;

    //compute new angle value
    int32_t new_angle = (int32_t)motor_position[0] - (int32_t)motor_position[1] + robot_angle_offset;
    int32_t delta_angle = new_angle - robot_angle;

    //compute X/Y coordonate
    float mid_angle = TICK2RAD(robot_angle + delta_angle / 2);
    float dx = delta_distance * cos(mid_angle);
    float dy = delta_distance * sin(mid_angle);
    robot_x += dx;
    robot_y += dy;

    robot_angle = new_angle;
    robot_distance = new_distance;
    robot_linear_speed = delta_distance;
    robot_angular_velocity = delta_angle;

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

#ifdef AUSBEE_SIM
  cocobot_vrep_init();
#endif //AUSBEE_SIM

  //Start task
  xTaskCreate(cocobot_position_task, "position", 200, NULL, task_priority, NULL);
}

float cocobot_position_get_x(void)
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  float x = TICK2MM(robot_x);
  xSemaphoreGive(mutex);

  return x;
}

float cocobot_position_get_y(void)
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  float y = TICK2MM(robot_y);
  xSemaphoreGive(mutex);

  return y;
}

float cocobot_position_get_distance(void)
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  float d = TICK2MM(robot_distance);
  xSemaphoreGive(mutex);

  return d;
}

float cocobot_position_get_angle(void)
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  double a = TICK2DEG(robot_angle);
  xSemaphoreGive(mutex);

  return a;
}

float cocobot_position_get_speed_distance(void)
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  float d = TICK2MM(robot_linear_speed);
  xSemaphoreGive(mutex);

  return d;
}

float cocobot_position_get_speed_angle(void)
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  double a = TICK2DEG(robot_angular_velocity);
  xSemaphoreGive(mutex);

  return a;
}

void cocobot_position_set_motor_command(float left_motor_speed, float right_motor_speed)
{
#ifdef AUSBEE_SIM
  cocobot_vrep_set_motor_command(left_motor_speed, right_motor_speed);
#else
  (void)left_motor_speed;
  (void)right_motor_speed;
#endif //AUSBEE_SIM
}

void cocobot_position_set_speed_distance_angle(float linear_speed, float angular_velocity)
{
  cocobot_position_set_motor_command(linear_speed+angular_velocity, linear_speed-angular_velocity);
}
