#include <cocobot.h>
#include <math.h>
#include <FreeRTOS.h>
#include <task.h>
#include <platform.h>
#include "generated/autoconf.h"

#ifdef AUSBEE_SIM
#include <cocobot/vrep.h>
#endif //AUSBEE_SIM

#define DEG2RAD(angle)  ((((float)angle) * M_PI) / ((float)180))


float robot_x=0, robot_y=0;

float robot_distance=0,     robot_angle=0;
float robot_linear_speed=0, robot_angular_velocity=0;

static void cocobot_position_task(void * arg)
{
  //arg is always NULL. Prevent "variable unused" warning
  (void)arg;
  float motor_position[2] = {0, 0}; // {left, right}

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
#ifdef AUSBEE_SIM
    cocobot_vrep_get_motor_position(motor_position);
#endif //AUSBEE_SIM

    //compute new curvilinear distance
    float new_distance = motor_position[0] + motor_position[1];
    float delta_distance = new_distance - robot_distance;

    //compute new angle value
    float new_angle = motor_position[0] - motor_position[1];
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

}


void cocobot_position_init(unsigned int task_priority)
{
#ifdef AUSBEE_SIM
  cocobot_vrep_init();
#endif //AUSBEE_SIM

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
#ifdef AUSBEE_SIM
  cocobot_vrep_set_motor_command(left_motor_speed, right_motor_speed);
#endif //AUSBEE_SIM
}

void cocobot_position_set_speed_distance_angle(float linear_speed, float angular_velocity)
{
  cocobot_position_set_motor_command(linear_speed+angular_velocity, linear_speed-angular_velocity);
}
