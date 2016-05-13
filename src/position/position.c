#include <cocobot.h>
#include <platform.h>
#include <math.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "generated/autoconf.h"

#ifdef AUSBEE_SIM
#include <stdlib.h>
#include <cocobot/vrep.h>
#else
#include <cocobot/encoders.h>
#endif //AUSBEE_SIM

//useful macros
#define TICK2RAD(tick)  ((((float)tick) * M_PI) / ((float)CONFIG_LIBCOCOBOT_POSITION_TICK_PER_180DEG))
#define TICK2DEG(tick)  ((((float)tick) * 180.0) / ((float)CONFIG_LIBCOCOBOT_POSITION_TICK_PER_180DEG))
#define TICK2MM(tick)  ((((float)tick) * 1000.0) / ((float)CONFIG_LIBCOCOBOT_POSITION_TICK_PER_METER))
#define MM2TICK(mm)  (((float)mm) * ((float)CONFIG_LIBCOCOBOT_POSITION_TICK_PER_METER) / 1000.0)
#define DEG2TICK(deg)  ((((float)deg) / 180.0) * ((float)CONFIG_LIBCOCOBOT_POSITION_TICK_PER_180DEG))

//mutex for internal value access
static SemaphoreHandle_t mutex;

//internal values (in tick)
static float robot_x=0, robot_y=0;
static int32_t robot_distance=0,     robot_angle=0, robot_angle_offset=0;
static int32_t robot_linear_speed=0, robot_angular_velocity=0;
int32_t motor_position[2] = {0, 0}; // {right, left}
static int position_debug = 0;
#ifdef AUSBEE_SIM
int fake_vrep = 0;
#endif
static float last_left_sp = 0;
static float last_right_sp = 0;
static float left_motor_alpha = (((float)CONFIG_LIBCOCOBOT_LEFT_MOTOR_ALPHA) / 1000.0f);
static float right_motor_alpha = (((float)CONFIG_LIBCOCOBOT_RIGHT_MOTOR_ALPHA) / 1000.0f);

static void cocobot_position_compute(void)
{
  //update encoder values
#ifdef AUSBEE_SIM
  if(!fake_vrep)
  {
    cocobot_vrep_get_motor_position(motor_position);
  }
#else
  cocobot_encoders_get_motor_position(motor_position);
#endif //AUSBEE_SIM

  xSemaphoreTake(mutex, portMAX_DELAY);

  //compute new curvilinear distance
  int32_t new_distance = motor_position[0] + motor_position[1];
  int32_t delta_distance = new_distance - robot_distance;

  //compute new angle value
  int32_t new_angle = motor_position[0] - motor_position[1] + robot_angle_offset;
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
}

static void cocobot_position_task(void * arg)
{
  //arg is always NULL. Prevent "variable unused" warning
  (void)arg;

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
    cocobot_position_compute();

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
  char * fvrep = getenv("FAKE_VREP");
  if(fvrep != NULL)
  {
    if(strcmp(getenv("FAKE_VREP"), "1") == 0) 
    {
      fake_vrep = 1;
    }
  }
  if(!fake_vrep)
  {
    cocobot_vrep_init();
  }
#endif //AUSBEE_SIM

  //Be sure that position manager have valid values before continuing the initialization process.
  //Need to run twice in order to intialize speed values
  cocobot_position_compute();
  cocobot_position_compute();

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
  if(left_motor_speed > 0xffff)
  {
    left_motor_speed = 0xffff;
  }
  if(left_motor_speed < -0xffff)
  {
    left_motor_speed = -0xffff;
  }
  if(right_motor_speed > 0xffff)
  {
    right_motor_speed = 0xffff;
  }
  if(right_motor_speed < -0xffff)
  {
    right_motor_speed = -0xffff;
  }

#ifdef AUSBEE_SIM
  if(!fake_vrep)
  {
    cocobot_vrep_set_motor_command(left_motor_speed, right_motor_speed);
  }
  else
  {
    motor_position[0] += right_motor_speed / 100.0f;
    motor_position[1] += left_motor_speed / 100.0f;
  }
#else

#ifdef COCOBOT_INVERT_LEFT_MOTOR
  left_motor_speed = -left_motor_speed;
#endif
#ifdef COCOBOT_INVERT_RIGHT_MOTOR
  right_motor_speed = -right_motor_speed;
#endif

  if(left_motor_speed >= 0)
  {
#ifdef COCOBOT_LMD18200T
    left_motor_speed = 0xFFFF - left_motor_speed;
#endif
    platform_gpio_clear(PLATFORM_GPIO_MOTOR_DIR_LEFT);
    platform_motor_set_left_duty_cycle(left_motor_speed);
  }
  else
  {
    left_motor_speed = -left_motor_speed;

#ifdef COCOBOT_LMD18200T
    left_motor_speed = 0xFFFF - left_motor_speed;
#endif
    platform_gpio_set(PLATFORM_GPIO_MOTOR_DIR_LEFT);
    platform_motor_set_left_duty_cycle(left_motor_speed);
  }

  if(right_motor_speed >= 0)
  {
#ifdef COCOBOT_LMD18200T
    right_motor_speed = 0xFFFF - right_motor_speed;
#endif
    platform_gpio_clear(PLATFORM_GPIO_MOTOR_DIR_RIGHT);
    platform_motor_set_right_duty_cycle(right_motor_speed);
  }
  else
  {
    right_motor_speed = -right_motor_speed;

#ifdef COCOBOT_LMD18200T
    right_motor_speed = 0xFFFF - right_motor_speed;
#endif

    platform_gpio_set(PLATFORM_GPIO_MOTOR_DIR_RIGHT);
    platform_motor_set_right_duty_cycle(right_motor_speed);
  }
#endif //AUSBEE_SIM
}

void cocobot_position_set_speed_distance_angle(float linear_speed, float angular_velocity)
{
  float c1 = linear_speed + angular_velocity;
  float c2 = linear_speed - angular_velocity;

  float k1 = 1;
  float k2 = 1;

  if(fabsf(c1) > 0xFFFF)
  {
    k1 = ((float)0xFFFF) / fabsf(c1);
  }
  if(fabsf(c2) > 0xFFFF)
  {
    k2 = ((float)0xFFFF) / fabsf(c2);
  }

  if(k2 < k1)
  {
    k1 = k2;
  }

  float left_sp = k1 * (linear_speed-angular_velocity);
  float right_sp = k1 * (linear_speed+angular_velocity);

  left_sp = last_left_sp + left_motor_alpha * (left_sp - last_left_sp);
  right_sp = last_right_sp + right_motor_alpha * (right_sp - last_right_sp);

  cocobot_position_set_motor_command(left_sp, right_sp);
}

int cocobot_position_handle_console(char * command)
{
  if(strcmp(command,"position_debug") == 0)
  {
    cocobot_console_get_iargument(0, &position_debug);
    cocobot_console_send_answer("%d", position_debug);
    return 1;
  }

  if(strcmp(command,"left_motor_alpha") == 0)
  {
    float set;
    if(cocobot_console_get_fargument(0, &set))
    {
      left_motor_alpha = set / 1000.0;
    }
    cocobot_console_send_answer("%.3f", (double)(left_motor_alpha * 1000));
    return 1;
  }

  if(strcmp(command,"right_motor_alpha") == 0)
  {
    float set;
    if(cocobot_console_get_fargument(0, &set))
    {
      right_motor_alpha = set / 1000.0;
    }
    cocobot_console_send_answer("%.3f", (double)(right_motor_alpha * 1000));
    return 1;
  }

  return 0;
}

void cocobot_position_handle_async_console(void)
{
  if(position_debug)
  {
    cocobot_console_send_asynchronous("position", "%.3f,%.3f,%.3f,%.3f",
                                     (double)cocobot_position_get_x(),
                                     (double)cocobot_position_get_y(),
                                     (double)cocobot_position_get_angle(),
                                     (double)cocobot_position_get_distance()
                                    );
  }
}

void cocobot_position_set_x(float x)
{
  cocobot_asserv_state_t saved_state = cocobot_asserv_get_state();

  cocobot_asserv_set_state(COCOBOT_ASSERV_DISABLE);

  xSemaphoreTake(mutex, portMAX_DELAY);
  robot_x = MM2TICK(x);
  xSemaphoreGive(mutex);
#ifdef AUSBEE_SIM
  if(!fake_vrep)
  {
    cocobot_vrep_position_set_x(x);
  }
#endif

  cocobot_asserv_set_state(saved_state);
}

void cocobot_position_set_y(float y)
{
  cocobot_asserv_state_t saved_state = cocobot_asserv_get_state();

  cocobot_asserv_set_state(COCOBOT_ASSERV_DISABLE);

  xSemaphoreTake(mutex, portMAX_DELAY);
  robot_y = MM2TICK(y);
  xSemaphoreGive(mutex);
#ifdef AUSBEE_SIM
  if(!fake_vrep)
  {
    cocobot_vrep_position_set_y(y);
  }
#endif

  cocobot_asserv_set_state(saved_state);
}

void cocobot_position_set_angle(float angle)
{
  cocobot_asserv_state_t saved_state = cocobot_asserv_get_state();

  cocobot_asserv_set_state(COCOBOT_ASSERV_DISABLE);

  xSemaphoreTake(mutex, portMAX_DELAY);
  float old_angle = TICK2DEG(robot_angle);
  float diff = angle - old_angle;
  robot_angle_offset += DEG2TICK(diff);
  xSemaphoreGive(mutex);
#ifdef AUSBEE_SIM
  if(!fake_vrep)
  {
    cocobot_vrep_position_set_angle(angle);
  }
#endif

  cocobot_asserv_set_state(saved_state);
}
