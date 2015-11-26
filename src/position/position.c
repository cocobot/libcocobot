#include <cocobot.h>
#include <math.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <platform.h>
#include "generated/autoconf.h"

//useful macros
#define TICK2RAD(tick)  ((((float)tick) * M_PI) / ((float)CONFIG_LIBCOCOBOT_POSITION_TICK_PER_180DEG))
#define TICK2DEG(tick)  ((((float)tick) * 180.0) / ((float)CONFIG_LIBCOCOBOT_POSITION_TICK_PER_180DEG))
#define TICK2MM(tick)  ((((float)tick) * 1000.0) / ((float)CONFIG_LIBCOCOBOT_POSITION_TICK_PER_METER))

//mutex for internal value access
static SemaphoreHandle_t mutex;

//internal values (in tick)
static float _enc_x;
static float _enc_y;
static int32_t _enc_angle;
static int32_t _enc_angle_offset;
static int32_t _enc_distance;
static int32_t _enc_value[2];
static uint16_t _enc_last_angle[2];

static void cocobot_position_encoder_update(int id)
{
  uint16_t raw;

  if(id == 0)
  {
    platform_spi_position_select(PLATFORM_SPI_ENCR_SELECT);
  }
  else
  {
    platform_spi_position_select(PLATFORM_SPI_ENCL_SELECT);
  }

  raw = platform_spi_position_transfert(0xff) & 0xFF;
  raw |= (platform_spi_position_transfert(0xff) << 8) & 0xFF00;

  platform_spi_position_select(PLATFORM_SPI_CS_UNSELECT);
  
  //swap octets + delete MSB
  raw = ((raw & 0xff) << 8) | (raw >> 8);
  raw <<= 1;

  //compute new angle by abusing 16 bits integer overflow
  int16_t delta = raw - _enc_last_angle[id];
  _enc_last_angle[id] = raw;
  if(id == 0)
  {
    _enc_value[id] += delta;
  }
  else
  {
    _enc_value[id] -= delta;
  }
}

static void cocobot_position_task(void * arg)
{
  //arg is always NULL. Prevent "variable unused" warning
  (void)arg;

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
    //update encoder values
    cocobot_position_encoder_update(0);
    cocobot_position_encoder_update(1);

    xSemaphoreTake(mutex, portMAX_DELAY);

    //compute new curvilinear distance
    int32_t new_distance = _enc_value[0] + _enc_value[1];
    int32_t delta_disance = new_distance - _enc_distance;

    //compute new angle value
    int32_t new_angle = _enc_value[0] - _enc_value[1] + _enc_angle_offset; 
    int32_t delta_angle = new_angle - _enc_angle;

    //compute X/Y coordonate
    float mid_angle = TICK2RAD(_enc_angle + delta_angle / 2);
    float dx = delta_disance * cos(mid_angle);
    float dy = delta_disance * sin(mid_angle);
    _enc_x += dx;
    _enc_y += dy;

    _enc_angle = new_angle;
    _enc_distance = new_distance;


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

  //start task
  xTaskCreate(cocobot_position_task, "position", 200, NULL, task_priority, NULL);
}

float cocobot_position_get_x(void)
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  float x = TICK2MM(_enc_x);
  xSemaphoreGive(mutex);

  return x;
}

float cocobot_position_get_y(void)
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  float y = TICK2MM(_enc_y);
  xSemaphoreGive(mutex);

  return y;
}

float cocobot_position_get_distance(void)
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  float d = TICK2MM(_enc_distance);
  xSemaphoreGive(mutex);

  return d;
}

float cocobot_position_get_angle(void)
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  double a = TICK2DEG(_enc_angle);
  xSemaphoreGive(mutex);

  return a;
}

