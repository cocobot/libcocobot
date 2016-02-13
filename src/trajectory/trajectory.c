#include <string.h>
#include <cocobot.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include <math.h>

#define TRAJECTORY_MAX_ORDER 16

typedef enum
{
  COCOBOT_TRAJECTORY_GOTO_D,
  COCOBOT_TRAJECTORY_GOTO_A,
  COCOBOT_TRAJECTORY_GOTO_XY,
  COCOBOT_TRAJECTORY_GOTO_XY_BACKWARD,
  COCOBOT_TRAJECTORY_GOTO_XY_CIRCLE,
  COCOBOT_TRAJECTORY_GOTO_XY_CIRCLE_BACKWARD,
} cocobot_trajectory_order_type_t;

typedef enum
{
  COCOBOT_TRAJECTORY_ORDER_IN_PROGRESS,
  COCOBOT_TRAJECTORY_ORDER_DONE,
} cocobot_trajectory_order_status_t;

typedef struct
{
  cocobot_trajectory_order_type_t type;
  cocobot_trajectory_handle_t handle;
  float time;
  union
  {
    struct
    {
      float distance;
    } d_order;

    struct
    {
      float angle;
    } a_order;

    struct
    {
      float x;
      float y;
    } xy_order;

    struct
    {
      float xi;
      float yi;
      float xe;
      float ye;
    } circle_order;
  };

  uint8_t initialized;

  struct
  {
    float distance;
    float angle;
    float x;
    float y;
  }start;

} cocobot_trajectory_order_t;

//create an list of order (implemented by a circular buffer of struct cocobot_trajectory_order_t)
static cocobot_trajectory_order_t order_list[TRAJECTORY_MAX_ORDER];
static unsigned int order_list_write;
static unsigned int order_list_read;

//mutex for order_list access
static SemaphoreHandle_t mutex;

//handle generator internal counter
static cocobot_trajectory_handle_t last_handle;

//handle end of trajectory point list
static volatile cocobot_trajectory_result_t result;

static float cocobot_trajectory_find_best_angle(cocobot_trajectory_order_t * order, float angle)
{
  //get modulo multiplier
  float abase = floorf(order->start.angle / 360.0f) * 360.0f;

  //set set point in valid range (-180 > 180)
  while(angle > 180.0f) {
    angle -= 360.0f;
  }
  while(angle < -180.0f) {
    angle += 360.0f;
  }

  //find best target
  float t1 = abase + angle;
  float t2 = abase + angle + 360.0f;
  float t3 = abase + angle - 360.0f;
  float t4 = abase + angle - 2 * 360.0f;

  float dt1 = fabsf(t1 - order->start.angle);
  float dt2 = fabsf(t2 - order->start.angle);
  float dt3 = fabsf(t3 - order->start.angle);
  float dt4 = fabsf(t4 - order->start.angle);

  float target = t1;
  float dtarget = dt1;
  if(dt2 < dtarget)
  {
    target = t2;
    dtarget = dt2;
  }
  if(dt3 < dtarget)
  {
    target = t3;
    dtarget = dt3;
  }
  if(dt4 < dtarget)
  {
    target = t4;
    dtarget = dt4;
  }

  return target;
}

static cocobot_trajectory_order_status_t cocobot_trajectory_handle_type_d(cocobot_trajectory_order_t * order)
{
  float l = fabsf(order->start.distance + order->d_order.distance - cocobot_position_get_distance());
  if(l < 3.0f)
  {
    return COCOBOT_TRAJECTORY_ORDER_DONE;
  }

  cocobot_asserv_set_distance_set_point(order->start.distance + order->d_order.distance);

  return COCOBOT_TRAJECTORY_ORDER_IN_PROGRESS;
}

static cocobot_trajectory_order_status_t cocobot_trajectory_handle_type_a(cocobot_trajectory_order_t * order)
{
  float target = cocobot_trajectory_find_best_angle(order, order->a_order.angle);

  float l = fabsf(target - cocobot_position_get_angle());
  if(l < 2.5f)
  {
    return COCOBOT_TRAJECTORY_ORDER_DONE;;
  }

  cocobot_asserv_set_angular_set_point(target);

  return COCOBOT_TRAJECTORY_ORDER_IN_PROGRESS;
}

void cocobot_trajectory_task(void * arg)
{
  //arg is always NULL. Prevent "variable unused" warning
  (void)arg;

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
    cocobot_trajectory_order_t * order = NULL;

    //get current order
    xSemaphoreTake(mutex, portMAX_DELAY);
    if(order_list_read != order_list_write)
    {
      order = &order_list[order_list_read];
    }
    xSemaphoreGive(mutex);

    if(order != NULL)
    {
      //check if order is new
      if(!order->initialized)
      {
        //set real start values
        order->start.distance = cocobot_position_get_distance();
        order->start.angle = cocobot_position_get_angle();
        order->start.x = cocobot_position_get_x();
        order->start.y = cocobot_position_get_y();

        order->initialized = 1;
      }

      cocobot_trajectory_order_status_t status;
      //find the right order handler
      switch(order->type)
      {
        case COCOBOT_TRAJECTORY_GOTO_D:
          status = cocobot_trajectory_handle_type_d(order);
          break;

        case COCOBOT_TRAJECTORY_GOTO_A:
          status = cocobot_trajectory_handle_type_a(order);
          break;

        default:
          //something is broken, signal to user and try next order
          cocobot_console_send_asynchronous("trajectory", "Unknown order type");
          status = COCOBOT_TRAJECTORY_ORDER_DONE;
          break;
      }

      //remove order of the list if needed
      if(status == COCOBOT_TRAJECTORY_ORDER_DONE)
      {
        xSemaphoreTake(mutex, portMAX_DELAY);
        order_list_read = (order_list_read + 1) % TRAJECTORY_MAX_ORDER;
        xSemaphoreGive(mutex);
      }
    }
    else if(result == COCOBOT_TRAJECTORY_RUNNING)
    {
      result = COCOBOT_TRAJECTORY_SUCCESS;
    }

    //wait 100ms
    vTaskDelayUntil( &xLastWakeTime, 100 / portTICK_PERIOD_MS);
  }
}


void cocobot_trajectory_init(unsigned int task_priority)
{
  //reset order_list circular buffer
  order_list_write = 0;
  order_list_read = 0;

  //create mutex
  mutex = xSemaphoreCreateMutex();

  //init handle generator
  last_handle = 0;

  //start task
  xTaskCreate(cocobot_trajectory_task, "trajectory", 200, NULL, task_priority, NULL);
}

void cocobot_add_new_order(cocobot_trajectory_order_t * order)
{
  //reset initialized flag
  order->initialized = 0;
  result = COCOBOT_TRAJECTORY_RUNNING;

  xSemaphoreTake(mutex, portMAX_DELAY);

  //compute next write position on the circuar buffer
  unsigned int next_write_position = (order_list_write + 1) % TRAJECTORY_MAX_ORDER;

  //check that circular buffer is not full
  if(order_list_read != next_write_position)
  {
    memcpy(&order_list[order_list_write], order, sizeof(cocobot_trajectory_order_t));
  }

  order_list_write = next_write_position;

  last_handle += 1;
  order->handle = last_handle;

  xSemaphoreGive(mutex);
}

cocobot_trajectory_handle_t cocobot_trajectory_goto_d(float distance, float time)
{
  //fill cocobot_trajectory_order_t struct
  cocobot_trajectory_order_t order;
  order.handle = COCOBOT_TRAJECTORY_INVALID_HANDLE;
  order.type = COCOBOT_TRAJECTORY_GOTO_D;
  order.time = time;
  order.d_order.distance = distance;

  //add order on the circular buffer
  cocobot_add_new_order(&order);

  //return the handle
  return order.handle;
}

cocobot_trajectory_handle_t cocobot_trajectory_goto_a(float angle, float time)
{
  //fill cocobot_trajectory_order_t struct
  cocobot_trajectory_order_t order;
  order.handle = COCOBOT_TRAJECTORY_INVALID_HANDLE;
  order.type = COCOBOT_TRAJECTORY_GOTO_A;
  order.time = time;
  order.a_order.angle = angle;

  //add order on the circular buffer
  cocobot_add_new_order(&order);

  //return the handle
  return order.handle;
}

cocobot_trajectory_handle_t cocobot_trajectory_goto_xy(float x, float y, float time)
{
  //fill cocobot_trajectory_order_t struct
  cocobot_trajectory_order_t order;
  order.handle = COCOBOT_TRAJECTORY_INVALID_HANDLE;
  order.type = COCOBOT_TRAJECTORY_GOTO_XY;
  order.time = time;
  order.xy_order.x = x;
  order.xy_order.y = y;

  //add order on the circular buffer
  cocobot_add_new_order(&order);

  //return the handle
  return order.handle;
}

cocobot_trajectory_handle_t cocobot_trajectory_goto_xy_backward(float x, float y, float time)
{
  //fill cocobot_trajectory_order_t struct
  cocobot_trajectory_order_t order;
  order.handle = COCOBOT_TRAJECTORY_INVALID_HANDLE;
  order.type = COCOBOT_TRAJECTORY_GOTO_XY_BACKWARD;
  order.time = time;
  order.xy_order.x = x;
  order.xy_order.y = y;

  //add order on the circular buffer
  cocobot_add_new_order(&order);

  //return the handle
  return order.handle;
}

cocobot_trajectory_handle_t cocobot_trajectory_goto_xy_circle(float xi, float yi, float xe, float ye, float time)
{
  //fill cocobot_trajectory_order_t struct
  cocobot_trajectory_order_t order;
  order.handle = COCOBOT_TRAJECTORY_INVALID_HANDLE;
  order.type = COCOBOT_TRAJECTORY_GOTO_XY_CIRCLE;
  order.time = time;
  order.circle_order.xi = xi;
  order.circle_order.yi = yi;
  order.circle_order.xe = xe;
  order.circle_order.ye = ye;

  //add order on the circular buffer
  cocobot_add_new_order(&order);

  //return the handle
  return order.handle;
}

cocobot_trajectory_handle_t cocobot_trajectory_goto_xy_circle_backward(float xi, float yi, float xe, float ye, float time)
{
  //fill cocobot_trajectory_order_t struct
  cocobot_trajectory_order_t order;
  order.handle = COCOBOT_TRAJECTORY_INVALID_HANDLE;
  order.type = COCOBOT_TRAJECTORY_GOTO_XY_CIRCLE_BACKWARD;
  order.time = time;
  order.circle_order.xi = xi;
  order.circle_order.yi = yi;
  order.circle_order.xe = xe;
  order.circle_order.ye = ye;

  //add order on the circular buffer
  cocobot_add_new_order(&order);

  //return the handle
  return order.handle;
}

cocobot_trajectory_result_t cocobot_trajectory_wait(void)
{
  while(result == COCOBOT_TRAJECTORY_RUNNING);

  return result;
}
