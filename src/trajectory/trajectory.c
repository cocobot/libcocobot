#include <string.h>
#include <cocobot.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include <event_groups.h>
#include <math.h>

#define TRAJECTORY_MAX_ORDER 32
#define TRAJECTORY_D_STOP_MM  3.0f
#define TRAJECTORY_XY_STOP_MM  3.0f
#define TRAJECTORY_XY_STOP_ANGLE_DEG 30.0f
#define TRAJECTORY_EST_STOP_ANGLE_DEG 30.0f

#define TRAJECTORY_FLAG_SLOW_DOWN (1 << 0)

#define BIT_0 (1 << 0)

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
      float d_stop;
    } xy_order;

    struct
    {
      float xc;
      float yc;
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
    TickType_t time;
  }start;

  struct
  {
    float x;
    float y;
    float angle;
  }estimated_start;

  struct
  {
    float x;
    float y;
    float angle;
  }estimated_end;

  float estimated_distance_before_stop;

} cocobot_trajectory_order_t;

//create an list of order (implemented by a circular buffer of struct cocobot_trajectory_order_t)
static cocobot_trajectory_order_t order_list[TRAJECTORY_MAX_ORDER];
static unsigned int order_list_write;
static unsigned int order_list_read;
static int estimations_need_recompute;

//mutex for order_list access
static SemaphoreHandle_t mutex;

//handle generator internal counter
static cocobot_trajectory_handle_t last_handle;

//handle end of trajectory point list
static volatile cocobot_trajectory_result_t result;

static EventGroupHandle_t no_more_orders;

static cocobot_trajectory_xy_default_t xy_pref;

static int enable_opponent_detection;

static float cocobot_trajectory_find_best_angle(float current_angle, float angle)
{
  //get modulo multiplier
  float abase = floorf(current_angle / 360.0f) * 360.0f;

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

  float dt1 = fabsf(t1 - current_angle);
  float dt2 = fabsf(t2 - current_angle);
  float dt3 = fabsf(t3 - current_angle);
  float dt4 = fabsf(t4 - current_angle);

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
  float diff = order->start.distance + order->d_order.distance - cocobot_position_get_distance();
  float estimation = order->estimated_distance_before_stop;

  

  if(order->d_order.distance < 0)
  {
    if(enable_opponent_detection)
    {
      COCOBOT_OPPONENT_DETECTION_ENABLE_BACK();
    }
    if(diff > -TRAJECTORY_D_STOP_MM)
    {
      return COCOBOT_TRAJECTORY_ORDER_DONE;
    }

    estimation = -estimation;
  }
  else
  {
    if(enable_opponent_detection)
    {
      COCOBOT_OPPONENT_DETECTION_ENABLE_FRONT();
    }
    if(diff < TRAJECTORY_D_STOP_MM)
    {
      return COCOBOT_TRAJECTORY_ORDER_DONE;
    }
  }

  cocobot_asserv_set_distance_set_point(order->start.distance + order->d_order.distance + estimation);

  return COCOBOT_TRAJECTORY_ORDER_IN_PROGRESS;
}

static cocobot_trajectory_order_status_t cocobot_trajectory_handle_type_a(cocobot_trajectory_order_t * order)
{
  float target = cocobot_trajectory_find_best_angle(order->start.angle, order->a_order.angle);

  float l = fabsf(target - cocobot_position_get_angle());
  if(l < 2.5f)
  {
    return COCOBOT_TRAJECTORY_ORDER_DONE;;
  }

  cocobot_asserv_set_angular_set_point(target);
  cocobot_asserv_set_distance_set_point(order->start.distance);

  return COCOBOT_TRAJECTORY_ORDER_IN_PROGRESS;
}

static cocobot_trajectory_order_status_t cocobot_trajectory_handle_type_xy(cocobot_trajectory_order_t * order)
{
  float x = cocobot_position_get_x();
  float y = cocobot_position_get_y();

  float tx = order->xy_order.x - order->start.x;
  float ty = order->xy_order.y - order->start.y;
  float td = tx * tx + ty * ty;

  float cx = x - order->start.x;
  float cy = y - order->start.y;
  float cd = cx * cx + cy * cy;

  if(enable_opponent_detection)
  {
    COCOBOT_OPPONENT_DETECTION_ENABLE_FRONT();
  }

  if(sqrtf(cd) > sqrtf(td) - TRAJECTORY_XY_STOP_MM)
  {
    return COCOBOT_TRAJECTORY_ORDER_DONE;
  }
  else
  {
    float dx = order->xy_order.x - x;
    float dy = order->xy_order.y - y;
    float dd = dx * dx + dy * dy;

    float cur_angle = cocobot_position_get_angle();
    float abs_target = atan2f(dy, dx) * 180.0f / M_PI;

    float target = cocobot_trajectory_find_best_angle(cur_angle, abs_target);
    if(fabsf(cur_angle - target) < TRAJECTORY_XY_STOP_ANGLE_DEG)
    {
      order->xy_order.d_stop = NAN;
      cocobot_asserv_set_distance_set_point(cocobot_position_get_distance() + sqrtf(dd) + order->estimated_distance_before_stop);
    }
    else
    {
      if(isnan(order->xy_order.d_stop))
      {
        order->xy_order.d_stop = cocobot_position_get_distance();
      }
      cocobot_asserv_set_distance_set_point(order->xy_order.d_stop);
    }

    cocobot_asserv_set_angular_set_point(target);
  }

  return COCOBOT_TRAJECTORY_ORDER_IN_PROGRESS;
}

static cocobot_trajectory_order_status_t cocobot_trajectory_handle_type_xy_circle(cocobot_trajectory_order_t * order)
{
  float x = cocobot_position_get_x();
  float y = cocobot_position_get_y();

  float abs_end_angle = atan2f(order->circle_order.ye - order->circle_order.yc, order->circle_order.xe - order->circle_order.xc) * 180.0 /M_PI + 90;
  float abs_target_angle = atan2f(y - order->circle_order.yc, x - order->circle_order.xc) * 180.0 / M_PI + 90;
  float diff = abs_target_angle - abs_end_angle;
  while(diff > 360) {
    diff -= 360.0;
  }
  while(diff < 0) {
    diff += 360.0;
  }

  if((diff > 30) && (diff < 360 - 30))
  {
    float dcx = order->circle_order.xe - order->circle_order.xc;
    float dcy = order->circle_order.ye - order->circle_order.yc;
    float radius2 = dcx * dcx + dcy * dcy;
    float radius = sqrtf(radius2);

    float dccx = x - order->circle_order.xc;
    float dccy = y - order->circle_order.yc;
    float d2c = sqrtf(dccx * dccx + dccy * dccy);


    float dd = 2 * M_PI * radius * diff / 360.0 + fabsf(d2c - radius);

    if(dd < 2.5f)
    {
      return COCOBOT_TRAJECTORY_ORDER_DONE;
    }
    else
    {
      float target = cocobot_trajectory_find_best_angle(cocobot_position_get_angle(), abs_target_angle);
      cocobot_asserv_set_angular_set_point(target);
      cocobot_asserv_set_distance_set_point(cocobot_position_get_distance() + sqrtf(dd));
    }
  }
  else
  {
    float dx = order->circle_order.xe - x;
    float dy = order->circle_order.ye - y;
    float dd = dx * dx + dy * dy;

    if(dd < 10.0f)
    {
      return COCOBOT_TRAJECTORY_ORDER_DONE;
    }
    else
    {
      float cur_angle = cocobot_position_get_angle();
      double abs_target = atan2f(dy, dx) * 180.0 / M_PI;

      float target = cocobot_trajectory_find_best_angle(cur_angle, abs_target);
      if(fabsf(cur_angle - target) < TRAJECTORY_XY_STOP_ANGLE_DEG)
      {
        cocobot_asserv_set_distance_set_point(cocobot_position_get_distance() + sqrtf(dd));
      }
      else
      {
        cocobot_asserv_set_distance_set_point(cocobot_position_get_distance());
      }

      cocobot_asserv_set_angular_set_point(target);
    }

  }

  return COCOBOT_TRAJECTORY_ORDER_IN_PROGRESS;
}

static cocobot_trajectory_order_status_t cocobot_trajectory_handle_type_xy_backward(cocobot_trajectory_order_t * order)
{
  float x = cocobot_position_get_x();
  float y = cocobot_position_get_y();

  float tx = order->xy_order.x - order->start.x;
  float ty = order->xy_order.y - order->start.y;
  float td = tx * tx + ty * ty;

  float cx = x - order->start.x;
  float cy = y - order->start.y;
  float cd = cx * cx + cy * cy;

  if(enable_opponent_detection)
  {
    COCOBOT_OPPONENT_DETECTION_ENABLE_BACK();
  }

  if(sqrtf(cd) > sqrtf(td) - TRAJECTORY_XY_STOP_MM)
  {
    return COCOBOT_TRAJECTORY_ORDER_DONE;
  }
  else
  {
    float dx = order->xy_order.x - x;
    float dy = order->xy_order.y - y;
    float dd = dx * dx + dy * dy;

    float cur_angle = cocobot_position_get_angle();
    double abs_target = atan2f(dy, dx) * 180.0 / M_PI + 180.0;

    float target = cocobot_trajectory_find_best_angle(cur_angle, abs_target);
    if(fabsf(cur_angle - target) < TRAJECTORY_XY_STOP_ANGLE_DEG)
    {
      order->xy_order.d_stop = NAN;
      cocobot_asserv_set_distance_set_point(cocobot_position_get_distance() - sqrtf(dd) - order->estimated_distance_before_stop);
    }
    else
    {
      if(isnan(order->xy_order.d_stop))
      {
        order->xy_order.d_stop = cocobot_position_get_distance();
      }
      cocobot_asserv_set_distance_set_point(order->xy_order.d_stop);
    }

    cocobot_asserv_set_angular_set_point(target);
  }

  return COCOBOT_TRAJECTORY_ORDER_IN_PROGRESS;
}

static void cocobot_trajectory_compute_estimations(void)
{
  cocobot_trajectory_order_t * last_order = NULL;

  xSemaphoreTake(mutex, portMAX_DELAY);
  //process list forward
  int opost = order_list_read;
  while(1)
  {
    cocobot_trajectory_order_t * order = NULL;

    if(opost != (int)order_list_write)
    {
      order = &order_list[opost];
    }

    if(order == NULL)
    {
      break;
    }

    if(last_order == NULL)
    {
      order->estimated_start.x = cocobot_position_get_x();
      order->estimated_start.y = cocobot_position_get_y();
      order->estimated_start.angle = cocobot_position_get_angle();
    }
    else
    {
      order->estimated_start.x = last_order->estimated_end.x;
      order->estimated_start.y = last_order->estimated_end.y;
      order->estimated_start.angle = last_order->estimated_end.angle;
    }

    switch(order->type)
    {
      case COCOBOT_TRAJECTORY_GOTO_D:
        order->estimated_end.x = order->estimated_start.x + order->d_order.distance * cosf(order->estimated_start.angle * M_PI / 180.0);
        order->estimated_end.y = order->estimated_start.y + order->d_order.distance * sinf(order->estimated_start.angle * M_PI / 180.0);
        order->estimated_end.angle = order->estimated_start.angle;
        break;

      case COCOBOT_TRAJECTORY_GOTO_A:
        order->estimated_end.x = order->estimated_start.x;
        order->estimated_end.y = order->estimated_start.y;
        order->estimated_end.angle = order->a_order.angle;
        break;

      case COCOBOT_TRAJECTORY_GOTO_XY:
        order->estimated_end.x = order->xy_order.x;
        order->estimated_end.y = order->xy_order.y;
        order->estimated_end.angle = atan2f(order->estimated_end.y - order->estimated_start.y, order->estimated_end.x - order->estimated_start.x) * 180.0 / M_PI;
        break;

      case COCOBOT_TRAJECTORY_GOTO_XY_BACKWARD:
        order->estimated_end.x = order->xy_order.x;
        order->estimated_end.y = order->xy_order.y;
        order->estimated_end.angle = atan2f(order->estimated_end.y - order->estimated_start.y, order->estimated_end.x - order->estimated_start.x) * 180.0 / M_PI + 180.0;
        break;

      case COCOBOT_TRAJECTORY_GOTO_XY_CIRCLE:
        order->estimated_end.x = order->circle_order.xe;
        order->estimated_end.y = order->circle_order.ye;
        order->estimated_end.angle = atan2f(order->circle_order.ye - order->circle_order.yc, order->circle_order.xe - order->circle_order.xc) * 180.0 / M_PI + 90.0;
        break;



      default:
        order->estimated_end.x = order->estimated_start.x;
        order->estimated_end.y = order->estimated_start.y;
        order->estimated_end.angle = order->estimated_start.angle;
        break;
    }


    last_order = order;
    opost = (opost + 1) % TRAJECTORY_MAX_ORDER;
  }
  
  //process list backward
  opost = order_list_write;
  float distance = 0;
  float forward = 1;
  float last_angle = 0;
  while(1)
  {
    opost = (opost - 1);
    if(opost < 0) 
    {
      opost = TRAJECTORY_MAX_ORDER - 1;
    }
    cocobot_trajectory_order_t * order = NULL;

    order = &order_list[opost];


    switch(order->type)
    {
      case COCOBOT_TRAJECTORY_GOTO_D:
        if(((order->d_order.distance < 0) && (!forward)) || ((order->d_order.distance >= 0) && (forward)))
        {
          order->estimated_distance_before_stop = distance;
        }
        else
        {
          order->estimated_distance_before_stop = 0;
          distance = 0;
        }
        
        distance += fabsf(order->d_order.distance); 
        last_angle = order->estimated_end.angle;
        if(order->d_order.distance < 0)
        {
          forward = 0;
        }
        else
        {
          forward = 1;
        }
        break;

      case COCOBOT_TRAJECTORY_GOTO_XY_BACKWARD:
        {
          int iaend = (int)(order->estimated_end.angle);
          int iastart = (int)(last_angle);

          iaend = iaend % 360;
          iastart = iastart % 360;
          if(iaend < 0) 
          {
            iaend += 360;
          }
          if(iastart < 0) 
          {
            iastart += 360;
          }
          int angle = iaend - iastart;
          angle = angle % 360;

          if(angle < 0)
          {
            angle += 360;
          }
          if(angle > 180)
          {
            angle -= 360;
          }

          if((!forward) && (fabsf(angle) < TRAJECTORY_EST_STOP_ANGLE_DEG))
          {
            order->estimated_distance_before_stop = distance;
            float dx = order->estimated_end.x - order->estimated_start.x;
            float dy = order->estimated_end.y - order->estimated_start.y;
            distance += sqrtf(dx * dx + dy * dy);

          }
          else
          {
            order->estimated_distance_before_stop = 0;
            distance = 0;
          }
          last_angle = order->estimated_end.angle;
          forward = 0;
        }
        break;

      case COCOBOT_TRAJECTORY_GOTO_XY:
        {
          int iaend = (int)(order->estimated_end.angle);
          int iastart = (int)(last_angle);

          iaend = iaend % 360;
          iastart = iastart % 360;
          if(iaend < 0) 
          {
            iaend += 360;
          }
          if(iastart < 0) 
          {
            iastart += 360;
          }
          int angle = iaend - iastart;
          angle = angle % 360;

          if(angle < 0)
          {
            angle += 360;
          }
          if(angle > 180)
          {
            angle -= 360;
          }


          if((forward) && (fabsf(angle) < TRAJECTORY_EST_STOP_ANGLE_DEG))
          {
            order->estimated_distance_before_stop = distance;
          }
          else
          {
            order->estimated_distance_before_stop = 0;
            distance = 0;
          }

          float dx = order->estimated_end.x - order->estimated_start.x;
          float dy = order->estimated_end.y - order->estimated_start.y;
          distance += sqrtf(dx * dx + dy * dy);

          last_angle = order->estimated_end.angle;
          forward = 1;
        }
        break;

      default:
        order->estimated_distance_before_stop = 0;
        last_angle = order->estimated_end.angle;
        distance = 0;
        break;
    }

    if(opost == (int)order_list_read)
    {
      break;
    }
  }

  xSemaphoreGive(mutex);
}

void cocobot_trajectory_task(void * arg)
{
  //arg is always NULL. Prevent "variable unused" warning
  (void)arg;

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
    if(estimations_need_recompute)
    {
      cocobot_trajectory_compute_estimations();
      estimations_need_recompute = 0;
    }

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
        order->start.time = xTaskGetTickCount();

        COCOBOT_OPPONENT_DETECTION_DISABLE();

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

        case COCOBOT_TRAJECTORY_GOTO_XY:
          status = cocobot_trajectory_handle_type_xy(order);
          break;

        case COCOBOT_TRAJECTORY_GOTO_XY_BACKWARD:
          status = cocobot_trajectory_handle_type_xy_backward(order);
          break;

        case COCOBOT_TRAJECTORY_GOTO_XY_CIRCLE:
          status = cocobot_trajectory_handle_type_xy_circle(order);
          break;

        default:
          //something is broken, signal to user and try next order
          cocobot_console_send_asynchronous("trajectory", "Unknown order type");
          status = COCOBOT_TRAJECTORY_ORDER_DONE;
          break;
      }

      if(order->time >= 0)
      {
        if((xTaskGetTickCount() - order->start.time) * portTICK_PERIOD_MS > order->time)
        {
          status = COCOBOT_TRAJECTORY_ORDER_DONE;
          result = COCOBOT_TRAJECTORY_TIMEOUT_BEFORE_END;

          cocobot_asserv_set_distance_set_point(cocobot_position_get_distance());
          cocobot_asserv_set_angular_set_point(cocobot_position_get_angle());
        }
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
      xEventGroupSetBits(no_more_orders, BIT_0);
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
  estimations_need_recompute = 0;
  xy_pref = COCOBOT_TRAJECTORY_FORWARD;

  enable_opponent_detection = 1;

  //create mutex
  mutex = xSemaphoreCreateMutex();
  
  //create event
  no_more_orders = xEventGroupCreate();

  //init handle generator
  last_handle = 0;

  //start task
  xTaskCreate(cocobot_trajectory_task, "trajectory", 200, NULL, task_priority, NULL);
}

void cocobot_add_new_order(cocobot_trajectory_order_t * order)
{
  //reset initialized flag
  order->initialized = 0;
  order->estimated_start.x = NAN;
  order->estimated_start.y = NAN;
  order->estimated_start.angle = NAN;
  order->estimated_end.x = NAN;
  order->estimated_end.y = NAN;
  order->estimated_end.angle = NAN;
  order->estimated_distance_before_stop = 0;
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
  estimations_need_recompute = 1;

  xSemaphoreGive(mutex);
  xEventGroupClearBits(no_more_orders, BIT_0);
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
  if(xy_pref == COCOBOT_TRAJECTORY_BACKWARD)
  {
    return cocobot_trajectory_goto_xy_backward(x, y, time);
  }
  else
  {
    return cocobot_trajectory_goto_xy_forward(x, y, time);
  }
}

cocobot_trajectory_handle_t cocobot_trajectory_goto_xy_forward(float x, float y, float time)
{
  //fill cocobot_trajectory_order_t struct
  cocobot_trajectory_order_t order;
  order.handle = COCOBOT_TRAJECTORY_INVALID_HANDLE;
  order.type = COCOBOT_TRAJECTORY_GOTO_XY;
  order.time = time;
  order.xy_order.x = x;
  order.xy_order.y = y;
  order.xy_order.d_stop = NAN;

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
  order.xy_order.d_stop = NAN;

  //add order on the circular buffer
  cocobot_add_new_order(&order);

  //return the handle
  return order.handle;
}

cocobot_trajectory_handle_t cocobot_trajectory_goto_xy_circle(float xc, float yc, float xe, float ye, float time)
{
  //fill cocobot_trajectory_order_t struct
  cocobot_trajectory_order_t order;
  order.handle = COCOBOT_TRAJECTORY_INVALID_HANDLE;
  order.type = COCOBOT_TRAJECTORY_GOTO_XY_CIRCLE;
  order.time = time;
  order.circle_order.xc = xc;
  order.circle_order.yc = yc;
  order.circle_order.xe = xe;
  order.circle_order.ye = ye;

  //add order on the circular buffer
  cocobot_add_new_order(&order);

  //return the handle
  return order.handle;
}

cocobot_trajectory_handle_t cocobot_trajectory_goto_xy_circle_backward(float xc, float yc, float xe, float ye, float time)
{
  //fill cocobot_trajectory_order_t struct
  cocobot_trajectory_order_t order;
  order.handle = COCOBOT_TRAJECTORY_INVALID_HANDLE;
  order.type = COCOBOT_TRAJECTORY_GOTO_XY_CIRCLE_BACKWARD;
  order.time = time;
  order.circle_order.xc = xc;
  order.circle_order.yc = yc;
  order.circle_order.xe = xe;
  order.circle_order.ye = ye;

  //add order on the circular buffer
  cocobot_add_new_order(&order);

  //return the handle
  return order.handle;
}

cocobot_trajectory_result_t cocobot_trajectory_wait(void)
{
  while(result == COCOBOT_TRAJECTORY_RUNNING)
  {
    xEventGroupWaitBits(no_more_orders, BIT_0, pdFALSE, pdFALSE, 100 / portTICK_PERIOD_MS); 
  }

  return result;
}

void cocobot_trajetory_set_xy_default(cocobot_trajectory_xy_default_t pref)
{
  xy_pref = pref;
}

int cocobot_trajectory_handle_console(char * command)
{
  if(strcmp(command,"trajectory_list") == 0)
  {
    xSemaphoreTake(mutex, portMAX_DELAY);
    int opost = order_list_read;
    xSemaphoreGive(mutex);
    while(1)
    {
      cocobot_trajectory_order_t * order = NULL;

      xSemaphoreTake(mutex, portMAX_DELAY);
      if(opost != (int)order_list_write)
      {
        order = &order_list[opost];
      }
      xSemaphoreGive(mutex);

      if(order == NULL)
      {
        break;
      }

      int flags = 0;

      if(order->estimated_distance_before_stop > 0)
      {
        flags |= TRAJECTORY_FLAG_SLOW_DOWN;
      }

      switch(order->type)
      {
        case COCOBOT_TRAJECTORY_GOTO_D:
          cocobot_console_send_answer("D,%d,%.3f,%.3f", flags, (double)order->time, (double)order->d_order.distance);
          break;

        case COCOBOT_TRAJECTORY_GOTO_A:
          cocobot_console_send_answer("A,%d,%.3f,%.3f", flags, (double)order->time, (double)order->a_order.angle);
          break;

        case COCOBOT_TRAJECTORY_GOTO_XY:
          cocobot_console_send_answer("XY,%d,%.3f,%.3f,%.3f", flags, (double)order->time, (double)order->xy_order.x, (double)order->xy_order.y);
          break;

        case COCOBOT_TRAJECTORY_GOTO_XY_BACKWARD:
          cocobot_console_send_answer("XY BACKWARD,%d,%.3f,%.3f,%.3f", flags, (double)order->time, (double)order->xy_order.x, (double)order->xy_order.y);
          break;



        default:
          cocobot_console_send_answer("?");
          break;
      }
      cocobot_console_send_answer("%.3f,%.3f,%.3f", (double)order->estimated_start.x, (double)order->estimated_start.y, (double)order->estimated_start.angle);
      cocobot_console_send_answer("%.3f,%.3f,%.3f", (double)order->estimated_end.x, (double)order->estimated_end.y, (double)order->estimated_end.angle);

      opost = (opost + 1) % TRAJECTORY_MAX_ORDER;
    }
    return 1;
  }

  return 0;
}

void cocobot_trajectory_set_opponent_detection(int enable)
{
  enable_opponent_detection = enable;
}
