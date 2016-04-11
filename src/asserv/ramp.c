#include <cocobot.h>
#include "cocobot/asserv_ramp.h"

void cocobot_asserv_ramp_init(cocobot_asserv_ramp_t * ramp)
{
  ramp->max_speed = 0;
  ramp->max_accel = 0;

  ramp->position_target = 0;
  ramp->position_current = 0;
  ramp->speed_target = 0;
}
void cocobot_asserv_ramp_set_feedback(cocobot_asserv_ramp_t * ramp, float feedback)
{
  ramp->position_feedback = feedback;
}

void cocobot_asserv_ramp_set_max_speed(cocobot_asserv_ramp_t * ramp, float max_speed)
{
  ramp->max_speed = max_speed;
}

float cocobot_asserv_ramp_get_max_speed(cocobot_asserv_ramp_t * ramp)
{
  return ramp->max_speed;
}

void cocobot_asserv_ramp_set_max_accel(cocobot_asserv_ramp_t * ramp, float max_accel)
{
  ramp->max_accel = max_accel;
}

float cocobot_asserv_ramp_get_max_accel(cocobot_asserv_ramp_t * ramp)
{
  return ramp->max_accel;
}

void cocobot_asserv_ramp_compute(cocobot_asserv_ramp_t * ramp)
{
  //compute how much distance the robot will do if we start decreasing the speed now
  float delta_position = ramp->speed_target * ramp->speed_target / (2 * ramp->max_accel);

  //check if we want to move forward or backward
  float err = ramp->position_target - ramp->position_current;
  if(err > 0)
  {
    //forward

    //check if we need to speed up or down
    if(delta_position + ramp->position_current < ramp->position_target)
    {
      //we can increase the speed
      ramp->speed_target += ramp->max_accel;
    }
    else
    {
      //we should decrease the speed
      ramp->speed_target -= ramp->max_accel;
    }
  }
  else if(err < 0)
  {
    //backward

    //check if we need to speed up or down
    if(-delta_position + ramp->position_current < ramp->position_target)
    {
      //we should decrease the (negative) speed
      ramp->speed_target += ramp->max_accel;
    }
    else
    {
      //we can increase the (negative) speed
      ramp->speed_target -= ramp->max_accel;
    }
  }

  //set speed limit (because of cops !)
  if(ramp->speed_target > ramp->max_speed)
  {
    ramp->speed_target = ramp->max_speed;
  }
  if(ramp->speed_target < -ramp->max_speed)
  {
    ramp->speed_target = -ramp->max_speed;
  }

  if((err > 0) && (ramp->speed_target > 0))
  {
    if(ramp->position_current < ramp->position_feedback)
    {
      ramp->position_current = ramp->position_feedback;
    }
  }
  if((err < 0) && (ramp->speed_target < 0))
  {
    if(ramp->position_current > ramp->position_feedback)
    {
      ramp->position_current = ramp->position_feedback;
    }
  }

  //compute next output
  float output = ramp->position_current + ramp->speed_target;

  //prevent position overshoot because of discrete speed
  if((err > 0) && (ramp->speed_target > 0))
  {
    if(output > ramp->position_target)
    {
      output = ramp->position_target;
      ramp->speed_target = 0;
    }
  }
  else if((err < 0) && (ramp->speed_target < 0))
  {
    if(output < ramp->position_target)
    {
      output = ramp->position_target;
      ramp->speed_target = 0;
    }
  }

  //assign output
  ramp->position_current = output;
}

void cocobot_asserv_ramp_reset(cocobot_asserv_ramp_t * ramp, float current_position)
{
  ramp->position_target = current_position;
  ramp->position_current = current_position;
  ramp->speed_target = 0;
}

void cocobot_asserv_ramp_set_position_target(cocobot_asserv_ramp_t * ramp, float target)
{
  ramp->position_target = target;
}

float cocobot_asserv_ramp_get_position_target(cocobot_asserv_ramp_t * ramp)
{
  return ramp->position_target;
}

float cocobot_asserv_ramp_get_speed_target(cocobot_asserv_ramp_t * ramp)
{
  return ramp->speed_target;
}

float cocobot_asserv_ramp_get_output(cocobot_asserv_ramp_t * ramp)
{
  return ramp->position_current;
}
