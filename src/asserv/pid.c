#include <math.h>
#include "cocobot/asserv_pid.h"

void cocobot_asserv_pid_init(cocobot_asserv_pid_t * pid)
{
  pid->kp = 0;
  pid->kd = 0;
  pid->ki = 0;
  pid->max_integral = 0;
  pid->max_error_for_integration = 0;

  pid->input = 0;
  pid->feedback = 0;
  pid->output = 0;

  pid->last_error = 0;
  pid->integral = 0;

  pid->p_contrib = 0;
  pid->d_contrib = 0;
  pid->i_contrib = 0;
}

void cocobot_asserv_pid_set_kp(cocobot_asserv_pid_t * pid, float kp)
{
  pid->kp = kp;
}

void cocobot_asserv_pid_set_kd(cocobot_asserv_pid_t * pid, float kd)
{
  pid->kd = kd;
}

void cocobot_asserv_pid_set_ki(cocobot_asserv_pid_t * pid, float ki)
{
  pid->ki = ki;
}

void cocobot_asserv_pid_set_max_integral(cocobot_asserv_pid_t * pid, float max_integral)
{
  pid->max_integral = max_integral;
}

void cocobot_asserv_pid_set_max_error_for_integration(cocobot_asserv_pid_t * pid, float max_error_for_integration)
{
  pid->max_error_for_integration = max_error_for_integration;
}

float cocobot_asserv_pid_get_kp(cocobot_asserv_pid_t * pid)
{
  return pid->kp;
}

float cocobot_asserv_pid_get_kd(cocobot_asserv_pid_t * pid)
{
  return pid->kd;
}

float cocobot_asserv_pid_get_ki(cocobot_asserv_pid_t * pid)
{
  return pid->ki;
}

float cocobot_asserv_pid_get_max_integral(cocobot_asserv_pid_t * pid)
{
  return pid->max_integral;
}

float cocobot_asserv_pid_get_max_error_for_integration(cocobot_asserv_pid_t * pid)
{
  return pid->max_error_for_integration;
}

void cocobot_asserv_pid_reset(cocobot_asserv_pid_t * pid)
{
  pid->integral = 0;
  pid->last_error = 0;
  pid->output = 0;
}

void cocobot_asserv_pid_compute(cocobot_asserv_pid_t * pid)
{
  //compute error
  float error = pid->input - pid->feedback;

  //compute proportional
  float p = pid->kp * error;

  //compute derivate
  float d = (error - pid->last_error) * pid->kd;
  pid->last_error = error;

  //compute integrate
  if(fabsf(error) > pid->max_error_for_integration)
  {
    //error too big. Reset integral to prevent windup
    pid->integral = 0;
  }
  else
  {
    pid->integral += error;
  }
  //saturate integral
  if(pid->integral > pid->max_integral)
  {
    pid->integral = pid->max_integral;
  }
  if(pid->integral < -pid->max_integral)
  {
    pid->integral = -pid->max_integral;
  }
  float i = pid->integral * pid->ki;

  pid->p_contrib = p;
  pid->d_contrib = d;
  pid->i_contrib = i;

  pid->output = p + d + i;
}

void cocobot_asserv_pid_set_input(cocobot_asserv_pid_t * pid, float input)
{
  pid->input = input;
}

void cocobot_asserv_pid_set_feedback(cocobot_asserv_pid_t * pid, float feedback)
{
  pid->feedback = feedback;
}

float cocobot_asserv_pid_get_output(cocobot_asserv_pid_t * pid)
{
  return pid->output;
}

float cocobot_asserv_pid_get_p_contribution(cocobot_asserv_pid_t * pid)
{
  return pid->p_contrib;
}

float cocobot_asserv_pid_get_i_contribution(cocobot_asserv_pid_t * pid)
{
  return pid->i_contrib;
}

float cocobot_asserv_pid_get_d_contribution(cocobot_asserv_pid_t * pid)
{
  return pid->d_contrib;
}

