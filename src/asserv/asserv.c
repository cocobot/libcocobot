#include <cocobot.h>
#include "cocobot/asserv_ramp.h"
#include "cocobot/asserv_pid.h"
#include "generated/autoconf.h"

static cocobot_asserv_ramp_t _ramp_dist;
static cocobot_asserv_ramp_t _ramp_angu;
static cocobot_asserv_pid_t _pid_dist;
static cocobot_asserv_pid_t _pid_angu;

static cocobot_asserv_state_t _state = COCOBOT_ASSERV_DISABLE;


void cocobot_asserv_init(void)
{
  cocobot_asserv_set_state(COCOBOT_ASSERV_DISABLE);

  //init ramps
  cocobot_asserv_ramp_init(&_ramp_dist);
  cocobot_asserv_ramp_init(&_ramp_angu);
  cocobot_asserv_ramp_reset(&_ramp_dist, cocobot_position_get_distance());
  cocobot_asserv_ramp_reset(&_ramp_angu, cocobot_position_get_angle());

  //init pids
  cocobot_asserv_pid_init(&_pid_dist);
  cocobot_asserv_pid_init(&_pid_angu);

  //set coefs
  cocobot_asserv_ramp_set_max_speed(&_ramp_dist, CONFIG_LIBCOCOBOT_DIST_RAMP_MAX_SPEED / 1000.0f);
  cocobot_asserv_ramp_set_max_speed(&_ramp_angu, CONFIG_LIBCOCOBOT_ANGU_RAMP_MAX_SPEED / 1000.0f);
  cocobot_asserv_ramp_set_max_accel(&_ramp_dist, CONFIG_LIBCOCOBOT_DIST_RAMP_MAX_ACCEL / 1000.0f);
  cocobot_asserv_ramp_set_max_accel(&_ramp_angu, CONFIG_LIBCOCOBOT_ANGU_RAMP_MAX_ACCEL / 1000.0f);
  cocobot_asserv_pid_set_kp(&_pid_dist, CONFIG_LIBCOCOBOT_DIST_PID_KP / 1000.0f);
  cocobot_asserv_pid_set_kp(&_pid_angu, CONFIG_LIBCOCOBOT_ANGU_PID_KP / 1000.0f);
  cocobot_asserv_pid_set_kd(&_pid_dist, CONFIG_LIBCOCOBOT_DIST_PID_KD / 1000.0f);
  cocobot_asserv_pid_set_kd(&_pid_angu, CONFIG_LIBCOCOBOT_ANGU_PID_KD / 1000.0f);
  cocobot_asserv_pid_set_ki(&_pid_dist, CONFIG_LIBCOCOBOT_DIST_PID_KI / 1000.0f);
  cocobot_asserv_pid_set_ki(&_pid_angu, CONFIG_LIBCOCOBOT_ANGU_PID_KI / 1000.0f);
  cocobot_asserv_pid_set_max_integral(&_pid_dist, CONFIG_LIBCOCOBOT_DIST_PID_MAX_INTEGRAL / 1000.0f);
  cocobot_asserv_pid_set_max_integral(&_pid_angu, CONFIG_LIBCOCOBOT_ANGU_PID_MAX_INTEGRAL / 1000.0f);
  cocobot_asserv_pid_set_max_error_for_integration(&_pid_dist, CONFIG_LIBCOCOBOT_DIST_PID_MAX_ERROR_FOR_INTEGRATION / 1000.0f);
  cocobot_asserv_pid_set_max_error_for_integration(&_pid_angu, CONFIG_LIBCOCOBOT_ANGU_PID_MAX_ERROR_FOR_INTEGRATION / 1000.0f);
}

void cocobot_asserv_compute(void)
{
 if(_state == COCOBOT_ASSERV_ENABLE)
 {
   //compute ramps
   cocobot_asserv_ramp_compute(&_ramp_dist); 
   cocobot_asserv_ramp_compute(&_ramp_angu); 

   //set PID inputs
   cocobot_asserv_pid_set_input(&_pid_dist, cocobot_asserv_ramp_get_output(&_ramp_dist));
   cocobot_asserv_pid_set_input(&_pid_angu, cocobot_asserv_ramp_get_output(&_ramp_angu));
   cocobot_asserv_pid_set_feedback(&_pid_dist, cocobot_position_get_distance());
   cocobot_asserv_pid_set_feedback(&_pid_angu, cocobot_position_get_angle());

   //compute PIDs
   cocobot_asserv_pid_compute(&_pid_dist);
   cocobot_asserv_pid_compute(&_pid_angu);
 }
 else
 {
   cocobot_asserv_ramp_reset(&_ramp_dist, cocobot_position_get_distance());
   cocobot_asserv_ramp_reset(&_ramp_angu, cocobot_position_get_angle());

   cocobot_asserv_pid_reset(&_pid_dist);
   cocobot_asserv_pid_reset(&_pid_angu);
 }
}

void cocobot_asserv_set_distance_set_point(float distance)
{
  cocobot_asserv_ramp_set_position_target(&_ramp_dist, distance);
}

void cocobot_asserv_set_angular_set_point(float angular)
{
  cocobot_asserv_ramp_set_position_target(&_ramp_angu, angular);
}

void cocobot_asserv_set_state(cocobot_asserv_state_t state)
{
  _state = state;
}
