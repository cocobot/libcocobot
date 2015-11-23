#include <cocobot.h>
#include "cocobot/asserv_ramp.h"

static cocobot_asserv_ramp_t ramp_dist;
static cocobot_asserv_ramp_t ramp_angu;

void cocobot_asserv_init(void)
{
  //init ramps
  cocobot_asserv_ramp_init(&ramp_dist);
  cocobot_asserv_ramp_init(&ramp_angu);
  cocobot_asserv_ramp_reset(&ramp_dist, cocobot_position_get_distance());
  cocobot_asserv_ramp_reset(&ramp_angu, cocobot_position_get_angle());
}

void cocobot_asserv_set_distance_set_point(float distance)
{
  (void)distance;
}
