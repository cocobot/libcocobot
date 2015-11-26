#ifndef COCOBOT_ASSERV_H
#define COCOBOT_ASSERV_H

typedef enum
{
  COCOBOT_ASSERV_DISABLE,
  COCOBOT_ASSERV_ENABLE,
} cocobot_asserv_state_t;

/* Initialization of the asserv module. Need to be called before any other action 
 */
void cocobot_asserv_init(void);

/* Run the asserv computation
 */
void cocobot_asserv_compute(void);


void cocobot_asserv_set_distance_set_point(float distance);
void cocobot_asserv_set_angular_set_point(float angular);

void cocobot_asserv_set_state(cocobot_asserv_state_t state);

#endif// COCOBOT_ASSERV_H
