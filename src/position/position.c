#include <cocobot.h>



void cocobot_position_init(unsigned int priority);
{
  xTaskCreate(cocobot_position_thread, (const signed char *)"position", 200, NULL, priority, NULL );
}
