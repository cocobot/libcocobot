#ifndef COCOBOT_CONSOLE_H
#define COCOBOT_CONSOLE_H

#include <mcual.h>

typedef int (*cocobot_console_handler_t)(const char *);

void cocobot_console_init(mcual_usart_id_t usart_id, unsigned int priority, cocobot_console_handler_t handler);
void cocobot_console_send_asynchronous(char * title, char * data);
int cocobot_console_get_iargument(int id);

#endif// COCOBOT_CONSOLE_H
