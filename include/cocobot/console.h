#ifndef COCOBOT_CONSOLE_H
#define COCOBOT_CONSOLE_H

#include <mcual.h>

void cocobot_console_init(mcual_usart_id_t usart_id, unsigned int priority);
void cocobot_console_send_asynchronous(char * title, char * data);

#endif// COCOBOT_CONSOLE_H
