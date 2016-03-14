#ifndef COCOBOT_CONSOLE_H
#define COCOBOT_CONSOLE_H

#include <mcual.h>
#include <string.h>

//useful macro for handling commmand
#define COCOBOT_CONSOLE_TRY_HANDLER_IF_NEEDED(handled, cmd, f) do { handled = handled ? handled: f(cmd);} while(0)

typedef int (*cocobot_console_handler_t)(const char *);

/* Initialization of the console module. Need to be called before any other action 
 * Argument:
 *  - usart_id: usart id (from mcual) for the console
 *  - priority_monitor: priority of the console monitor task. This task handle user request
 *  - priority_async: priority of the task sending asynchronous debug messages
 *  - handler: function pointer for handling user defined commands
 */
void cocobot_console_init(mcual_usart_id_t usart_id, unsigned int priority_monitor, unsigned int priority_async, cocobot_console_handler_t handler);

/* Try to read a string argument
 *  - id: position of the argument (0 means first argument)
 *  - str: reference to a valid storage for the string
 *  - maxsize: size of the string variable
 * Return:
 *  1 : if the argument exists
 *  0 : if the argument does not exist
 */
int cocobot_console_get_sargument(int id, char * str, int maxsize);

/* Try to read a signed integer argument
 *  - id: position of the argument (0 means first argument)
 *  - i: reference to a valid storage for the integer
 * Return:
 *  1 : if the argument exists
 *  0 : if the argument does not exist
 */
int cocobot_console_get_iargument(int id, int * i);

/* Try to read a float argument
 *  - id: position of the argument (0 means first argument)
 *  - f: reference to a valid storage for the float
 * Return:
 *  1 : if the argument exists
 *  0 : if the argument does not exist
 */
int cocobot_console_get_fargument(int id, float * f);

/* Send an answer to a command
 * Argument:
 *  - fmt: format string (see printf documentation)
 *  - ...: argument list for the format string
 */
void cocobot_console_send_answer(char * fmt, ...)  __attribute__ ((format (printf, 1, 2)));

/* Send asynchronous debug information
 * Argument:
 *  - time : namespace of the information
 *  - fmt: format string (see printf documentation)
 *  - ...: argument list for the format string
 */
void cocobot_console_send_asynchronous(char * title, char * fmt, ...)  __attribute__ ((format (printf, 2, 3)));

#endif// COCOBOT_CONSOLE_H
