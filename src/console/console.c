#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <cocobot.h>
#include <FreeRTOS.h>
#include <task.h>

//define protocol special characters
#define COCOBOT_CONSOLE_ASYNCHRONOUS_START  "#"
#define COCOBOT_CONSOLE_SYNCHRONOUS_START   "< "
#define COCOBOT_CONSOLE_COMMAND_SEPARATOR   "="
#define COCOBOT_CONSOLE_END_LINE            "\n"
#define COCOBOT_CONSOLE_USER_INPUT_START    "> "
#define COCOBOT_CONSOLE_BUFFER_LENGTH       255

//useful macro for handling commmand
#define TRY_HANDLER_IF_NEEDED(handled, cmd, f) do { handled = handled ? handled: f(cmd);} while(0)

//internal parameters
static mcual_usart_id_t _usart;
static char _sync_buffer[COCOBOT_CONSOLE_BUFFER_LENGTH];
static char _async_buffer[COCOBOT_CONSOLE_BUFFER_LENGTH];
static unsigned int _buffer_position;
static cocobot_console_handler_t _user_handler;
static char * _arguments;

static void cocobot_console_send_string(char * str)
{
  // /!\ str must be a valid NULL terminated string (otherwise -> infinite loop !)
  
  while(*str != 0)
  {
    mcual_usart_send(_usart, *str);
    str += 1;
  }
}

void cocobot_console_send_asynchronous(char * title, char * fmt, ...)
{
  //prevent other to use uart peripheral
  xSemaphoreTake(_mutex, portMAX_DELAY);

  //format output using vsnprintf. Be careful if using float, it may alloc some memory
  va_list args;
  va_start (args, fmt);
  vsnprintf(_async_buffer, sizeof(_async_buffer), fmt, args); 
  va_end (args);

  //send the output to the serial line
  cocobot_console_send_string(COCOBOT_CONSOLE_ASYNCHRONOUS_START);
  cocobot_console_send_string(title);
  cocobot_console_send_string(COCOBOT_CONSOLE_COMMAND_SEPARATOR);
  cocobot_console_send_string(async_buffer);
  cocobot_console_send_string(COCOBOT_CONSOLE_END_LINE);

  //release the lock
  xSemaphoreGive(mutex);
}

void cocobot_console_send_answer(char * fmt, ...)
{
  //prevent other to use uart peripheral
  xSemaphoreTake(_mutex, portMAX_DELAY);

  //format output using vsnprintf. Be careful if using float, it may alloc some memory
  va_list args;
  va_start (args, fmt);
  vsnprintf(sync_buffer, sizeof(sync_buffer), fmt, args); 
  va_end (args);

  //send the output to the serial line
  cocobot_console_send_string(sync_buffer);
  cocobot_console_send_string(COCOBOT_CONSOLE_END_LINE);

  //release the lock
  xSemaphoreGive(mutex);
}

int cocobot_console_handle_freertos(char * command)
{
  //list freertos task
  if(strcmp(command,"freertos") == 0)
  {
    TaskStatus_t tasks[10];
    int tasks_num = uxTaskGetSystemState(tasks, 10, NULL);
    int i;
    for(i = 0; i < tasks_num; i += 1)
    {
      cocobot_console_send_answer("%s,%lu,%lu,%lu,%d", 
               tasks[i].pcTaskName,
               (long unsigned int)tasks[i].uxCurrentPriority,
               (long unsigned int)tasks[i].uxBasePriority,
               tasks[i].ulRunTimeCounter,
               tasks[i].usStackHighWaterMark
               );
    }
    return 1;
  }

  return 0;
}


int cocobot_console_get_fargument(int id, float * out)
{
  char * ptr = arguments;
  while(*ptr)
  {
    if(id == 0)
    {
      //use strtof to convert to float without modifying the buffer
      *out = strtof(ptr, NULL);
      return 1;
    }
    if(*ptr == ' ')
    {
      id -= 1;
    }
    ptr += 1;
  }

  return 0;
}

int cocobot_console_get_iargument(int id, int * i)
{
  float f;
  //read float then convert it to integer (TODO: parse directly in integer)
  int r = cocobot_console_get_fargument(id, &f);
  *i = (int)f;
  return r;
}

void cocobot_console_async_thread(void *arg)
{
  (void)arg;

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(pdTRUE)
  {
    //send debug information if needed
    cocobot_asserv_handle_async_console();

    //wait 100ms (minus time used by previous handler)
    vTaskDelayUntil( &xLastWakeTime, 100 / portTICK_PERIOD_MS);
  }
}

void cocobot_console_sync_thread(void *arg)
{
  (void)arg;
  
  //fresh console start
  cocobot_console_send_string(COCOBOT_CONSOLE_END_LINE);
  cocobot_console_send_string(COCOBOT_CONSOLE_USER_INPUT_START);

  while(pdTRUE)
  {
    uint8_t recv = mcual_usart_recv(usart);
    if(recv != '\r') //discard \r character
    {      
      if(recv == '\n') //new command
      {
        sync_buffer[buffer_position] = 0;

        //find arguments
        char * cmd = sync_buffer;
        arguments = sync_buffer;
        while(*arguments)
        {
          if(*arguments == ' ')
          {
            *arguments = 0;
            arguments += 1;
            break;
          }
          arguments += 1;
        }

        //try to parse the command with builtin command
        int handled = 0;
        TRY_HANDLER_IF_NEEDED(handled, cmd, cocobot_console_handle_freertos);
        TRY_HANDLER_IF_NEEDED(handled, cmd, cocobot_asserv_handle_console);

        //try to parse the command with user defined callback
        if(user_handler != NULL)
        {
          TRY_HANDLER_IF_NEEDED(handled, cmd, user_handler);
        }

        if(!handled)
        {
          handled = user_handler(cmd);
        }

        if(!handled)
        {
          //send error message (command not found)
          xSemaphoreTake(mutex, portMAX_DELAY);
          cocobot_console_send_string("invalid command: '");
          cocobot_console_send_string(cmd);
          cocobot_console_send_string("'" COCOBOT_CONSOLE_END_LINE);
          xSemaphoreGive(mutex);
        }
        buffer_position = 0;

        xSemaphoreTake(mutex, portMAX_DELAY);
        cocobot_console_send_string(COCOBOT_CONSOLE_USER_INPUT_START);
        xSemaphoreGive(mutex);
      }
      else
      {
        if(buffer_position < COCOBOT_CONSOLE_BUFFER_LENGTH - 1)
        {
          sync_buffer[buffer_position] = recv;
          buffer_position += 1;
        }
      }
    }
  }
}

void cocobot_console_init(mcual_usart_id_t usart_id, unsigned int priority_monitor, unsigned int priority_async, cocobot_console_handler_t handler)
{
  //internal storage initialization
  usart = usart_id;
  buffer_position = 0;
  user_handler = handler;

  //create mutex
  mutex = xSemaphoreCreateMutex();

  //init usart peripheral
  mcual_usart_init(usart, 115200);

  //start tasks
  xTaskCreate(cocobot_console_sync_thread, "con. sync", 512, NULL, priority_monitor, NULL );
  xTaskCreate(cocobot_console_async_thread, "con. async", 512, NULL, priority_async, NULL );
}
