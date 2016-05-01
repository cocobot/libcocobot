#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <cocobot.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>



//define protocol special characters
#define COCOBOT_CONSOLE_ASYNCHRONOUS_START  "#"
#define COCOBOT_CONSOLE_SYNCHRONOUS_START   "< "
#define COCOBOT_CONSOLE_COMMAND_SEPARATOR   "="
#define COCOBOT_CONSOLE_END_LINE            "\n"
#define COCOBOT_CONSOLE_USER_INPUT_START    ">"
#define COCOBOT_CONSOLE_BUFFER_LENGTH       255


//internal parameters
static mcual_usart_id_t _usart;
static char _sync_buffer[COCOBOT_CONSOLE_BUFFER_LENGTH];
static char _async_buffer[COCOBOT_CONSOLE_BUFFER_LENGTH];
static unsigned int _buffer_position;
static cocobot_console_handler_t _user_handler;
static char * _arguments;
static SemaphoreHandle_t _mutex;

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
  //format output using vsnprintf. Be careful if using float, it may alloc some memory
  va_list args;
  va_start (args, fmt);
  vsnprintf(_async_buffer, sizeof(_async_buffer), fmt, args);
  va_end (args);

  //prevent other to use uart peripheral
  xSemaphoreTake(_mutex, portMAX_DELAY);

  //send the output to the serial line
  cocobot_console_send_string(COCOBOT_CONSOLE_ASYNCHRONOUS_START);
  cocobot_console_send_string(title);
  cocobot_console_send_string(COCOBOT_CONSOLE_COMMAND_SEPARATOR);
  cocobot_console_send_string(_async_buffer);
  cocobot_console_send_string(COCOBOT_CONSOLE_END_LINE);

  //release the lock
  xSemaphoreGive(_mutex);
}

void cocobot_console_send_answer(char * fmt, ...)
{
  //format output using vsnprintf. Be careful if using float, it may alloc some memory
  va_list args;
  va_start (args, fmt);
  vsnprintf(_sync_buffer, sizeof(_sync_buffer), fmt, args);
  va_end (args);

  //prevent other to use uart peripheral
  xSemaphoreTake(_mutex, portMAX_DELAY);

  //send the output to the serial line
  cocobot_console_send_string(_sync_buffer);
  cocobot_console_send_string(COCOBOT_CONSOLE_END_LINE);

  //release the lock
  xSemaphoreGive(_mutex);
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

int cocobot_console_handle_system(char * command)
{
  //list freertos task
  if(strcmp(command,"system_reboot") == 0)
  {
    mcual_bootloader();
    return 1;
  }

  return 0;
}


int cocobot_console_get_sargument(int id, char * str, int maxsize)
{
  char * ptr = _arguments;
  while(*ptr)
  {
    if(id == 0)
    {
      memset(str, 0, maxsize);
      for(int i = 0; i < maxsize - 1; i += 1, str += 1, ptr += 1)
      {
        if((*ptr == 0) || (*ptr == ' '))
        {
          break;
        }
        *str = *ptr;
      }
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

int cocobot_console_get_fargument(int id, float * out)
{
  char * ptr = _arguments;
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
  if(r) {
    *i = (int)f;
  }
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
    cocobot_position_handle_async_console();
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
    uint8_t recv = mcual_usart_recv(_usart);
    if(recv != '\r') //discard \r character
    {
      if(recv == '\n') //new command
      {
        _sync_buffer[_buffer_position] = 0;

        //find arguments
        char * cmd = _sync_buffer;
        _arguments = _sync_buffer;
        while(*_arguments)
        {
          if(*_arguments == ' ')
          {
            *_arguments = 0;
            _arguments += 1;
            break;
          }
          _arguments += 1;
        }

        //try to parse the command with builtin command
        int handled = 0;
        COCOBOT_CONSOLE_TRY_HANDLER_IF_NEEDED(handled, cmd, cocobot_console_handle_system);
        COCOBOT_CONSOLE_TRY_HANDLER_IF_NEEDED(handled, cmd, cocobot_console_handle_freertos);
        COCOBOT_CONSOLE_TRY_HANDLER_IF_NEEDED(handled, cmd, cocobot_position_handle_console);
        COCOBOT_CONSOLE_TRY_HANDLER_IF_NEEDED(handled, cmd, cocobot_trajectory_handle_console);
        COCOBOT_CONSOLE_TRY_HANDLER_IF_NEEDED(handled, cmd, cocobot_asserv_handle_console);
        COCOBOT_CONSOLE_TRY_HANDLER_IF_NEEDED(handled, cmd, cocobot_opponent_detection_handle_console);
        COCOBOT_CONSOLE_TRY_HANDLER_IF_NEEDED(handled, cmd, cocobot_action_scheduler_handle_console);

        //try to parse the command with user defined callback
        if(_user_handler != NULL)
        {
          COCOBOT_CONSOLE_TRY_HANDLER_IF_NEEDED(handled, cmd, _user_handler);
        }

        if(!handled)
        {
          //send error message (command not found)
          xSemaphoreTake(_mutex, portMAX_DELAY);
          cocobot_console_send_string("invalid command: '");
          cocobot_console_send_string(cmd);
          cocobot_console_send_string("'" COCOBOT_CONSOLE_END_LINE);
          xSemaphoreGive(_mutex);
        }
        _buffer_position = 0;

        xSemaphoreTake(_mutex, portMAX_DELAY);
        cocobot_console_send_string(COCOBOT_CONSOLE_USER_INPUT_START);
        xSemaphoreGive(_mutex);
      }
      else
      {
        if(_buffer_position < COCOBOT_CONSOLE_BUFFER_LENGTH - 1)
        {
          _sync_buffer[_buffer_position] = recv;
          _buffer_position += 1;
        }
      }
    }
  }
}

void cocobot_console_init(mcual_usart_id_t usart_id, unsigned int priority_monitor, unsigned int priority_async, cocobot_console_handler_t handler)
{
  //internal storage initialization
  _usart = usart_id;
  _buffer_position = 0;
  _user_handler = handler;

  //create mutex
  _mutex = xSemaphoreCreateMutex();

  //init usart peripheral
  mcual_usart_init(_usart, 115200);

  //start tasks
  xTaskCreate(cocobot_console_sync_thread, "con. sync", 512, NULL, priority_monitor, NULL );
  xTaskCreate(cocobot_console_async_thread, "con. async", 512, NULL, priority_async, NULL );
}
