#ifndef OPPONENT_DETECTION_H
#define OPPONENT_DETECTION_H

#define COCOBOT_OPPONENT_DETECTION_DEACTIVATED  0
#define COCOBOT_OPPONENT_DETECTION_ACTIVATED    1

#define COCOBOT_OPPONENT_DETECTION_USIR_FRONT_LEFT 0
#define COCOBOT_OPPONENT_DETECTION_USIR_FRONT_RIGHT 1
#define COCOBOT_OPPONENT_DETECTION_USIR_BACK_LEFT 2
#define COCOBOT_OPPONENT_DETECTION_USIR_BACK_RIGHT 3

#define COCOBOT_OPPONENT_DETECTION_MAX_X (1500 - 100)
#define COCOBOT_OPPONENT_DETECTION_MAX_Y (1000 - 100)


/* Initialization of the opponent detection module. Need to be called before any other action 
 * Argument:
 *  - task_priority: FreeRTOS task priority for detection thread
 */
void cocobot_opponent_detection_init(unsigned int task_priority);

/* Handle console user command related to the opponent dectection module
 * Argument:
 *  - command: requested command
 * Return:
 *  0 : if command is not reconized
 *  1 : if command has been successfully handled
 */
int cocobot_opponent_detection_handle_console(char * command);

void cocobot_opponent_detection_set_enable(int id, int status);

int cocobot_opponent_detection_is_in_alert(void);

#define COCOBOT_OPPONENT_DETECTION_ENABLE_FRONT() do \
                {\
                  cocobot_opponent_detection_set_enable(COCOBOT_OPPONENT_DETECTION_USIR_FRONT_LEFT, COCOBOT_OPPONENT_DETECTION_ACTIVATED);\
                  cocobot_opponent_detection_set_enable(COCOBOT_OPPONENT_DETECTION_USIR_FRONT_RIGHT, COCOBOT_OPPONENT_DETECTION_ACTIVATED);\
                  cocobot_opponent_detection_set_enable(COCOBOT_OPPONENT_DETECTION_USIR_BACK_LEFT, COCOBOT_OPPONENT_DETECTION_DEACTIVATED);\
                  cocobot_opponent_detection_set_enable(COCOBOT_OPPONENT_DETECTION_USIR_BACK_RIGHT, COCOBOT_OPPONENT_DETECTION_DEACTIVATED);\
                } while(0)

#define COCOBOT_OPPONENT_DETECTION_ENABLE_BACK() do \
                {\
                  cocobot_opponent_detection_set_enable(COCOBOT_OPPONENT_DETECTION_USIR_FRONT_LEFT, COCOBOT_OPPONENT_DETECTION_DEACTIVATED);\
                  cocobot_opponent_detection_set_enable(COCOBOT_OPPONENT_DETECTION_USIR_FRONT_RIGHT, COCOBOT_OPPONENT_DETECTION_DEACTIVATED);\
                  cocobot_opponent_detection_set_enable(COCOBOT_OPPONENT_DETECTION_USIR_BACK_LEFT, COCOBOT_OPPONENT_DETECTION_ACTIVATED);\
                  cocobot_opponent_detection_set_enable(COCOBOT_OPPONENT_DETECTION_USIR_BACK_RIGHT, COCOBOT_OPPONENT_DETECTION_ACTIVATED);\
                } while(0)

#define COCOBOT_OPPONENT_DETECTION_DISABLE() do \
                {\
                  cocobot_opponent_detection_set_enable(COCOBOT_OPPONENT_DETECTION_USIR_FRONT_LEFT, COCOBOT_OPPONENT_DETECTION_DEACTIVATED);\
                  cocobot_opponent_detection_set_enable(COCOBOT_OPPONENT_DETECTION_USIR_FRONT_RIGHT, COCOBOT_OPPONENT_DETECTION_DEACTIVATED);\
                  cocobot_opponent_detection_set_enable(COCOBOT_OPPONENT_DETECTION_USIR_BACK_LEFT, COCOBOT_OPPONENT_DETECTION_DEACTIVATED);\
                  cocobot_opponent_detection_set_enable(COCOBOT_OPPONENT_DETECTION_USIR_BACK_RIGHT, COCOBOT_OPPONENT_DETECTION_DEACTIVATED);\
                } while(0)

#endif// OPPONENT_DETECTION_H
