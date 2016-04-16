#ifndef OPPONENT_DETECTION_H
#define OPPONENT_DETECTION_H

#define COCOBOT_OPPONENT_DETECTION_DEACTIVATED  0
#define COCOBOT_OPPONENT_DETECTION_ACTIVATED    1

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

#endif// OPPONENT_DETECTION_H
