#ifndef COCOBOT_GAME_STATE_H
#define COCOBOT_GAME_STATE_H

typedef enum
{
  COCOBOT_GAME_STATE_COLOR_NEG,
  COCOBOT_GAME_STATE_COLOR_POS,
} cocobot_game_state_color_t;

typedef void (*cocobot_game_state_funny_action_t)(void);

/* Initialization of the game state module. Need to be called before any other action 
 * Argument:
 *  - funny_action: task to execute when the game is ended
 */
void cocobot_game_state_init(cocobot_game_state_funny_action_t funny_action);

/* Wait until the starter is removed (begining of the 90s match)
 * The function will enable the motor power
 */
void cocobot_game_state_wait_for_starter_removed(void);

/*
 * Get the color.
 * Return:
 *  - cocobot_game_state_color_t. if COCOBOT_GAME_STATE_COLOR_NEG if the robot starts with negative x, and COCOBOT_GAME_STATE_COLOR_POS if the robot starts with positive x.
 */
cocobot_game_state_color_t cocobot_game_state_get_color(void);

#endif// COCOBOT_GAME_STATE_H
