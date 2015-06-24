#ifndef COCOBOT_POSITION_H
#define COCOBOT_POSITION_H

void cocobot_position_init(unsigned int priority);

float cocobot_position_get_Xmm(void);
float cocobot_position_get_Ymm(void);
float cocobot_position_get_Dmm(void);
float cocobot_position_get_Adeg(void);

void cocobot_position_set_Xmm(float new_x);
void cocobot_position_set_Ymm(float new_y);
void cocobot_position_set_Adeg(float new_a);

#endif// COCOBOT_POSITION_H
