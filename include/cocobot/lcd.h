#ifndef COCOBOT_LCD_H
#define COCOBOT_LCD_H

#include <stdint.h>

#define COCOBOT_LCD_X_MAX 84
#define COCOBOT_LCD_Y_MAX 48

/* Initialization of the lcd module. Need to be called before any other action */
void cocobot_lcd_init(void);

/* Clear framebuffer (set all pixel to white) */
void cocobot_lcd_clear(void);

/* Send framebuffer to lcd */
void cocobot_lcd_render(void);

/* Draw a line from (x0; y0) to (x1; y1) */
void cocobot_lcd_draw_line(int x0, int y0, int x1, int y1);

/* Print a string on lcd. Start on coord (xpos; ypos)*/
void cocobot_lcd_print(int xpos, int ypos, const char * fmt, ...)  __attribute__ ((format (printf, 3, 4)));

#endif//COCOBOT_LCD_H
