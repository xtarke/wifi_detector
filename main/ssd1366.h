/*
* ssd1366.c
*
*  Created on: Mar 2, 2020
*      Author: Renan Augusto Starke
*      Instituto Federal de Santa Catarina *
*
*
*      Definitions from: https://github.com/yanbe/ssd1306-esp-idf-i2c
*
*/

#ifndef SSD1366_H_
#define SSD1366_H_

#include <stdint.h>

typedef enum  {
  WHITE_PIXEL, BLACK_PIXEL
} pixel_color_t;

void ssd1306_init_new();

void ssd1306_display_data();
void ssd1306_display_clear();

void ssd1306_clear_vertical_region(uint8_t x, uint8_t hor_size);
void ssd1306_fill_vertical_region(uint8_t x, uint8_t hor_size);

void ssd1306_clearDisplay_buffer(void);

void ssd1306_draw_h_line(int16_t x, int16_t y, int16_t size, pixel_color_t color);
void ssd1306_draw_pixel(int16_t x, int16_t y, pixel_color_t color);

void ssd306_write_string(int16_t x, int16_t y, char *data);
void ssd1306_write_char(int16_t x, int16_t y, char data);


#endif /* SSD1366_H_ */
