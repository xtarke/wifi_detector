/*
* ssd1366.c
*
*  Created on: Mar 2, 2020
*      Author: Renan Augusto Starke
*      Instituto Federal de Santa Catarina *
*
*
*     OLED control definitions from: https://github.com/yanbe/ssd1306-esp-idf-i2c
*
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "driver/i2c.h"

#include "i2c.h"
#include "ssd1366.h"
#include "font8x8_basic.h"

#define tag "SSD1306"

// Following definitions are bollowed from
// http://robotcantalk.blogspot.com/2015/03/interfacing-arduino-with-ssd1306-driven.html

// SLA (0x3C) + WRITE_MODE (0x00) =  0x78 (0b01111000)
#define OLED_I2C_ADDRESS   0x3C

#define OLED_HEIGHT 64
#define OLED_WIDTH 128

// Control byte
#define OLED_CONTROL_BYTE_CMD_SINGLE    0x80
#define OLED_CONTROL_BYTE_CMD_STREAM    0x00
#define OLED_CONTROL_BYTE_DATA_STREAM   0x40

// Fundamental commands (pg.28)
#define OLED_CMD_SET_CONTRAST           0x81    // follow with 0x7F
#define OLED_CMD_DISPLAY_RAM            0xA4
#define OLED_CMD_DISPLAY_ALLON          0xA5
#define OLED_CMD_DISPLAY_NORMAL         0xA6
#define OLED_CMD_DISPLAY_INVERTED       0xA7
#define OLED_CMD_DISPLAY_OFF            0xAE
#define OLED_CMD_DISPLAY_ON             0xAF
#define OLED_DEACTIVATE_SCROLL          0x2E

// Addressing Command Table (pg.30)
#define OLED_CMD_SET_MEMORY_ADDR_MODE   0x20    // follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD
#define OLED_CMD_SET_COLUMN_RANGE       0x21    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x7F = COL127
#define OLED_CMD_SET_PAGE_RANGE         0x22    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x07 = PAGE7

// Hardware Config (pg.31)
#define OLED_CMD_SET_DISPLAY_START_LINE 0x40
#define OLED_CMD_SET_SEGMENT_REMAP      0xA0
#define OLED_CMD_SET_MUX_RATIO          0xA8    // follow with 0x3F = 64 MUX
#define OLED_CMD_SET_COM_SCAN_MODE      0xC8
#define OLED_CMD_SET_DISPLAY_OFFSET     0xD3    // follow with 0x00
#define OLED_CMD_SET_COM_PIN_MAP        0xDA    // follow with 0x12
#define OLED_CMD_NOP                    0xE3    // NOP
#define OLED_SETSTARTLINE               0x40

// Timing and Driving Scheme (pg.32)
#define OLED_CMD_SET_DISPLAY_CLK_DIV    0xD5    // follow with 0x80
#define OLED_CMD_SET_PRECHARGE          0xD9    // follow with 0xF1
#define OLED_CMD_SET_VCOMH_DESELCT      0xDB    // follow with 0x30

// Charge Pump (pg.62)
#define OLED_CMD_SET_CHARGE_PUMP        0x8D    // follow with 0x14



uint8_t oled_buffer[(OLED_WIDTH * ((OLED_HEIGHT + 7) / 8))];

static void ssd1306_single_command(uint8_t data);
static void ssd1306_command_list(uint8_t *data, uint8_t size);


void ssd1306_clearDisplay_buffer(void) {
 memset(oled_buffer, 0, sizeof(oled_buffer));
}

void ssd1306_draw_pixel(int16_t x, int16_t y, pixel_color_t color){
 if ((x >= 0) && (x < OLED_WIDTH && (y >= 0) && (y < OLED_HEIGHT))) {
   if (color)
     oled_buffer[x + (y / 8) * OLED_WIDTH] &= ~(1 << (y & 7));
   else
     oled_buffer[x + (y / 8) * OLED_WIDTH] |= (1 << (y & 7));
 }
}

void ssd1306_init_new(){
  static const uint8_t init1[] = {OLED_CMD_DISPLAY_OFF,         // 0xAE
                               OLED_CMD_SET_DISPLAY_CLK_DIV, // 0xD5
                               0x80, // the suggested ratio 0x80
                               OLED_CMD_SET_MUX_RATIO // 0xA8
                             };
                               //0x00,
                               //0x3F}; // HEIGHT - 1

 static const uint8_t init2[] = {OLED_CMD_SET_DISPLAY_OFFSET, // 0xD3
                                 0x0,                      // no offset
                                 OLED_SETSTARTLINE | 0x0, // 0x40 line #0
                                 OLED_CMD_SET_CHARGE_PUMP // 0x8D
                                 };



 static const uint8_t init3[] = {0x00,
                                 OLED_CMD_SET_MEMORY_ADDR_MODE, // 0x20
                                 0x00, // 0x0 act like ks0108
                                 OLED_CMD_SET_SEGMENT_REMAP | 0x1, //0xa1
                                 OLED_CMD_SET_COM_SCAN_MODE, //0xc8
                                 };

 static const uint8_t init4[] = {
                         OLED_CMD_SET_VCOMH_DESELCT, // 0xDB
                         0x40,
                         OLED_CMD_DISPLAY_RAM, // 0xA4
                         OLED_CMD_DISPLAY_NORMAL,       // 0xA6
                         OLED_DEACTIVATE_SCROLL,
                         OLED_CMD_DISPLAY_ON}; // Main screen turn on

  /* First I2C transaction always fail */
  ssd1306_single_command(0x00);

  ssd1306_command_list((uint8_t *)init1, sizeof(init1));
  ssd1306_single_command(OLED_HEIGHT - 1);
  ssd1306_command_list((uint8_t *)init2, sizeof(init2));
  ssd1306_single_command(0x14); //vccstate == SSD1306_EXTERNALVCC
  ssd1306_command_list((uint8_t *)init3, sizeof(init3));

  ssd1306_single_command(OLED_CMD_SET_COM_PIN_MAP);
  ssd1306_single_command(0x12); //WIDTH == 128) && (HEIGHT == 64)
  ssd1306_single_command(OLED_CMD_SET_CONTRAST);
  ssd1306_single_command(0xCF);
  ssd1306_single_command(OLED_CMD_SET_PRECHARGE);
  ssd1306_single_command(0xF1);
  ssd1306_command_list((uint8_t *)init4, sizeof(init4));

}


void ssd1306_display_data(){
 i2c_cmd_handle_t cmd;
 static const uint8_t instr[] = {
                       OLED_CMD_SET_PAGE_RANGE,
                       0,                      // Page start address
                       0xFF,                   // Page end (not really, but works here)
                       OLED_CMD_SET_COLUMN_RANGE, 0}; // Column start address

 ssd1306_command_list((uint8_t *)instr, sizeof(instr));
 ssd1306_single_command(OLED_WIDTH - 1);

 uint8_t *data = oled_buffer;
 for (int i=0; i < 1024; i+=128){
   cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);

   i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
   i2c_master_write_byte(cmd, 0x40, true);
   i2c_master_write(cmd, data + i, 128, true);
   i2c_master_stop(cmd);
   i2c_master_cmd_begin(I2C_NUM_0, cmd, 100/portTICK_PERIOD_MS);

   i2c_cmd_link_delete(cmd);

 }
}

void ssd1306_draw_h_line(int16_t x, int16_t y, int16_t size, pixel_color_t color){
 for (int i=0;i < size;i++)
   ssd1306_draw_pixel(x+i,y,color);

}

void ssd1306_display_clear() {
	i2c_cmd_handle_t cmd;

 static const uint8_t init[] = {OLED_CMD_SET_PAGE_RANGE,   // 0x22
                               0, //
                               0xFF,
                               OLED_CMD_SET_COLUMN_RANGE, // 0x21
                               0 };

 ssd1306_command_list((uint8_t *)init, sizeof(init));
 ssd1306_single_command(OLED_WIDTH - 1);

	uint8_t zero[128]={0};
	for (uint8_t i = 0; i < 8; i++) {
		cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
		i2c_master_write_byte(cmd, 0x40, true);
		i2c_master_write(cmd, zero, 128, true);
		i2c_master_stop(cmd);
		i2c_master_cmd_begin(I2C_NUM_0, cmd, 100/portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);
	}

	// vTaskDelete(NULL);
}

static void ssd1306_single_command(uint8_t data){
 esp_err_t espRc;
 i2c_cmd_handle_t cmd = i2c_cmd_link_create();

 i2c_master_start(cmd);
 i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
 i2c_master_write_byte(cmd,0x00, true);
 i2c_master_write_byte(cmd, data, true);
 i2c_master_stop(cmd);

 espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100/portTICK_PERIOD_MS);

 i2c_cmd_link_delete(cmd);
}

static void ssd1306_command_list(uint8_t *data, uint8_t size){
 esp_err_t espRc;
 i2c_cmd_handle_t cmd = i2c_cmd_link_create();

 i2c_master_start(cmd);
 i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
 i2c_master_write_byte(cmd,0x00, true);
 i2c_master_write(cmd, data, size, true);
 i2c_master_stop(cmd);

 espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100/portTICK_PERIOD_MS);

 i2c_cmd_link_delete(cmd);
}

void task_ssd1306_display_text(const void *arg_text) {
	char *text = (char*)arg_text;
	uint8_t text_len = strlen(text);

	i2c_cmd_handle_t cmd;

	uint8_t cur_page = 0;

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
	i2c_master_write_byte(cmd, 0x00, true); // reset column
	i2c_master_write_byte(cmd, 0x10, true);
	i2c_master_write_byte(cmd, 0xB0 | cur_page, true); // reset page

	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	for (uint8_t i = 0; i < text_len; i++) {
		if (text[i] == '\n') {
			cmd = i2c_cmd_link_create();
			i2c_master_start(cmd);
			i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
			i2c_master_write_byte(cmd, 0x00, true); // reset column
			i2c_master_write_byte(cmd, 0x10, true);
			i2c_master_write_byte(cmd, 0xB0 | ++cur_page, true); // increment page

			i2c_master_stop(cmd);
			i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
			i2c_cmd_link_delete(cmd);
		} else {
			cmd = i2c_cmd_link_create();
			i2c_master_start(cmd);
			i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
			i2c_master_write(cmd, font8x8_basic_tr[(uint8_t)text[i]], 8, true);

			i2c_master_stop(cmd);
			i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
			i2c_cmd_link_delete(cmd);
		}
	}
}
