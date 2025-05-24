//driver file for SSD1306
//By Vignesh Saravanan

#ifndef SSD1306_H
#define SSD1306_H

#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <stdint.h>
#include "font5x7.h"

//height and width
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64

//spi communication pins
#define SPI_PORT spi0
#define PIN_SCK 2
#define PIN_MOSI 3
#define PIN_CS 5
#define PIN_DC 6
#define PIN_RES 7

static uint8_t buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
static uint8_t num[8][16];


void ssd1306_reset() {
  gpio_put(PIN_RES, 0);
  sleep_ms(10);
  gpio_put(PIN_RES, 1);
  sleep_ms(10);
}

void ssd1306_send_command(uint8_t cmd) {
    gpio_put(PIN_DC, 0); // command mode
    gpio_put(PIN_CS, 0); // active low chip select set to true
    spi_write_blocking(SPI_PORT, &cmd, 1);
    gpio_put(PIN_CS, 1);
}

void ssd1306_send_data(uint8_t data, size_t len) {
    gpio_put(PIN_DC, 1);
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, &data, len);
    gpio_put(PIN_CS, 1);
}

void ssd1306_init() {
  spi_init(SPI_PORT, 1000000);
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

  gpio_init(PIN_CS);
  gpio_set_dir(PIN_CS, GPIO_OUT);
  gpio_put(PIN_CS, 1);

  gpio_init(PIN_DC);
  gpio_set_dir(PIN_DC, GPIO_OUT);

  gpio_init(PIN_RES);
  gpio_set_dir(PIN_RES, GPIO_OUT);

  ssd1306_reset();

  ssd1306_send_command(0xAE); //turns display off
  ssd1306_send_command(0xD5); //default clock divider, factor of 1
  ssd1306_send_command(0x80);
  ssd1306_send_command(0xA8); //multiplex ratio (number of rows)
  ssd1306_send_command(0x3F); //rows 0 - 63
  ssd1306_send_command(0xD3); //no vertical offset (display is not shifted down or up)
  ssd1306_send_command(0x00);
  ssd1306_send_command(0x40); //start display at row 0
  ssd1306_send_command(0x00);
  ssd1306_send_command(0x8D); //oled needs 9V, so it steps up the 3.3V from microcontroller
  ssd1306_send_command(0x14); //as long as we aren't providing a separate 9V supply
  ssd1306_send_command(0x20); //horizontal memory addressing mode (8 bit horizontal pages of data)
  ssd1306_send_command(0x00);
  ssd1306_send_command(0xA0); //left to right column addressing
  ssd1306_send_command(0xC0); //scan from top to bottom
  ssd1306_send_command(0x81); //default contrast level: max brightness
  ssd1306_send_command(0xCF);
  ssd1306_send_command(0xD9); //pre-charge period set to default (time that the system waits for capacitors to charge)
  ssd1306_send_command(0xF1);
  ssd1306_send_command(0xDB); //VCOMH deselect level (0.77 * vcc)
  ssd1306_send_command(0x20);
  ssd1306_send_command(0xA6); //normal display (not inverted)
  ssd1306_send_command(0xAF); //turn the display on
}


void ssd1306_clear() {
	for (uint8_t i = 0; i < 8; i++) {
       ssd1306_send_command(0xB0 + i);
       ssd1306_send_command(0x00);
       ssd1306_send_command(0x10);
       for (uint8_t col = 0; col < 128; col++) {
           ssd1306_send_data(0x00, 1);
       }
    }
}

int ssd1306_draw_char(uint8_t x, uint8_t page, char c) {
    if (c < 32 || c > 126) c = '?';
    ssd1306_send_command(0xB0 + page); // sets page address
    ssd1306_send_command(0x00 + (x & 0x0F)); //sets column start address
    ssd1306_send_command(0x10 + (x >> 4));
    for (int i = 0; i < 5; i++) {
        ssd1306_send_data((uint8_t)font5x7[c-32][i], 1); // copies pixel values from font array
    }
    ssd1306_send_data(0x00, 1); // adds column of space
    x += 6;
    return x;
}

//writes text
int ssd1306_draw_text(uint8_t x, uint8_t page, const char* str) {
    while (*str && x < 128 && page < 8) {
        ssd1306_draw_char(x, page, *str++);
        x += 6;
    }
    return x;
}

void updateNumArray() {
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 16; j++) {
            if (i%2 == 0 && j%2 == 0) {
                num[i][j] = 0x7F;
            }
            else {
                num[i][j] = 0x00;
            }
        }
    }
}

void ssd1306_fill_blocks() {
    ssd1306_clear();
    updateNumArray();
    for (uint8_t i = 0; i < 8; i++) {
        ssd1306_send_command(0xB0 + i);
        ssd1306_send_command(0x00);
        ssd1306_send_command(0x10);
        for (uint8_t col = 0; col < 128; col++) {
            ssd1306_send_data(num[i][col/8], 1);
        }
    }
}



#endif //SSD1306_H
