//Digital Alarm Clock and Stopwatch
//Raspberry Pi Pico W, ESP32, SSD1306 OLED Display, MCP4822 DAC, 4 by 3 Matrix Keypad
//By Vignesh Saravanan
//Primary resources used were Hunter Adams' ECE4760 course lectures, which can be found over at ece4760.github.io


//libraries and header files
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/uart.h"
#include "rtc_time.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "ssd1306.h"
#include "math.h"

//constants and pin numbers
#define START 8
#define STOP 9
#define RESET 10
#define RTC 11
#define DEBOUNCE_MS 50
#define UART_ID uart0
#define BAUD_RATE 9600

#define two32 42946967296.0
#define DELAY 20
#define Fs 50000

//dac pins
#define DAC_PIN_CS 13
#define DAC_PIN_SCK 14
#define DAC_PIN_MOSI 15
#define DAC_SPI_PORT spi1

//channel A
#define DAC_CHANNEL_A  0x3000
//channel B
#define DAC_CHANNEL_B 0xB000


//hour, minute, and second displayed
volatile uint8_t hour = 0;
volatile uint8_t minute = 0;
volatile uint8_t second = 0;

//counter used for UART transmission (to set the real time clock)
volatile uint8_t counter = 0;

//value read from keypad
char reading[8] = "";
int valueRead;

//used to write alarm time to OLED display
int position = 0;
bool adjustAlarm = false;

//alarm times
uint8_t alarmHour = 0;
uint8_t alarmMinute = 0;
uint8_t alarmSecond = 0;

//used to start and stop the alarm
absolute_time_t alarmStart;
absolute_time_t alarmElapsed;

//used for real time clock mode
datetime_t dt_data;
datetime_t now_data;
datetime_t *dt = &dt_data;
datetime_t *now = &now_data;

//four states of the alarm clock
volatile bool startState = false;
volatile bool stopState = false;
volatile bool resetState = false;
volatile bool rtcState = false;

//used for direct digital synthesis
volatile int sineArray[256];
volatile unsigned int accumulator = 0;
volatile double frequency = 1740;
volatile unsigned int increment = (int)((1000 * two32)/Fs);
volatile uint16_t DAC_DATA;
volatile int x = 0;

//decodes string to yield the integer read by the keypad, -1 if nothing is read
int getReading(const char* str) {
    if (strcmp(str, "1101110") == 0) {
        return 1;
    }
    else if (strcmp(str, "1011110") == 0) {
        return 2;
    }
    else if (strcmp(str, "0111110") == 0) {
        return 3;
    }
    else if (strcmp(str, "1101101") == 0) {
        return 4;
    }
    else if (strcmp(str, "1011101") == 0) {
        return 5;
    }
    else if (strcmp(str, "0111101") == 0) {
        return 6;
    }
    else if (strcmp(str, "1101011") == 0) {
        return 7;
    }
    else if (strcmp(str, "1011011") == 0) {
        return 8;
    }
    else if (strcmp(str, "0111011") == 0) {
        return 9;
    }
    else if (strcmp(str, "1100111") == 0) {
        return 10;
    }
    else if (strcmp(str, "1010111") == 0) {
        printf("TARGET\n");
        return 0;
    }
    else if (strcmp(str, "0110111") == 0) {
        return -5;
    }
    else {
        return -1;
    }
}

//handles adjusting hour, minute, and second based on the current mode
void stopwatch_adjust() {
  if (startState) {
      second += 1;
      if (second == 60) {
        minute += 1;
        second = second%60;
      }
      if (minute == 60) {
         hour += 1;
         minute = minute%60;
      }
      if (hour == 24) {
         hour = hour%24;
      }
  }
  else if (resetState) {
      second = 0;
      minute = 0;
      hour = 0;
      startState = false;
      resetState = false;
      rtcState = false;
      stopState = true;
  }
  else if (rtcState) {
      rtc_time_get(now);
      hour = now->hour;
      second = now->sec;
      minute = now->min;
      startState = false;
      resetState = false;
      stopState = false;
      rtcState = true;
  }
}

//handles writing to the OLED display (used the ssd1306 header file
void oled_write() {
    const char* line1 = "Hour ";
    const char* line2 = "Minute ";
    const char* line3 = "Second ";
    const char* line4 = "Alarm ";
    int nextX;
    char data1[5];

    ssd1306_clear();

    //hour
    nextX = 0;
    nextX = ssd1306_draw_text(0, 0, line1);
    sprintf(data1, "%d", hour);
    nextX = ssd1306_draw_text(nextX, 0, data1);
    //minute
    nextX = 0;
    nextX = ssd1306_draw_text(0, 1, line2);
    sprintf(data1, "%d", minute);
    nextX = ssd1306_draw_text(nextX, 1, data1);
    //second
    nextX = 0;
    nextX = ssd1306_draw_text(0, 2, line3);
    sprintf(data1, "%d", second);
    nextX = ssd1306_draw_text(nextX, 2, data1);
    //alarm
    nextX = 0;
    nextX = ssd1306_draw_text(0, 3, line4);
    sprintf(data1, "%d", alarmHour);
    nextX = ssd1306_draw_text(nextX, 3, data1);
    nextX = ssd1306_draw_text(nextX, 3, " ");
    sprintf(data1, "%d", alarmMinute);
    nextX = ssd1306_draw_text(nextX, 3, data1);
    sprintf(data1, "%d", alarmSecond);
    nextX = ssd1306_draw_text(nextX, 3, " ");
    nextX = ssd1306_draw_text(nextX, 3, data1);
}

//used for debouncing the buttons
absolute_time_t last_interrupt_time = {0};

//button interrupt service routine, sets states accordingly
void button_isr(uint gpio, uint32_t events) {
    //debouncing
    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(last_interrupt_time, now) < 200000) return; // 200ms debounce
    last_interrupt_time = now;
    //once the system is in rtc state, it can only leave the state by pressing the RESET button
    if (gpio == START) {
        if (!rtcState) {
            startState = true;
            stopState = false;
            rtcState = false;
            resetState = false;
        }
    }
    else if (gpio == STOP) {
        if (!rtcState) {
            startState = false;
            stopState = true;
            rtcState = false;
            resetState = false;
        }
    }
    else if (gpio == RESET) {
        startState = false;
        stopState = false;
        resetState = true;
        rtcState = false;
    }
    else if (gpio == RTC) {
        startState = false;
        stopState = false;
        resetState = false;
        rtcState = true;
    }
}

//interrupt service routine used to produce the bird chip tone in the speaker (direct digital synthesis)
void alarm1_isr() {
    hw_clear_bits(&timer_hw->intr, 1u << 1);
    //sets subsequent alarm for the first 10 seconds after the alarm has started
    if (alarmElapsed - alarmStart < 10000000) {
        timer_hw->alarm[1] = timer_hw->timerawl + DELAY;
    }
    //rotating phasor (used to index fractionally through sine table), increment affects frequency
    accumulator += increment;
    //produces DAC data from sine table value
    DAC_DATA = DAC_CHANNEL_A | (sineArray[accumulator >> 24] & 0xffff);
    //used for frequency modulation (to produce chirp-like sound)
    x++;
    x = x%13000;
    int amplitude;
    //amplitude modulation (constant in the middle, ramps up at the beginning and ramps down at the end of the period)
    if (x <= 200) {
        amplitude = 260 * (x/200.0);
    }
    else if (x >= 12800) {
        amplitude = 260 * (13000 - x)/200.0;
    }
    else {
        amplitude = 260;
    }
    //frequency equation
    frequency = 1740 - amplitude * sin(M_PI * x/6500);
    increment = two32 * frequency/Fs;
    spi_write16_blocking(DAC_SPI_PORT, &DAC_DATA, 1);
    alarmElapsed = get_absolute_time();
}

//interrupt service routine used to write to the OLED display
void alarm0_isr() {
    hw_clear_bits(&timer_hw->intr, 1u << 0);
    //increment by one second
    stopwatch_adjust();
    oled_write();

    uint32_t now = timer_hw->timerawl;  // Get the current timer value (in microseconds)
    timer_hw->alarm[0] = now + 1000000;  // Set Alarm 0 to trigger 1 second later

}

//sets up alarm for writing to the OLED display
void setup_timer_interrupt() {
    //clear pending interrupts
    hw_clear_bits(&timer_hw->intr, 1u << 0);
    //enable interrupt on timer ALARM0
    hw_set_bits(&timer_hw->inte, 1u << 0);

    //set ISR in vector table (look-up table used by microprocessor to execute interrupt)
    irq_set_exclusive_handler(TIMER_IRQ_0, alarm0_isr);
    irq_set_enabled(TIMER_IRQ_0, true);

    //first alarm
    uint32_t now = timer_hw->timerawl;
    timer_hw->alarm[0] = (uint32_t)(now + 1000000);
}

//interrupt service routine for UART communication with ESP32
void uart_isr() {
    while (uart_is_readable(uart0)) {
        uint8_t temp = uart_getc(uart0);
        if (counter == 1) {
            dt->hour = temp;
        }
        else if (counter == 2) {
            dt->min = temp;
        }
        else if (counter == 3) {
            dt->sec = temp;
            counter = 0;
            rtc_time_set(dt);
            break;
        }
        counter++;
    }
    uart_putc(uart0, 'a');
}

int main() {
    //initializes all GPIO pins
    stdio_init_all();

    //the real-time hour/minute/second values are all 0 unless set otherwise
    dt->hour = 0;
    dt->min = 0;
    dt->sec = 0;

    //initializes real time clock peripheral
    rtc_time_init();
    //sets up timer interrupt
    setup_timer_interrupt();

    //initializes and clears OLED display
    ssd1306_init();
    ssd1306_clear();

    //initializes GPIO pins for mode control and sets as inputs
    gpio_init(START);
    gpio_init(STOP);
    gpio_init(RESET);
    gpio_init(RTC);
    gpio_set_dir(START, GPIO_IN);
    gpio_set_dir(STOP, GPIO_IN);
    gpio_set_dir(RESET, GPIO_IN);
    gpio_set_dir(RTC, GPIO_IN);

    //initializes keypad column pins as inputs (tied to external pull up resistors)
    for (int i = 22; i > 19; i--) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
    }

    //initializes keypad row pins as outputs
    for (int i = 19; i > 15; i--) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_OUT);
    }

    //initializes UART pins for communication with ESP32
    gpio_set_function(0, UART_FUNCSEL_NUM(uart0, 0));
    gpio_set_function(1, UART_FUNCSEL_NUM(uart0, 1));
    uart_init(uart0, 9600);

    //initializes UART interrupt service routine for receiving data
    irq_set_exclusive_handler(UART0_IRQ, uart_isr);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(uart0, true, false);

    //intializes button interrupts
    gpio_set_irq_enabled_with_callback(START, GPIO_IRQ_EDGE_RISE, true, &button_isr);
    gpio_set_irq_enabled(STOP, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(RESET, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(RTC, GPIO_IRQ_EDGE_RISE, true);

    //initializes spi channel
    spi_init(DAC_SPI_PORT, 20000000);
    spi_set_format(DAC_SPI_PORT, 16, 0, 0, 0);

    //initializes SPI GPIO pins
    gpio_set_function(DAC_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(DAC_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(DAC_PIN_CS, GPIO_FUNC_SPI);

    //initializes sine table (uses cmath library), scaling values to integers between 0 and 4095
    for (int i = 0; i < 256; i++) {
        //sine values scaled from 0 to 4095
        sineArray[i] = (int)(2047 * sin(2.0 * M_PI * i/256.0) + 2048);
    }

    //sets real time year, month, day for completion of the struct (not used in system)
    dt->year = 2025;
    dt->month = 5;
    dt->day = 16;

    //configures the real time clock peripheral with the current time, day, month, year
    rtc_time_set(dt);

    //configures interrupt for alarm1, used to write to the DAC (and produce bird chip)
    hw_set_bits(&timer_hw->inte, 1u << 1);
    irq_set_exclusive_handler(TIMER_IRQ_1, alarm1_isr);
    irq_set_enabled(TIMER_IRQ_1, alarm1_isr);
    rtc_time_get(now);

    while (true) {
        //starts alarm if the time matches the set alarm time
        if (now->hour == alarmHour && now->min == alarmMinute && now->sec == alarmSecond) {
            timer_hw->alarm[1] = timer_hw->timerawl + DELAY;
            alarmStart = get_absolute_time();
            alarmElapsed = alarmStart;
        }
        //scans all the rows of the keypad
        int i = 1;
        int a = i^(0b1111);
        while (i <= 8) {
            gpio_put(16, a%2);
            gpio_put(17, (a >> 1)%2);
            gpio_put(18, (a >> 2)%2);
            gpio_put(19, (a >> 3)%2);

            //waits for changes to gpio pins to propagate
            sleep_us(100);

            int a1 = gpio_get(22);
            int a2 = gpio_get(21);
            int a3 = gpio_get(20);
            int a4 = gpio_get(19);
            int a5 = gpio_get(18);
            int a6 = gpio_get(17);
            int a7 = gpio_get(16);

            //fills up string used to retrive the reading
            sprintf(reading, "%d%d%d%d%d%d%d", a1, a2, a3, a4, a5, a6, a7);
            valueRead = getReading(reading);

            //debouncing (waits until nothing is pressed after something is pressed)
            while(true) {
                char read[8];
                sprintf(read, "%d%d%d%d%d%d%d", gpio_get(22), gpio_get(21), gpio_get(20), gpio_get(19), gpio_get(18), gpio_get(17), gpio_get(16));
                if (getReading(read) == -1) {
                    break;
                }
                sleep_ms(50);
            }

            //adjusts alarm hour/minute/second accordingly based on key pressed in the matrix keypad
            if (valueRead != -1) {
                if (valueRead == 0) {
                    valueRead = 0;
                    strcpy(reading, "1010111");
                }
                if (valueRead == 10) {
                    alarmHour = 0;
                    alarmMinute = 0;
                    alarmSecond = 0;
                    position = 0;
                    adjustAlarm = true;
                }
                else if (position == 0 && adjustAlarm) {
                    alarmHour = 10 * valueRead;
                    position++;
                }
                else if (position == 1 && adjustAlarm) {
                    alarmHour += valueRead;
                    position++;
                }
                else if (position == 2 && adjustAlarm) {
                    alarmMinute = 10 * valueRead;
                    position++;
                }
                else if (position == 3 && adjustAlarm) {
                    alarmMinute += valueRead;
                    position++;
                }
                else if (position == 4 && adjustAlarm) {
                    alarmSecond = 10 * valueRead;
                    position++;
                }
                else if (position == 5 && adjustAlarm) {
                    alarmSecond += valueRead;
                    adjustAlarm = false;
                    position++;
                    position = position%6;
                }
                break;
            }
            i = i << 1;
            a = i^(0b1111);
            sleep_ms(10);
        }
        sleep_ms(50);
    }
    return 0;
}
