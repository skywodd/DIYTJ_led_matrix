/* Includes */
#include <util/delay.h>    /* For delay */
#include <avr/io.h>        /* For registers and IO */
#include <avr/interrupt.h> /* For timer 2 */
#include <stdint.h>        /* For hardcoded type */
#include <string.h>        /* For memset() */

/* Led matrix's constants */
#define MATRIX_COLUMN_COUNT 96 /*!< Size of a line (in pixels) */
#define MATRIX_LINE_COUNT 64   /*!< Number of lines (in pixels) */

/* Pin mapping's constants */
#define CTRL_PORT PORTC
#define CTRL_DDR DDRC
#define DATA_PORT PORTB
#define DATA_DDR DDRB

/*
 * Port pins map
 * 
 * Controls pins
 * 0: A
 * 1: B
 * 2: C
 * 3: D
 * 4: OE (active LOW)
 * 5: STR (active LOW)
 * 6: -
 * 7: - 
 * 
 * Data pins
 * 0: R
 * 1: G
 * 2: CLK
 * 3: -
 * 4: -
 * 5: -
 * 6: -
 * 7: -
 */

/** Framebuffer (double buffered) for Red and Green color 
 *
 * Array dimensions: [buffer][color][line][column]
 */
static volatile uint8_t framebuffer[2][2][MATRIX_LINE_COUNT][MATRIX_COLUMN_COUNT / 8];

/** Framebuffer color's index enumeration */
enum {
  FB_RED,
  FB_GREEN
};

/** Framebuffer pointer */
static volatile uint8_t (*framebuffer_user)[MATRIX_LINE_COUNT][MATRIX_COLUMN_COUNT / 8] = framebuffer[0];
static volatile uint8_t (*framebuffer_display)[MATRIX_LINE_COUNT][MATRIX_COLUMN_COUNT / 8] = framebuffer[1];

/** Framebuffer rotation flag */
static volatile uint8_t framebuffer_rotate_flag = 0;

/**
 * Software SPI with two synchroneous signals output
 *
 * @param red Red pixels data (8 pixels chunk)
 * @param green Green pixels data (8 pixels chunk)
 */
static inline void dualShiftOut(register uint8_t red, register uint8_t green) {
    DATA_PORT = (DATA_PORT & ~0b111) | !!(red & 1) | (!!(green & 1) << 1);
    DATA_PORT |= _BV(2);
    DATA_PORT &= ~_BV(2);
	DATA_PORT = (DATA_PORT & ~0b111) | !!(red & 2) | (!!(green & 2) << 1);
	DATA_PORT |= _BV(2);
	DATA_PORT &= ~_BV(2);
	DATA_PORT = (DATA_PORT & ~0b111) | !!(red & 4) | (!!(green & 4) << 1);
	DATA_PORT |= _BV(2);
	DATA_PORT &= ~_BV(2);
	DATA_PORT = (DATA_PORT & ~0b111) | !!(red & 8) | (!!(green & 8) << 1);
	DATA_PORT |= _BV(2);
	DATA_PORT &= ~_BV(2);
	DATA_PORT = (DATA_PORT & ~0b111) | !!(red & 16) | (!!(green & 16) << 1);
	DATA_PORT |= _BV(2);
	DATA_PORT &= ~_BV(2);
	DATA_PORT = (DATA_PORT & ~0b111) | !!(red & 32) | (!!(green & 32) << 1);
	DATA_PORT |= _BV(2);
	DATA_PORT &= ~_BV(2);
	DATA_PORT = (DATA_PORT & ~0b111) | !!(red & 64) | (!!(green & 64) << 1);
	DATA_PORT |= _BV(2);
	DATA_PORT &= ~_BV(2);
	DATA_PORT = (DATA_PORT & ~0b111) | !!(red & 128) | (!!(green & 128) << 1);
	DATA_PORT |= _BV(2);
	DATA_PORT &= ~_BV(2);
}

/**
 * Shift out a whole line of pixels
 * 
 * @param line_red_buffer Red line pixels buffer
 * @param line_green_buffer Green line pixels buffer
 */
static inline void lineShiftOut(volatile uint8_t *line_red_buffer, volatile uint8_t *line_green_buffer) {

	/* Send each line bytes */
    for (register int8_t i = ((MATRIX_COLUMN_COUNT / 8) - 1); i >= 0; i -= 4) {
	
		/* Send byte */
		dualShiftOut(line_red_buffer[i - 3], line_green_buffer[i - 3]);
		dualShiftOut(line_red_buffer[i - 2], line_green_buffer[i - 2]);
		dualShiftOut(line_red_buffer[i - 1], line_green_buffer[i - 1]);
		dualShiftOut(line_red_buffer[i], line_green_buffer[i]);
    }
}

/**
 * Interruption routine - line refresh at 60Hz
 */
ISR(TIMER2_COMPA_vect) {

  /* Internal variables */
  static uint8_t line_index = 0; /* Current line index */

  /* Latch armed & disable output & select line */
  CTRL_PORT = (CTRL_PORT & 0b11000000) | _BV(4) | line_index;

  /* Send each lines multiple of line_index */
  for (register int8_t line = (MATRIX_LINE_COUNT - (16 - line_index)); line >= 0; line -= 16) {
	
		/* Send line */
		lineShiftOut(framebuffer_display[FB_RED][line], framebuffer_display[FB_GREEN][line]);
  }
  
  /* Enable output & latch trigered & keep select line */
  CTRL_PORT |= _BV(5);
  CTRL_PORT &= ~_BV(4);

  /* Jump to the next line */
  if (++line_index == 16) {

    /* Reset line index */
    line_index = 0; 

    /* Check for buffer rotation */
    if (framebuffer_rotate_flag) {

      /* Rotate user and display framebuffer */
      volatile uint8_t (*tmp)[MATRIX_LINE_COUNT][MATRIX_COLUMN_COUNT / 8] = framebuffer_user;
      framebuffer_user = framebuffer_display;
      framebuffer_display = tmp;

      /* Unset the flag */
      framebuffer_rotate_flag = 0; 
    }
  }
}

/**
 * Set the state of a pixel in the USER framebuffer
 * 
 * @param color Color buffer to use
 * @param x X oosition of the pixel
 * @param y Y position of the pixel
 * @param state New state of the pixel
 */
static inline void setPixelAt(register const uint8_t color, register const uint8_t x, register const uint8_t y, register const uint8_t state) {
  if (!state)
    framebuffer_user[color][y][x / 8] |= _BV(x & 7);
  else
    framebuffer_user[color][y][x / 8] &= ~_BV(x & 7);
}

/**
 * Get the state of a pixel in the DISPLAY framebuffer
 * 
 * @param color Color buffer to use
 * @param x X oosition of the pixel
 * @param y Y position of the pixel
 * @return The state of the pixel
 */
static inline uint8_t getPixelAt(const uint8_t color, const uint8_t x, const uint8_t y) {
  return !(framebuffer_display[color][y][x / 8] & _BV(x & 7));
}

/* -------------------- User program -------------------- */

int main(void) {

  /* Setup control pins */
  CTRL_DDR = 255;       // All OUTPUT
  CTRL_PORT = 0b110000; // Output disable

  /* Setup data pins */
  DATA_DDR = 0b111;    // All OUTPUT
  DATA_PORT = 0b100;   // Clock init

  /* Init the frame buffer */
  memset(framebuffer, 255, 2 * 2 * MATRIX_LINE_COUNT * (MATRIX_COLUMN_COUNT / 8));
  
  /* Setup refresh timer */
  cli();
  TCCR2A = _BV(WGM21);             // CTC mode
  TCCR2B = _BV(CS22) | _BV(CS21);  // Prescaler /256
  TCNT2 = 0;                       // Counter reset
  OCR2A = (F_CPU / 256 / 960) - 1; // 960Hz ISR
  TIMSK2 = _BV(OCIE2A);            // Enable timer 2's compare match A ISR
  sei();
  
  /* Main loop */
  for(;;) {
  
    static uint8_t x1 = 0;
    static uint8_t x2 = MATRIX_COLUMN_COUNT - 1;
    static uint8_t y1 = 0;
    static uint8_t y2 = MATRIX_LINE_COUNT - 1;
	
	setPixelAt(FB_GREEN, x1, y1, 1);
	//setPixelAt(FB_GREEN, x2, y2, 0);
  
    if(++x1 == MATRIX_COLUMN_COUNT) {
	  x1 = 0;
	  if(++y1 == MATRIX_LINE_COUNT)
	    y1 = 0;
	}
	
	if(++x2 == MATRIX_COLUMN_COUNT) {
	  x2 = 0;
	  if(++y2 == MATRIX_LINE_COUNT)
	    y2 = 0;
	}
  
    framebuffer_rotate_flag = 1;
    while (framebuffer_rotate_flag);
  }
  
  /* Compiler fix */
  return 0;
}
