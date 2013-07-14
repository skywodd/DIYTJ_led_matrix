/* Includes */
#include <avr/io.h>        /* For registers and IO */
#include <avr/interrupt.h> /* For timer and ISR */
#include <stdint.h>        /* For hardcoded type */
#include <string.h>        /* For memset() */

/* Led matrix's constants */
#define MATRIX_COLUMN_COUNT 96 /*!< Size of a line (in pixels) */
#define MATRIX_LINE_COUNT 64   /*!< Number of lines (in pixels) */

/* Pin mapping's constants */
#define CTRL_PORT PORTC
#define CTRL_PIN PINC
#define CTRL_DDR DDRC
#define DATA_PORT PORTB
#define DATA_DDR DDRB

/**
 * Port pins map
 * 
 * Controls pins
 * 0: A
 * 1: B
 * 2: C
 * 3: D
 * 4: OE (active LOW)
 * 5: STR (active LOW)
 * 6: Display mode (input, LOW = LIN, HIGH = LOG)
 * 7: debug fft
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
 * 
 * Analog input
 * ADC0: audio input
 */
 
#define DBG_HIGH() CTRL_PORT |= 0b10000000
#define DBG_LOW() CTRL_PORT &= ~0b10000000
#define DBG_TOGGLE() CTRL_PORT ^= 0b10000000
 
/* FHT library */
#define WINDOW 1
#define REORDER 1
#define LOG_OUT 1
#define LIN_OUT 1
//#define LIN_OUT8 1
//#define SCALE 7
#define FHT_N 256
#include "FHT.h"

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

/** Audio buffer full flag */
static volatile uint8_t audiobuffer_full_flag = 0;

/**
 * Audio sampling 
 */
ISR(ADC_vect) {

  /* Internal variables */
  static uint16_t sample_index = 0; /* Current sample index */
  
  // Clear the timer interrupt
  TIFR0 = _BV(OCF0A) | _BV(OCF0B);
  
  // Store the sample (10 bits upscaled mode)
  int16_t s = (uint16_t) ADCL | (ADCH << 8);
  s -= 0x0200; // 10 bits unsigned to 16 bits signed
  s <<= 6;     //
  fht_input[sample_index] = s;
  
  // Jump next sample index
  if (++sample_index == FHT_N) {
  
    // Reset sample index
	sample_index = 0;
	
	// Stop sampling timer
	TCCR0B = 0;
	
	// Set "buffer full" flag
	audiobuffer_full_flag = 1;
  }
}

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

  /* Allow sampling interrupt to execute over the display interrupt */
  sei();
  
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

/* -------------------- User program -------------------- */

int main(void) {

  /* Disable interrupts */
  cli();

  /* Setup control pins */
  CTRL_DDR = 0b10111111; // All OUTPUT (except for MODE switch)
  CTRL_PORT = 0b1110000; // Output disable + pull-up on MODE switch

  /* Setup data pins */
  DATA_DDR = 0b111;      // All OUTPUT
  DATA_PORT = 0b100;     // Clock init

  /* Init the frame buffer */
  memset(framebuffer, 255, 2 * 2 * MATRIX_LINE_COUNT * (MATRIX_COLUMN_COUNT / 8));
  
  /* Setup refresh timer (timer 2) */
  TCCR2A = _BV(WGM21);             // CTC mode
  TCCR2B = _BV(CS22) | _BV(CS21);  // Prescaler /256
  TCNT2 = 0;                       // Counter reset
  OCR2A = (F_CPU / 256 / 960) - 1; // 960Hz ISR
  TIMSK2 = _BV(OCIE2A);            // Enable timer 2's compare match A ISR
  
  /* Setup sampling timer (timer 0) */
  TCCR0A = _BV(WGM01); // CTC mode
  TCCR0B = 0;          // Timer off
  TCNT0 = 0;           // Counter reset
  OCR0A = (F_CPU / 8 / 8000) - 1; // 8KHz timer reset (/!\ OCR = 249 @16MHz -> JUST IN !)
  OCR0B = (F_CPU / 8 / 8000) - 1; // 8KHz ADC start
  
  /* Setup analog pin */
  //DDRA &= ~_BV(0);                /* Set ADC0 pin as INPUT */
  ADMUX = _BV(REFS0);               /* AVCC reference + ADC0 */
  ADCSRB = _BV(ADTS1) | _BV(ADTS0); /* Auto-trigger on timer 0 compare match */
  DIDR0 = _BV(0);                   /* Disable input buffer of ADC0 */
  ADCSRA = _BV(ADEN) | _BV(ADIF) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2); 
  /* ADC enable with prescaler /16 and auto-trigger enable + interrupt */

  /* Enable interrupts */
  sei();
  
  /* Main loop */
  for(;;) {

    /* Rotate user and display buffer */
    framebuffer_rotate_flag = 1;
    while (framebuffer_rotate_flag);

    /* Audio processing */
	DBG_HIGH();
	TCNT0 = 0;
    TCCR0B = _BV(CS01);             // Start sampling timer
	while (!audiobuffer_full_flag); // Wait for buffer to be full
	audiobuffer_full_flag = 0;      // Reset flag
    fht_window();  // Window the data for better frequency response
    fht_reorder(); // Reorder the data before doing the fht
    fht_run();     // Process the data in the fht
	if (CTRL_PIN & _BV(6)) // If MODE switch is on
      fht_mag_log();  // Make the output of the fht
    else
	  fht_mag_lin(); // Make the output of the fht
	DBG_LOW();

	/* Display the FFT result */
	for (register uint16_t i = 0; i < MATRIX_COLUMN_COUNT; ++i) {
      
	  /* Turn data into absolute values */
	  register uint8_t s;
	  
	  /* Get output sample */
	  s = (CTRL_PIN & _BV(6)) ? fht_log_out[i + 2] : (fht_lin_out[i + 2] >> 3);
	  // Skip the two first bin for better visual effect
	  
	  /* Display amplitude bars */
	  setPixelAt(FB_GREEN, i, 63, 1);
	  setPixelAt(FB_GREEN, i, 62, s > 4);
	  setPixelAt(FB_GREEN, i, 61, s > 8);
	  setPixelAt(FB_GREEN, i, 60, s > 12);
	  setPixelAt(FB_GREEN, i, 59, s > 16);
	  setPixelAt(FB_GREEN, i, 58, s > 20);
	  setPixelAt(FB_GREEN, i, 57, s > 24);
	  setPixelAt(FB_GREEN, i, 56, s > 28);

	  setPixelAt(FB_GREEN, i, 55, s > 32);
	  setPixelAt(FB_GREEN, i, 54, s > 36);
	  setPixelAt(FB_GREEN, i, 53, s > 40);
	  setPixelAt(FB_GREEN, i, 52, s > 44);
	  setPixelAt(FB_GREEN, i, 51, s > 48);
	  setPixelAt(FB_GREEN, i, 50, s > 52);
	  setPixelAt(FB_GREEN, i, 49, s > 56);
	  setPixelAt(FB_GREEN, i, 48, s > 60);

	  setPixelAt(FB_GREEN, i, 47, s > 64);
	  setPixelAt(FB_GREEN, i, 46, s > 68);
	  setPixelAt(FB_GREEN, i, 45, s > 72);
	  setPixelAt(FB_GREEN, i, 44, s > 76);
	  setPixelAt(FB_GREEN, i, 43, s > 80);
	  setPixelAt(FB_GREEN, i, 42, s > 84);
	  setPixelAt(FB_GREEN, i, 41, s > 88);
	  setPixelAt(FB_GREEN, i, 40, s > 92);

	  setPixelAt(FB_GREEN, i, 39, s > 96);
	  setPixelAt(FB_GREEN, i, 38, s > 100);
	  setPixelAt(FB_GREEN, i, 37, s > 104);
	  setPixelAt(FB_GREEN, i, 36, s > 108);
	  setPixelAt(FB_GREEN, i, 35, s > 112);
	  setPixelAt(FB_GREEN, i, 34, s > 116);
	  setPixelAt(FB_GREEN, i, 33, s > 120);
	  setPixelAt(FB_GREEN, i, 32, s > 124);

	  setPixelAt(FB_GREEN, i, 31, s > 128);
	  setPixelAt(FB_GREEN, i, 30, s > 132);
	  setPixelAt(FB_GREEN, i, 29, s > 136);
	  setPixelAt(FB_GREEN, i, 28, s > 140);
	  setPixelAt(FB_GREEN, i, 27, s > 144);
	  setPixelAt(FB_GREEN, i, 26, s > 148);
	  setPixelAt(FB_GREEN, i, 25, s > 152);
	  setPixelAt(FB_GREEN, i, 24, s > 156);

	  setPixelAt(FB_GREEN, i, 23, s > 160);
	  setPixelAt(FB_RED, i, 23, s > 160);
	  setPixelAt(FB_GREEN, i, 22, s > 164);
	  setPixelAt(FB_RED, i, 22, s > 164);
	  setPixelAt(FB_GREEN, i, 21, s > 168);
	  setPixelAt(FB_RED, i, 21, s > 168);
	  setPixelAt(FB_GREEN, i, 20, s > 172);
	  setPixelAt(FB_RED, i, 20, s > 172);
	  setPixelAt(FB_GREEN, i, 19, s > 176);
	  setPixelAt(FB_RED, i, 19, s > 176);
	  setPixelAt(FB_GREEN, i, 18, s > 180);
	  setPixelAt(FB_RED, i, 18, s > 180);
	  setPixelAt(FB_GREEN, i, 17, s > 184);
	  setPixelAt(FB_RED, i, 17, s > 184);
	  setPixelAt(FB_GREEN, i, 16, s > 188);
	  setPixelAt(FB_RED, i, 16, s > 188);

	  setPixelAt(FB_GREEN, i, 15, s > 192);
	  setPixelAt(FB_RED, i, 15, s > 192);
	  setPixelAt(FB_GREEN, i, 14, s > 196);
	  setPixelAt(FB_RED, i, 14, s > 196);
	  setPixelAt(FB_GREEN, i, 13, s > 200);
	  setPixelAt(FB_RED, i, 13, s > 200);
	  setPixelAt(FB_GREEN, i, 12, s > 204);
	  setPixelAt(FB_RED, i, 12, s > 204);
	  setPixelAt(FB_GREEN, i, 11, s > 208);
	  setPixelAt(FB_RED, i, 11, s > 208);
	  setPixelAt(FB_GREEN, i, 10, s > 212);
	  setPixelAt(FB_RED, i, 10, s > 212);
	  setPixelAt(FB_GREEN, i, 9, s > 216);
	  setPixelAt(FB_RED, i, 9, s > 216);
	  setPixelAt(FB_GREEN, i, 8, s > 220);
	  setPixelAt(FB_RED, i, 8, s > 220);

	  setPixelAt(FB_RED, i, 7, s > 224);
	  setPixelAt(FB_RED, i, 6, s > 228);
	  setPixelAt(FB_RED, i, 5, s > 232);
	  setPixelAt(FB_RED, i, 4, s > 236);
	  setPixelAt(FB_RED, i, 3, s > 240);
	  setPixelAt(FB_RED, i, 2, s > 244);
	  setPixelAt(FB_RED, i, 1, s > 248);
	  setPixelAt(FB_RED, i, 0, s > 252);
    }
  }
  
  /* Compiler fix */
  return 0;
}
