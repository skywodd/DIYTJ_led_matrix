/* Includes */
#include <avr/interrupt.h> /* For timer 2 */
#include <avr/io.h>        /* For registers and IO */
#include <stdint.h>        /* For hardcoded type */
#include <string.h>        /* For memset() */

/* UART macro */
#define BAUDRATE 460800
#define BAUD_PRESCALE(baudrate) ((F_CPU / ((baudrate) * 16UL)) - 1)

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
 * 6: debug ISR refresh
 * 7: debug ISR UART
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

#define DBG_REFRESH_HIGH() CTRL_PORT |= _BV(6)
#define DBG_REFRESH_LOW()  CTRL_PORT &= ~_BV(6)
#define DBG_REFRESH_TOGGLE() CTRL_PORT ^= _BV(6)
#define DBG_UART_HIGH() CTRL_PORT |= _BV(7)
#define DBG_UART_LOW()  CTRL_PORT &= ~_BV(7)
#define DBG_UART_TOGGLE() CTRL_PORT ^= _BV(7)
 
/** Framebuffer (triple buffered) for Red and Green color 
 *
 * Array dimensions: [buffer][color][line][column]
 */
static volatile uint8_t framebuffer[3][2][MATRIX_LINE_COUNT][MATRIX_COLUMN_COUNT / 8];

/** Framebuffer pointer */
static volatile uint8_t (*framebuffer_user)[MATRIX_LINE_COUNT][MATRIX_COLUMN_COUNT / 8] = framebuffer[0];
static volatile uint8_t (*framebuffer_wait)[MATRIX_LINE_COUNT][MATRIX_COLUMN_COUNT / 8] = framebuffer[1];
static volatile uint8_t (*framebuffer_display)[MATRIX_LINE_COUNT][MATRIX_COLUMN_COUNT / 8] = framebuffer[2];

/** Framebuffer rotation flag */
static volatile uint8_t framebuffer_rotate_flag = 0;

/** Buffer index : reset counter */
static volatile uint8_t raz_count = 0;

/** Buffer index : main counter */
static volatile uint16_t buffer_index = 0;

/**
 * Software SPI with two synchroneous signals output
 *
 * @param red Red pixels data (8 pixels chunk)
 * @param green Green pixels data (8 pixels chunk)
 */
static inline void dualShiftOut(uint8_t red, uint8_t green) {
	DATA_PORT = (DATA_PORT & ~0b111) | !!(red & 1) | (!!(green & 1) << 1);
    cli();
	DATA_PORT |= _BV(2);
    DATA_PORT &= ~_BV(2);
	sei();
	DATA_PORT = (DATA_PORT & ~0b111) | !!(red & 2) | (!!(green & 2) << 1);
	cli();
	DATA_PORT |= _BV(2);
	DATA_PORT &= ~_BV(2);
	sei();
	DATA_PORT = (DATA_PORT & ~0b111) | !!(red & 4) | (!!(green & 4) << 1);
	cli();
	DATA_PORT |= _BV(2);
	DATA_PORT &= ~_BV(2);
	sei();
	DATA_PORT = (DATA_PORT & ~0b111) | !!(red & 8) | (!!(green & 8) << 1);
	cli();
	DATA_PORT |= _BV(2);
	DATA_PORT &= ~_BV(2);
	sei();
	DATA_PORT = (DATA_PORT & ~0b111) | !!(red & 16) | (!!(green & 16) << 1);
	cli();
	DATA_PORT |= _BV(2);
	DATA_PORT &= ~_BV(2);
	sei();
	DATA_PORT = (DATA_PORT & ~0b111) | !!(red & 32) | (!!(green & 32) << 1);
	cli();
	DATA_PORT |= _BV(2);
	DATA_PORT &= ~_BV(2);
	sei();
	DATA_PORT = (DATA_PORT & ~0b111) | !!(red & 64) | (!!(green & 64) << 1);
	cli();
	DATA_PORT |= _BV(2);
	DATA_PORT &= ~_BV(2);
	sei();
	DATA_PORT = (DATA_PORT & ~0b111) | !!(red & 128) | (!!(green & 128) << 1);
	cli();
	DATA_PORT |= _BV(2);
	DATA_PORT &= ~_BV(2);
	sei();
}

/**
 * Shift out a whole line of pixels
 * 
 * @param line_red_buffer Red line pixels buffer
 * @param line_green_buffer Green line pixels buffer
 */
static inline void lineShiftOut(volatile uint8_t *line_red_buffer, volatile uint8_t *line_green_buffer) {

	/* Send each line bytes */
    for (int8_t i = ((MATRIX_COLUMN_COUNT / 8) - 1); i >= 0; i -= 4) {
	
		/* Send byte */
		dualShiftOut(line_red_buffer[i - 3], line_green_buffer[i - 3]);
		dualShiftOut(line_red_buffer[i - 2], line_green_buffer[i - 2]);
		dualShiftOut(line_red_buffer[i - 1], line_green_buffer[i - 1]);
		dualShiftOut(line_red_buffer[i], line_green_buffer[i]);
    }
}

/**
 * Serial RX interrrupt - store frame data
 */
ISR (USART0_RX_vect) {
//DBG_UART_HIGH();

  /* Avoid index reset */
  raz_count = 0;
  
  /* Store received data byte */
  ((uint8_t*) framebuffer_user)[buffer_index] = UDR0;
  
  /* Jump to the next data byte index */
  if(++buffer_index == 1536) {
  
    buffer_index = 0;
	//UDR0 = 'A';
	
	/* Rotate user and display framebuffer */
    volatile uint8_t (*tmp)[MATRIX_LINE_COUNT][MATRIX_COLUMN_COUNT / 8] = framebuffer_user;
    framebuffer_user = framebuffer_wait;
    framebuffer_wait = tmp;
  
    /* Turn flag and reset */
	framebuffer_rotate_flag = 1;
	
    DBG_UART_TOGGLE();
  }

//DBG_UART_LOW();
}

/* -------------------- User program -------------------- */

int main(void) {

  /* Setup control pins */
  CTRL_DDR = 255;       // All OUTPUT
  CTRL_PORT = 0b110000; // Output disable

  /* Setup data pins */
  DATA_DDR = 0b111;    // All OUTPUT
  DATA_PORT = 0b100;   // Clock init
  
  /* Clear display buffer */
  memset(framebuffer, 255, 3 * 2 * MATRIX_LINE_COUNT * (MATRIX_COLUMN_COUNT / 8));
  
  /* Setup UART */
  cli();
  UCSR0A = 0;
  UCSR0B = _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0);
  UCSR0C = _BV(UCSZ00) | _BV(UCSZ01); // 1 bit start, 8 bits data, 1 bit stop
  UBRR0H = BAUD_PRESCALE(BAUDRATE) >> 8;
  UBRR0L = BAUD_PRESCALE(BAUDRATE);
  
  /* Setup timer */
  TCCR2A = _BV(WGM21);             // CTC mode
  TCCR2B = _BV(CS22) | _BV(CS21);  // Prescaler /256
  TCNT2 = 0;                       // Counter reset
  OCR2A = (F_CPU / 256 / 960) - 1; // 960Hz ISR
  TIMSK2 = 0;                      // Disable timer 2's ISR
  sei();

  /* Main loop */
  for(;;) {
  
    /* Internal variables */
    static uint8_t line_index = 0; /* Current line index */
  
    /* Wait for timer compare match A */
	while(!(TIFR2 & _BV(OCF2A)));
	TIFR2 = _BV(OCF2A); /* Clear flag */
	
	DBG_REFRESH_HIGH();
	  
	  /* Latch armed & disable output & select line */
	  CTRL_PORT = (CTRL_PORT & 0b11000000) |  _BV(4) | line_index;

	  /* Send each lines multiple of line_index */
	  for (int8_t line = (MATRIX_LINE_COUNT - (16 - line_index)); line >= 0; line -= 16) {
		
			/* Send line */
			lineShiftOut(framebuffer_display[0][line], framebuffer_display[1][line]);
	  }
	  
	  /* Enable output & latch trigered & keep select line */
	  CTRL_PORT |= _BV(5);
	  CTRL_PORT &= ~_BV(4);

	  /* Jump to the next line */
	  if (++line_index == 16) {

		/* Reset line index */
		line_index = 0; 

		/* Check for buffer rotation */
		cli();
		if (framebuffer_rotate_flag) {

		  /* Rotate wait and display framebuffer */
		  volatile uint8_t (*tmp)[MATRIX_LINE_COUNT][MATRIX_COLUMN_COUNT / 8] = framebuffer_display;
		  framebuffer_display = framebuffer_wait;
		  framebuffer_wait = tmp;

		  /* Unset the flag */
		  framebuffer_rotate_flag = 0; 
		  
		  //DBG_REFRESH_TOGGLE();
		}
		sei();

		/* Handle buffer index reset */
		cli();
		if (++raz_count == 60) { // 1 second idle
		
		  /* Reset counter and buffer index */
		  buffer_index = 0;
		  raz_count = 0;
		  
		  //DBG_REFRESH_TOGGLE();
		}
		sei();
	  }
	  
	DBG_REFRESH_LOW();
  }
  
  /* Compiler fix */
  return 0;
}
