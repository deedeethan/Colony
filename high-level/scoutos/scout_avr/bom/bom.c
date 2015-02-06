#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "tiny-twi.h"
#include "bomi2c.h"

char last_bit;
char count;
uint16_t data;
volatile char currently_sending;
char sending_counter;
uint16_t sending_data;

#define READ_BIT(pin, bit) (((pin) >> bit) & 1)
#define SET_BIT(pin, bit) ((pin) |= (1 << (bit)))
#define CLEAR_BIT(pin, bit) ((pin) &= ~(1 << (bit)))
#define PRESCALAR 8
#define TIME_US(us) (F_CPU * (us) / 1000000 / PRESCALAR)

#define ADDRESS 0x3 
#define DETECT PB3
#define EMIT PB4

#if DETECT == PB3
  #define PCINT_DETECT PCINT3
#endif
#if DETECT == PB4
  #define PCINT_DETECT PCINT4
#endif

static void bom_init(void) {

  SET_BIT(DDRB, EMIT);

  init_detect();

  init_timer1();

  sei();
}

////////////////////////////////// Receiving ///////////////////////////////////

static void init_detect(void) {
  TCCR0A |= _BV(WGM01);    // enable reset on compare A
  TIMSK  |= _BV(OCIE0A);   // enable timer0 compare match A
  OCR0A   = TIME_US(1000); // trigger every 1ms

  GIFR   |= _BV(PCIF);   // Clear any previous detections
  GIMSK  |= _BV(PCIE);   // Enable pin change interrupt for detection
  PCMSK  |= _BV(PCINT_DETECT); // Enable interrupt on detect pin
}

// Disables the interrupt that begins the detect process while it is running
static void disable_detect_interrupt(void) {
  PCMSK &= ~_BV(PCINT_DETECT);
}

static void start_timer0(void) {
  TCNT0 = 0; // Reset timer 0 count to 0
  TCCR0B |= _BV(CS01);     // setup clkio and start the timer
}

static void start_detect(void) {
  disable_detect_interrupt();
  data = 0;
  count = 0;
  last_bit = 0;
  start_timer0();
}

static void restart_detect(void) {
  TCCR0B &= ~0x07; // Stop timer0
  PCMSK  |= _BV(PCINT_DETECT); // Enable interrupt on detect pin
}

ISR(PCINT0_vect) {
  start_detect();
}

ISR(TIMER0_COMPA_vect) {

  char this_bit = READ_BIT(PINB, DETECT);

  if (this_bit == 1) {
    if (last_bit == 1) {
      restart_detect();
    }
  } else {
    data = data << 1 | (last_bit & 1);
    count++;
  }

  last_bit = this_bit;

  if (count == 15) {
    restart_detect();
    smb_send_data((uint8_t*) &data, sizeof(data));
  }
}

////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////// Sending ///////////////////////////////////

static void start_timer1() {
  TCNT1 = 0; // Reset timer 1 count to 0
  TCCR1 |= _BV(CS12); // Set prescalar to 8 and start timer1
}

static void stop_timer1() {
  TCCR1 &= ~0x0F; // Stop the timer without clearing CTC1
}

static void init_timer1() {
  OCR1B = TIME_US(320); // Set match b to fire every 320 us
  TIMSK |= _BV(OCIE1A) | _BV(OCIE1B); // Enable interrupt for match a and b
  TCCR1 |= _BV(CTC1);   // Enables resetting timer1 after matches match c
}

static void send_data(uint16_t data) {
  sending_data = data;
  sending_counter = 0;
  currently_sending = 1;
  SET_BIT(PORTB, EMIT);

  start_timer1();
  send_next_bit();
}

static void send_next_bit() {
  char next_bit = sending_data >> (14 - sending_counter) & 1;
  char ocr_temp = next_bit ? TIME_US(2000) : TIME_US(1000);
  OCR1A = ocr_temp; // Compare A - Turns on transmitter
  OCR1C = ocr_temp; // Reset the timer when match a fires
}

// Compare A - Turns on transmitter
ISR(TIMER1_COMPA_vect) {
  SET_BIT(PORTB, EMIT); 
  sending_counter++;
  send_next_bit();
}

// Compare B - Turns off transmitter after 320us
ISR(TIMER1_COMPB_vect) {
  CLEAR_BIT(PORTB, EMIT);
  if (sending_counter >= 15) {
    currently_sending = 0;
    stop_timer1();
  }
}

////////////////////////////////////////////////////////////////////////////////


static void slave_rx(uint8_t *buf, int len) {
  if (len >= 3) {
    switch (buf[0]) {
      case BOM_I2C_SEND:
        if (!currently_sending) send_data((uint16_t) buf[1] << 8 | buf[2]);  
        break;
    }
  }
}



int main() {
  bom_init();
  smb_init(slave_rx);
  smb_set_address(ADDRESS);
  for (;;);;;;;;;;;;;;;;;;; 
}
