extern "C" {
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
}
#include "bom.h"

/*
 * Sharp protocol:
 *  Period modulation
 *   1: 2ms period
 *   0: 1ms period
 *  Signal:
 *   38kHz pulse for 320us, then off for rest of time (680us or 1680us)
 * BOM uses timer 4 for period timing, and timer 0 for 38kHz signal
 */

 /*
  * Recently modified so that when BOM is sending, it stops listening. When
  * not sending, it keeps listening.
  */

#define TIME_MICROS(us, prescale) (F_CPU / 1000000 * (us) / (prescale))

typedef uint16_t sharp_msg_t;

static sharp_msg_t sharp_msg_make(char address, bom_msg_t bom_msg) {
  return (((uint16_t)address & 0x3F) << 10) | ((uint16_t)bom_msg << 2) | 2;
}

static char robot_id;

// tx global vars
static sharp_msg_t out_msg;
static uint8_t out_pin_mask;
static char out_high, out_counter;
static volatile char out_done;

// rx global vars
static uint8_t prev_bom_sig;
static struct bom_rx_t {
    uint8_t count;
    uint16_t bits;
    int last_time;

    uint8_t new_data;
    uint8_t address;
    uint8_t data;
} bom_rx[4];

void set_robot_id(char id) {
  robot_id = id;
}

char get_robot_id(void) {
  return robot_id;
}

void bom_isr() {
  if (out_high) {
    PORT_BOM_EMIT ^= out_pin_mask;
  }
}

static void init_38kHz_signal() {
  out_high = 0;
  out_pin_mask = 0;

  // timer configuration now done in Atmega128rfa1.cpp
}

static void start_38kHz_signal() {
  TCNT3 = 0;
  PORT_BOM_EMIT |= out_pin_mask;
  out_high = 1;
}

static void stop_38kHz_signal() {
  PORT_BOM_EMIT &= ~ out_pin_mask; // why not just do PORT_BOM_EMIT &= 0; ?
  out_high = 0;
}

static void init_data_signal() {
  // timer 4 mode CTC (clear timer on compare), TOP = OCRA
  TCCR4A = 0;
  TCCR4B = _BV(WGM42);

  // run interrupt immediately when timer started
  OCR4A = 0;

  // enable interrupt
  TIMSK4 = _BV(OCIE4A);
}

static void start_data_signal() {
  TCNT4 = 0;

  // start timer 4 at F_CPU/64 prescaling
  TCCR4B |= _BV(CS41) | _BV(CS40);
}

static void stop_data_signal() {
  // stop timer 4
  TCCR4B &= ~ (_BV(CS42) | _BV(CS41) | _BV(CS40));
}

ISR(TIMER4_COMPA_vect) {
  if (out_high) {
    stop_38kHz_signal();
    if (out_counter) {
      out_counter--;
      if ((out_msg >> out_counter) & 1) {
        OCR4A = TIME_MICROS(1680, 64);
      } else {
        OCR4A = TIME_MICROS(680, 64);
      }
    } else {
      stop_data_signal();
      out_done = 1;
    }
  } else {
    start_38kHz_signal();
    OCR4A = TIME_MICROS(320, 64);
  }
}

void bom_init(void) {
  // BOM_SIG as input, interrupts enabled
  DDRB &= ~ (_BV(DDB0) | _BV(DDB1) | _BV(DDB2) | _BV(DDB3));
  PCMSK0 |= _BV(PCINT0) | _BV(PCINT1) | _BV(PCINT2) | _BV(PCINT3);
  PCICR |= _BV(PCIE0);

  // BOM_EMIT as output
  DDRF |= _BV(DDF4) | _BV(DDF5) | _BV(DDF6) | _BV(DDF7);

  init_38kHz_signal();
  init_data_signal();
}

static void stop_receiving(char dir) {
  switch (dir) {
    case BOM_FRONT: PCMSK0 &= ~_BV(PCINT0); break;
    case BOM_LEFT : PCMSK0 &= ~_BV(PCINT1); break;
    case BOM_RIGHT: PCMSK0 &= ~_BV(PCINT2); break;
    case BOM_BACK : PCMSK0 &= ~_BV(PCINT3); break;
  }
}

static void start_receiving(char dir) {
  // flush rx buffer before turning on the receiver
  bom_rx[(int)dir].bits = 0;
  bom_rx[(int)dir].count = 0;

  switch (dir) {
    case BOM_FRONT: PCMSK0 |= _BV(PCINT0); break;
    case BOM_LEFT : PCMSK0 |= _BV(PCINT1); break;
    case BOM_RIGHT: PCMSK0 |= _BV(PCINT2); break;
    case BOM_BACK : PCMSK0 |= _BV(PCINT3); break;
  }
}

void bom_send(char dir) {
  switch (dir) {
    case BOM_FRONT: out_pin_mask = _BV(P_BOM_EMIT0); break;
    case BOM_LEFT:  out_pin_mask = _BV(P_BOM_EMIT1); break;
    case BOM_RIGHT: out_pin_mask = _BV(P_BOM_EMIT2); break;
    case BOM_BACK:  out_pin_mask = _BV(P_BOM_EMIT3); break;
  }
  out_counter = 16;
  out_msg = sharp_msg_make(0x2A, bom_msg_make(robot_id, dir));
  out_done = 0;

  stop_receiving(dir); // disable receiver
  start_38kHz_signal(); // enable the transmitter
  start_data_signal();
  while (!out_done) {
    _delay_ms(0.1);
  }
  stop_data_signal(); // disable the transmitter
  stop_38kHz_signal();
  start_receiving(dir);  // enable the receiver
}

static void recv_edge(char is_rising, struct bom_rx_t *rx) {
  if (is_rising) {
    // TODO check 320us? or have 320us timeout on rising edge?
  } else {
    // uses timer 5, assuming prescale 1/64
    // timer 5 is set up by range_init()
    int now = TCNT5;
    int min_low = TIME_MICROS(MIN_LOW_PW, 64);
    int max_low = TIME_MICROS(MAX_LOW_PW, 64);
    int min_high = TIME_MICROS(MIN_HIGH_PW, 64);
    int max_high = TIME_MICROS(MAX_HIGH_PW, 64);

    if (rx->count) {
      int diff = (now - rx->last_time);
      rx->bits <<= 1;
      if (min_low < diff && diff < max_low) {
        // 0 already in bits
      } else if (min_high < diff && diff < max_high) {
        // add 1 to bits
        rx->bits |= 1;
      } else {
        // error, start from beginning
        rx->count = 0;
        rx->bits = 0;
      }
      if (rx->count == 16) {
        // finished!
        if ((rx->bits & 3) == 2) { // expansion and check bits
          rx->data = (rx->bits >> 2) & 0xff;
          rx->address = (rx->bits >> 10) & 0x1f;
          rx->new_data = 1;
        }
        rx->count = 0;
        rx->bits = 0;
      }
    }

    rx->count++;
    rx->last_time = now;
  }
}

ISR(PCINT0_vect) {
  uint8_t bom_sig = PIN_BOM_SIG;
  uint8_t changed = bom_sig ^ prev_bom_sig;
  if (changed & _BV(P_BOM_SIG0)) recv_edge(bom_sig & _BV(P_BOM_SIG0), &bom_rx[0]);
  if (changed & _BV(P_BOM_SIG1)) recv_edge(bom_sig & _BV(P_BOM_SIG1), &bom_rx[1]);
  if (changed & _BV(P_BOM_SIG2)) recv_edge(bom_sig & _BV(P_BOM_SIG2), &bom_rx[2]);
  if (changed & _BV(P_BOM_SIG3)) recv_edge(bom_sig & _BV(P_BOM_SIG3), &bom_rx[3]);
  prev_bom_sig = bom_sig;
}

int bom_get(char dir) {
  bom_rx_t *rx = &bom_rx[(int)dir];
  int ret = BOM_NO_DATA;
  cli();
  if (rx->new_data) {
    rx->new_data = 0;
    ret = rx->data;
  }
  sei();
  return ret;
}
