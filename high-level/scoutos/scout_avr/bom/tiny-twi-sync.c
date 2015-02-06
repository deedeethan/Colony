#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "tiny-twi-sync.h"

#define SDA PB0
#define SCL PB2

#define RECV_BUF_SIZE 70
#define SEND_BUF_SIZE 30

uint8_t address;

typedef enum {
  IDLE,
  ADDRESS,
  SEND_READ_ACK,
  SEND_WRITE_ACK,
  SEND_DATA,
  RECV_DATA,
  RECV_ACK
} state_t;

state_t state;
uint8_t recv_idx;
uint8_t recv_data[RECV_BUF_SIZE];
slave_rx_t onDataRecieved;
typedef struct {
  uint8_t start;
  uint8_t end;
  uint8_t data[SEND_BUF_SIZE];
  uint8_t full;
} queue;

queue send_queue;

static void twi_set_ctr(char n) {
  USISR = n;
}

static void twi_ready() {
  USISR = _BV(USISIF) | _BV(USIOIF) | _BV(USIPF) | _BV(USIDC);
  DDRB |= _BV(SCL);  //SCL output
  DDRB &= ~_BV(SDA); //SDA input

  state = IDLE;
}

static void twi_start() {
  // zero counter
  twi_set_ctr(0);

  state = ADDRESS;
  recv_idx = 0;
}

static void twi_send_ack(char is_read) {

  // set counter to 14
  twi_set_ctr(14);

  // set SDA low
  USIDR = 0;
  DDRB |= _BV(SDA);

  // set state
  state = is_read? SEND_READ_ACK : SEND_WRITE_ACK;
}

static void twi_send_data() {
  twi_set_ctr(0);
  if (send_queue.start == send_queue.end && !send_queue.full) {
    USIDR = 0xFF;
  } else {
    USIDR = send_queue.data[send_queue.start++];
    if (send_queue.start == SEND_BUF_SIZE)
      send_queue.start = 0;
    send_queue.full = 0;
  }
  DDRB |= _BV(SDA);
  state = SEND_DATA;
}

static void twi_recv_ack() {
  twi_set_ctr(14);
  DDRB &= ~_BV(SDA);
  state = RECV_ACK;
}

static void twi_on_overflow(void) {
  uint8_t input = USIDR; // TODO make sure this is correct
  switch (state) {

    case IDLE:
      // shouldn't reach here
      break;

    case ADDRESS:
      if (input >> 1 == address) {
        // send ack
        twi_send_ack(input & 1);
      } else {
        // go back to listening for start condition
        twi_ready();
      }
      break;

    case SEND_WRITE_ACK:
      DDRB &= ~_BV(SDA);
      state = RECV_DATA;
      break;

    case RECV_DATA:
      twi_send_ack(0);
      if (recv_idx < RECV_BUF_SIZE) {
        recv_data[recv_idx] = input;
        recv_idx++;
      }
      break;

    case SEND_READ_ACK:
      twi_send_data();
      break;

    case SEND_DATA:
      twi_recv_ack();
      break;

    case RECV_ACK:
      if (input & 1) {
        // received nak
        twi_ready();
      } else {
        // received ack
        twi_send_data();
      }
      break;
  }
}

static void twi_on_stop(void) {
  if (state == RECV_DATA) {
    onDataRecieved(recv_data, recv_idx);
    twi_ready();
  }
}

void smb_init(slave_rx_t callback) {
  // TODO do we want USICS0 (3)?
  USICR = _BV(USIWM1) | _BV(USIWM0) | _BV(USICS1);
  PORTB |= _BV(SDA) | _BV(SCL);
  twi_ready();
  onDataRecieved = callback;
}

void smb_set_address(uint8_t addr) {
  address = addr;
}

void smb_send_data(uint8_t* data, uint8_t length) {
  uint8_t i;
  for (i=0; i<length && !send_queue.full; i++) {
    send_queue.data[send_queue.end++] = data[i];
    if (send_queue.end == SEND_BUF_SIZE)
      send_queue.end = 0;
    if (send_queue.end == send_queue.start)
      send_queue.full = 1;
  }
}

void smb_poll(void) {
  if (USISR & _BV(USISIF)) {
    twi_start();
    USISR |= _BV(USISIF);
  }
  if (USISR & _BV(USIOIF)) {
    twi_on_overflow();
    USISR |= _BV(USIOIF);
  }
  if (USISR & _BV(USIPF)) {
    twi_on_stop();
    USISR |= _BV(USIPF);
  }
}
