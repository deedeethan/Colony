#include "Atmega128rfa1.h"
#include "bom.h"

extern "C"
{
#include <avr/io.h>
#include <avr/interrupt.h>
  void __cxa_pure_virtual(void) {}
}

#define T0_KHZ 76
#define T0_OCR (F_CPU / 1000 / T0_KHZ) // 210 ticks
#define T0_ERROR (F_CPU / 1000 - T0_OCR*T0_KHZ) // 40 ticks/ms

unsigned char t0_count, t0_error;
unsigned long millis;

int rx_start, rx_end;
char rx_buffer[RX_BUFFER_SIZE];

Atmega128rfa1::Atmega128rfa1()
{
}

ISR(TIMER0_COMPA_vect)
{
  bom_isr();

  // F_CPU = T0_OCR * T0_KHZ + T0_ERROR
  // every millisecond, accumulate T0_ERROR, and when it reaches T0_OCR skip
  // one iteration
  t0_count++;
  if (t0_count >= T0_KHZ) {
    t0_error += T0_ERROR;
    if (t0_error < T0_OCR) {
      t0_count = 0;
    } else {
      t0_count = -1;
      t0_error -= T0_OCR;
    }
    millis++;
  }
}

ISR(USART0_RX_vect)
{
  char data = UDR0;
  int new_end = rx_end+1;
  if (new_end == RX_BUFFER_SIZE) {
    new_end = 0;
  }
  if (new_end == rx_start)
  {
    // TODO warn of buffer overflow?
  }
  else
  {
    rx_buffer[rx_end] = data;
    rx_end = new_end;
  }
}

void Atmega128rfa1::init()
{
  // === init serial ===
  // baud = F_CPU / (16 (UBRR + 1))
  uint16_t ubrr = F_CPU / 16 / BAUD_RATE - 1;
  UBRR0H = ubrr >> 8;
  UBRR0L = ubrr;
  // UMSEL0 = 0, asynchronous usart
  // UPM0 = 0, parity check disabled
  // USBS0 = 0, 1 stop bit
  // UCSZ0 = 3, 8-bit
  UCSR0B = _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0);
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);

  // === init time ===
  // COM0x = 0, pin OC0x not used
  // WGM0 = 2, clear timer on compare match, TOP = OCRA
  // CS0 = 1, no prescaler
  TCCR0A = _BV(WGM01);
  TCCR0B = _BV(CS00);
  // enable interrupt on compare match A
  TIMSK0 = _BV(OCIE0A);
  OCR0A = T0_OCR;
  millis = 0;

  sei();
}

int Atmega128rfa1::read()
{
  int ret;
  cli();
  if (rx_start == rx_end)
    ret = -1;
  else
  {
    ret = rx_buffer[rx_start];
    rx_start++;
    if (rx_start == RX_BUFFER_SIZE)
      rx_start = 0;
  }
  sei();
  return ret;
}

void Atmega128rfa1::write(uint8_t* data, int length)
{
  // TODO make this non-blocking with a tx buffer
  int i;
  for (i = 0; i < length; i++)
  {
    while (!(UCSR0A & _BV(UDRE0)));
    UDR0 = data[i];
  }
}

unsigned long Atmega128rfa1::time()
{
  unsigned long ret;
  cli();
  ret = millis;
  sei();
  return ret;
}
