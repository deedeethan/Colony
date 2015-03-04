#include <avr/io.h>
#include <util/delay.h>

int main() {
  DDRB |= _BV(PB4);
  while (1) {
    PINB |= _BV(PB4);
    _delay_ms(500);
  }
  return 0;
}
