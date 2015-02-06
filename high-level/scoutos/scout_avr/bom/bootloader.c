#include <inttypes.h>
#include <avr/boot.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "tiny-twi-sync.h"
#include "bootloader.h"

uint32_t current_page;

void boot_program_page (uint32_t page, uint8_t *buf)
{
  uint16_t i;
  uint8_t sreg;

  // Disable interrupts.

  sreg = SREG;
  cli();

  eeprom_busy_wait ();

  boot_page_erase (page);
  boot_spm_busy_wait ();      // Wait until the memory is erased.

  for (i=0; i<SPM_PAGESIZE; i+=2)
  {
    // Set up little-endian word.
    uint16_t w = *buf++;
    w += (*buf++) << 8;

    boot_page_fill (page + i, w);
  }

  boot_page_write (page);     // Store buffer in flash page.
  boot_spm_busy_wait();       // Wait until the memory is written.

  // Re-enable interrupts (if they were ever enabled).

  SREG = sreg;
}

static void bom_pgrm(uint8_t *buf, int len) {
  if (len == SPM_PAGESIZE) {
    if (current_page == 0) {
      uint16_t w = 0xC000 | ((START_ADDR >> 1) - 1);
      buf[0] = w & 0xFF;
      buf[1] = (w >> 8) & 0xFF;
    }
    boot_program_page(current_page, buf);
    current_page += SPM_PAGESIZE;
  }
}

static void slave_rx(uint8_t *buf, int len) {
  if (len >= 1) {
    switch (buf[0]) {
      case BOM_I2C_PGRM:
        bom_pgrm(buf+1, len-1);
        break;
      case BOM_I2C_STRT:
        strt_pgrm(); //yolo
        break;
    }
  }
}

static void timer_init(void) {
  TCNT0 = 0;

  OCR0A = 200;

  // clear on compare match A
  TCCR0A |= _BV(WGM01);
  // 1024 prescaler
  TCCR0B |= _BV(CS02) | _BV(CS00);
}

int main() {
  smb_init(slave_rx);
  smb_set_address(3); // TODO parameterize address
  while (1) {
    smb_poll();
  }
  return 0;
}
