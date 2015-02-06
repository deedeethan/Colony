#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "tiny-twi-sync.h"


static void slave_rx(uint8_t *buf, int len) {
  if (len > 1) {
    switch (buf[0]) {
      case BOM_I2C_SEND:
        smb_send_data(buf, len);
    }
  }
}

int main(void) {
    smb_init(slave_rx);
    smb_set_address(3);
    twi_run();
}