#ifndef TINY_TWI_SYNC_H
#define TINY_TWI_SYNC_H

#define BOM_I2C_SEND 1

#include <stdint.h>

typedef void (*slave_rx_t)(uint8_t*, int);

void smb_init(slave_rx_t slave_rx);
void smb_set_address(uint8_t addr);
void smb_send_data(uint8_t* data, uint8_t length);
void smb_poll(void);

#endif
