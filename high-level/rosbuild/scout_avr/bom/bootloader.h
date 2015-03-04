#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#define BOM_I2C_PGRM 30
#define BOM_I2C_STRT 31
#define START_ADDR 0xA00

void strt_pgrm(void);

void boot_program_page (uint32_t page, uint8_t *buf);

#endif
