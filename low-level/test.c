#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <stdio.h>

#define CONFIG_SOC_IMX6Q
#include "iomux-v3.h"
#include "iomux-mx6q.h"

#define IOMUXC_BASE 0x020E0000

// alt 6 i2c1 sda
#define IOMUXC_SW_MUX_CTL_PAD_EIM_DATA21 0x0A4
#define IOMUXC_SW_PAD_CTL_PAD_EIM_DATA21 0x3B8
#define IOMUXC_I2C1_SCL_IN_SELECT_INPUT  0x898

// alt 1 i2c1 scl
#define IOMUXC_SW_MUX_CTL_PAD_EIM_DATA28 0x0C4
#define IOMUXC_SW_PAD_CTL_PAD_EIM_DATA28 0x3D8
#define IOMUXC_I2C1_SDA_IN_SELECT_INPUT  0x89C

#define io_read(base, offset) \
  (*(unsigned int*)((char*)(base) + (offset)))

#define io_write(base, offset, value) \
  (io_read(base, offset) = (value))

#define SIZE 0x1000

int main(int argc, char **argv) {
  int fd = open("/dev/mem", O_RDWR);
  if (fd < 0) {
    perror("open");
    return 1;
  }

  void *base = mmap(NULL, SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, IOMUXC_BASE);
  if (base == MAP_FAILED) {
    perror("mmap");
    return 1;
  }

  if (argc == 1) {
    printf("0x%08x 0x%08x 0x%08x\n",
      io_read(base, IOMUXC_SW_MUX_CTL_PAD_EIM_DATA21),
      io_read(base, IOMUXC_SW_PAD_CTL_PAD_EIM_DATA21),
      io_read(base, IOMUXC_I2C1_SCL_IN_SELECT_INPUT));
    printf("0x%08x 0x%08x 0x%08x\n",
      io_read(base, IOMUXC_SW_MUX_CTL_PAD_EIM_DATA28),
      io_read(base, IOMUXC_SW_PAD_CTL_PAD_EIM_DATA28),
      io_read(base, IOMUXC_I2C1_SDA_IN_SELECT_INPUT));
  } else {
    // set MUX EIM_DATA21 to I2C1 SCL, with SION
    io_write(base, IOMUXC_SW_MUX_CTL_PAD_EIM_DATA21, 0x16);

    // set PAD EIM_DATA21 to who knows what
    io_write(base, IOMUXC_SW_PAD_CTL_PAD_EIM_DATA21, MX6Q_I2C_PAD_CTRL);

    // set I2C SCL input to EIM_DATA21 ALT6
    io_write(base, IOMUXC_I2C1_SCL_IN_SELECT_INPUT, 0);

    // set MUX EIM_DATA28 to I2C1 SDA, with SION
    io_write(base, IOMUXC_SW_MUX_CTL_PAD_EIM_DATA28, 0x11);

    // set PAD EIM_DATA28 to who knows what
    io_write(base, IOMUXC_SW_PAD_CTL_PAD_EIM_DATA28, MX6Q_I2C_PAD_CTRL);

    // set I2C SDA input to EIM_DATA28 ALT1
    io_write(base, IOMUXC_I2C1_SDA_IN_SELECT_INPUT, 0);
  }
  
  munmap(base, SIZE);
  return 0;
}
