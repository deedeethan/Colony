#include <linux/i2c-dev.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdio.h>

#define SLAVE_ADDR 5

int main(int argc, char **argv) {
  int i;
  if (argc <= 1) return 255;
  int f = open(argv[1], O_RDWR);
  if (f < 0) {
    perror("open");
    return 255;
  }
  if (ioctl(f, I2C_SLAVE, SLAVE_ADDR) < 0) {
    perror("ioctl");
    return 255;
  }
  
  /*uint8_t A[32]; 
  int n = i2c_smbus_read_block_data(f, 0x00, A);
  if (n < 0) {
    perror("i2c_smbus_read_block_data");
    return 255;
  }

  uint8_t A[] = {0x20, 0x30, 0x40, 0x50};
  int n = write(f, A, 4);
  if (n < 0) {
    perror("write");
    return 255;
  }*/

  uint8_t A[5];
  int n = read(f, A, 5);
  if (n < 0) {
    perror("read");
    return 255;
  }

  A[0] = 'a';
  A[1] = 'b';
  n = write(f, A, 2);
  if (n < 0) {
    perror("write");
    return 255;
  }

  printf("%d\n", n);
  for (i=0; i<sizeof(A); i++) printf("%d ", A[i]);
  printf("\n");

  return 0;
}

