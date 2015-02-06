obj-m += encoder.o
KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build

all: encoder.ko

encoder.ko: encoder.c
	make -C $(KERNEL_DIR) M=$(PWD) modules

clean:
	make -C $(KERNEL_DIR) M=$(PWD) clean
