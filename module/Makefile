obj-m:=kirq.o
KDIR := /lib/modules/$(shell uname -r)/build
TARGET=kirq_test

all: default $(TARGET)

default:
	$(MAKE) -C $(KDIR) M=$(shell pwd) modules
	
$(TARGET): kirq_test.c
	gcc $^ -o $(TARGET) -pthread -g

clean:
	rm -f *.[oas] .*.flags *.ko .*.cmd .*.d .*.tmp *.mod.c *.mod *.order *.symvers $(TARGET)

test:
	sudo insmod kirq.ko && sudo ./kirq_test && sudo rmmod kirq.ko
