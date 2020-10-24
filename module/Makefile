obj-m:=kirq.o
KDIR := /lib/modules/$(shell uname -r)/build
TARGET=user

all: default $(TARGET)

default:
	$(MAKE) -C $(KDIR) M=$(shell pwd) modules
	
$(TARGET):
	gcc user.c -o $(TARGET) -pthread -g

clean:
	rm -f *.[oas] .*.flags *.ko .*.cmd .*.d .*.tmp *.mod.c *.mod *.order *.symvers $(TARGET)

test:
	sudo insmod kirq.ko && sudo ./user && sudo rmmod kirq.ko
