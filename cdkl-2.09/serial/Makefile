TOPLEVEL = ..

include $(TOPLEVEL)/Config

ifeq ($(strip $(SERIAL_SUPPORT)), true)
 	CLIBFLAGS += -DSERIAL_SUPPORT
endif

all: serial.o

serial.o : serial.c ../include/serial.h
ifeq ($(strip $(SERIAL_SUPPORT)), true)
	$(GCC) $(CFLAGS) $(CLIBFLAGS) -lbluetooth -fPIC -c -o serial.o serial.c
endif

clean :
	rm -f serial.o

