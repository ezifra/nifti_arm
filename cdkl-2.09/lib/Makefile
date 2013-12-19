TOPLEVEL = ..

include $(TOPLEVEL)/Config

ETHERCAN_LIB = ../ethercan/ethercan_s.o

ifeq ($(strip $(SERIAL_SUPPORT)),true)
SERIAL_LIB   = ../serial/serial.o
endif

OBJS = $(ETHERCAN_LIB) cpclib.o ini_parser.o cpc_async.o

ifeq ($(strip $(SOCKETCAN_SUPPORT)),true)
	OBJS += socketcan.o
	CLIBFLAGS += -DSOCKETCAN_SUPPORT
endif

ifeq ($(strip $(SERIAL_SUPPORT)),true)
	OBJS += $(SERIAL_LIB)
	CLIBFLAGS += -DSERIAL_SUPPORT
endif
  
LIBVER=2.42

CLIBFLAGS += -DCPC_LIB_VERSION=\"$(LIBVER)\" -D_CPC_CONF_INI -D_CPC_LIB_COMPAT_MODE

all: libcpc.a libcpc.so

cpclib.o : cpclib.c cpc_async.c socketcan.c ../include/cpclib.h ../include/ini_parser.h ../include/cpc.h ../ethercan/ethercan_s.o $(SERIAL_LIB)
	$(GCC) $(CLIBFLAGS) -fPIC -c ini_parser.c
	$(GCC) $(CLIBFLAGS) -fPIC -c cpclib.c
	$(GCC) $(CLIBFLAGS) -fPIC -c cpc_async.c
ifeq ($(strip $(SOCKETCAN_SUPPORT)),true)
	$(GCC) $(CLIBFLAGS) -fPIC -c socketcan.c
endif

libcpc.a: cpclib.o
	ar -r libcpc.a $(OBJS)

install: libcpc.a libcpc.so
	[ -e /etc/cpcconf.ini ] || install -o root -g users -m 664 cpcconf.ini /etc
	install -o root -g root -m 644 libcpc.a /usr/lib
	install -o root -g root -m 644 ../include/cpclib.h /usr/include
	install -o root -g root -m 644 libcpc.so /usr/lib/libcpc.so.$(LIBVER)
	ln -sf /usr/lib/libcpc.so.$(LIBVER) /usr/lib/libcpc.so

libcpc.so: cpclib.o
	$(GCC) $(CLIBFLAGS) -shared -Wl,-soname=libcpc.so.$(LIBVER) -o libcpc.so $(OBJS)

clean :
	rm -f *.o *.so *.a

