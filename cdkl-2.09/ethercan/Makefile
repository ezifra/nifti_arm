TOPLEVEL = ..

include $(TOPLEVEL)/Config

all: ethercan_s.o

ethercan_s.o : ethercan_s.c ../include/ethercan.h
	$(GCC) $(CFLAGS) $(CLIBFLAGS) -fPIC -c -o ethercan_s.o ethercan_s.c

clean :
	rm -f ethercan_s.o

