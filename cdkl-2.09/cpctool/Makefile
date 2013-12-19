TOPLEVEL = ..

include $(TOPLEVEL)/Config

EXEC = cpctool

SRC = cpctool.c kbhit.c utility.c
OBJ = $(SRC:.c=.o)

LDFLAGS=-lbluetooth

INCLUDE = ../include
CPCLIB = ../lib/libcpc.a

all: $(EXEC)

$(EXEC): $(SRC) 
	$(GCC) $(LDFLAGS) $(CFLAGS) -I$(INCLUDE) -o $@ $(SRC) $(CPCLIB) 

install: cpctool
	cp cpctool /usr/bin
	
clean:
	rm -f $(EXEC) $(OBJ)
	
	
