# Host-side pamodbus discovery tests: compile against the pamodbus and
# pamodbus-disco libraries (in-memory) plus the termios serial provider here.

CC      ?= gcc
STD     ?= c99
CFLAGS   = -std=$(STD) -Wall -Wextra -g
CFLAGS  += -Ishim                             # shadow briscits headers w/ libc
CFLAGS  += -I../include -I../src
CFLAGS  += -I../../pamodbus-disco/include -I../../pamodbus-disco/src
CFLAGS  += -I.

PA_SRCS  = ../src/pamodbus.c ../src/pdu.c \
           ../src/crc16.c ../src/framer_rtu.c ../src/framer_tcp.c
DISCO    = ../../pamodbus-disco/src/pa_disco_master.c \
           ../../pamodbus-disco/src/pa_disco_slave.c \
           ../../pamodbus-disco/src/pa_disco_list.c \
           ../../pamodbus-disco/src/pa_disco_register_map.c

SERIAL   = pa_serial.c shim/brisc_shim.c

all: master_disco slave_disco

master_disco: master_disco.c
	$(CC) $(CFLAGS) -o $@ master_disco.c $(SERIAL) $(DISCO) $(PA_SRCS)

slave_disco: slave_disco.c
	$(CC) $(CFLAGS) -o $@ slave_disco.c $(SERIAL) $(DISCO) $(PA_SRCS)

clean:
	rm -f master_disco slave_disco

.PHONY: all clean