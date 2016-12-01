# Expected to be invoked from ./grub-core/bus/usb.

CC ?= gcc
SRCS = xhci_unittest.c xhci.c xhci_io_mock.c
PROG = xhci_unittest

all: $(PROG)

$(PROG): $(SRCS)
	$(CC) $^ -o $(PROG)
