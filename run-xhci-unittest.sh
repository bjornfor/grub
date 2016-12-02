#!/bin/sh

# TODO: Use a real unittest framework and find a proper location for the source
# code for the tests.
cd grub-core/bus/usb/ && \
  make -f build_xhci_test.mk && \
  ./xhci_unittest; \
  echo $?
