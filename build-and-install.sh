#!/usr/bin/env bash

tftpdir=/srv/tftp

if [ "$0" = "quick" ]; then
    echo
    echo "WARNING: Building only select modules: usb,usbtest,xhci"
    echo
    make CFLAGS+=-Wno-error=unused-function -C grub-core {usb,usbtest,xhci}{.mod,.module} && sudo cp -v grub-core/{usb,usbtest,xhci}{.mod,.module} "$tftpdir"/boot/grub/i386-pc/
    echo
    echo "WARNING: Built only select modules: usb,usbtest,xhci"
    echo
else
    # ./autogen.sh
    # ./configure --prefix=$PWD/_install
    make CFLAGS+=-Wno-error=unused-function install && sudo ./_install/bin/grub-mknetdir --net-directory="$tftpdir"/
fi
