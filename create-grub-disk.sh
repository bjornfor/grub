#!/usr/bin/env bash

# Based on ./README_BFO.txt.

./build-and-install.sh quick

dd if=/dev/zero of=disk.img bs=1M count=50
parted disk.img mklabel msdos
parted disk.img mkpart primary 1M 100%
sudo kpartx -s -a disk.img   # creates loop0p1 on my system (loopN is dynamically allocated)

sudo mkfs.ext3 /dev/mapper/loop0p1
mkdir -p mnt
sudo mount /dev/mapper/loop0p1 mnt
sudo _install/sbin/grub-install --no-floppy --grub-mkdevicemap=<(printf '(hd0) /dev/loop0\n') --root-directory=$PWD/mnt /dev/loop0

# modify fs
grub_cfg="$PWD"/mnt/boot/grub/grub.cfg
echo "set debug=xhci,usb" | sudo tee "$grub_cfg"
echo "serial" | sudo tee -a "$grub_cfg"
echo "terminal_input serial" | sudo tee -a "$grub_cfg"
echo "terminal_output serial" | sudo tee -a "$grub_cfg"
#echo "echo BFO: entering GRUB console" | sudo tee -a "$grub_cfg"
#echo "sleep 2" | sudo tee -a "$grub_cfg"
sudo cp _install/lib/grub/i386-pc/xhci.mod "$PWD"/mnt/boot/grub/i386-pc/
sudo cp _install/lib/grub/i386-pc/xhci.module "$PWD"/mnt/boot/grub/i386-pc/

sudo umount mnt
sudo kpartx -d disk.img

if [ "$1" = "run" ]; then
  echo "Running QEMU (exit with 'Ctrl-a x'): qemu-system-x86_64 -drive file=disk.img,format=raw -nographic"
  qemu-system-x86_64 -drive file=disk.img,format=raw -nographic
else
  echo
  echo "Now run 'qemu-system-x86_64 -drive file=disk.img,format=raw -nographic'"
fi
