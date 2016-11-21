#!/usr/bin/env bash

# Based on ./README_BFO.txt.

cleanup1()
{
  echo "Cleaning up..."
  sudo kpartx -d disk.img
}

cleanup2()
{
  sudo umount mnt
  cleanup1
}

./build-and-install.sh quick || exit 1

dd if=/dev/zero of=disk.img bs=1M count=50 || exit 1
parted disk.img mklabel msdos || exit 1
parted disk.img mkpart primary 1M 100% || exit 1
sudo kpartx -s -a disk.img || exit 1  # creates loop0p1 on my system (loopN is dynamically allocated)

trap 'cleanup1' EXIT
sudo mkfs.ext3 /dev/mapper/loop0p1 || exit 1
mkdir -p mnt || exit 1
sudo mount /dev/mapper/loop0p1 mnt || exit 1
trap 'cleanup2' EXIT
sudo _install/sbin/grub-install --no-floppy --grub-mkdevicemap=<(printf '(hd0) /dev/loop0\n') --root-directory=$PWD/mnt /dev/loop0 || exit 1

# modify fs
grub_cfg="$PWD"/mnt/boot/grub/grub.cfg
echo "set debug=xhci,usb" | sudo tee "$grub_cfg" || exit 1
echo "serial" | sudo tee -a "$grub_cfg"
echo "terminal_input serial" | sudo tee -a "$grub_cfg"
echo "terminal_output serial" | sudo tee -a "$grub_cfg"
#echo "echo BFO: entering GRUB console" | sudo tee -a "$grub_cfg"
#echo "sleep 2" | sudo tee -a "$grub_cfg"
sudo cp _install/lib/grub/i386-pc/xhci.mod "$PWD"/mnt/boot/grub/i386-pc/ || exit 1
sudo cp _install/lib/grub/i386-pc/xhci.module "$PWD"/mnt/boot/grub/i386-pc/ || exit 1

sudo umount mnt
trap 'cleanup1' EXIT
sudo kpartx -d disk.img
trap - EXIT

if [ "$1" = "run" ]; then
  echo "Running QEMU (exit with 'Ctrl-a x'): qemu-system-x86_64 -drive file=disk.img,format=raw -nographic"
  qemu-system-x86_64 -drive file=disk.img,format=raw -nographic
else
  echo
  echo "Now run 'qemu-system-x86_64 -drive file=disk.img,format=raw -nographic'"
fi
