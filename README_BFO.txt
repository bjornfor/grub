Build and install GRUB for PXE booting:

  ./autogen.sh
  ./configure --prefix=$PWD/_install
  make
  make install
  sudo ./_install/bin/grub-mknetdir --net-directory=/srv/tftp/

(/srv/tftp being the TFTP server root directory configured in e.g.
dnsmasq with the tftp-root= directive.)

Edit /etc/dnsmasq.conf with the following dhcp-boot= directive, just like
grub-mknetdir from above says:

  dhcp-boot=/srv/tftp/boot/grub/i386-pc/core.0


Installing GRUB to a disk image
-------------------------------

First ensure that GRUB is built with libdevmapper support. GRUB from Ubuntu
16.04 has that. If building from source, ensure that the libdevmapper-dev
package is installed, or else GRUB will fail to install to a loop device:

  sudo apt install libdevmapper-dev

Create a disk image with GRUB on it:

  dd if=/dev/zero of=disk.img bs=1M count=50
  parted disk.img mklabel msdos
  parted disk.img mkpart primary 1M 100%
  sudo kpartx -s -a disk.img   # creates loop0p1 on my system (loopN is dynamically allocated)
  sudo mkfs.ext3 /dev/mapper/loop0p1
  sudo mount /dev/mapper/loop0p1 mnt
  sudo grub-install --no-floppy --grub-mkdevicemap=<(printf '(hd0) /dev/loop0\n') --root-directory=$PWD/mnt /dev/loop0
  sudo umount mnt
  sudo losetup -d /dev/loop0
  sudo kpartx -d disk.img

Now run QEMU:

  qemu-system-x86_64 -drive file=disk.img,format=raw

TODO: xHCI controller in QEMU.
