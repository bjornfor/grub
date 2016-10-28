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
