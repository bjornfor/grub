#ifndef XHCI_H
#define XHCI_H

#include <grub/types.h>
#include <grub/usb.h>
#include <grub/pci.h>

struct xhci; /* hidden */

enum xhci_speed {
  XHCI_SPEED_NONE = 0,
  XHCI_SPEED_LOW,
  XHCI_SPEED_FULL,
  XHCI_SPEED_HIGH,
  XHCI_SPEED_SUPER,
};

struct xhci *xhci_new(void);

int xhci_init (struct xhci *xhci, volatile void *mmio_base_addr, int seqno);

int xhci_setup_transfer (struct xhci *xhci);

int xhci_check_transfer (struct xhci *xhci);

int xhci_cancel_transfer (struct xhci *xhci);

int xhci_hubports (struct xhci *xhci);

int xhci_portstatus (struct xhci *xhci,
                                unsigned int port,
                                unsigned int enable);

enum xhci_speed xhci_detect_dev (struct xhci *xhci,
                                  int port, int *changed);

#endif /* XHCI_H */
