#ifndef XHCI_H
#define XHCI_H

#include <stdint.h>

struct xhci; /* forward declaration (details are hidden) */

enum xhci_speed {
  XHCI_SPEED_NONE = 0,
  XHCI_SPEED_LOW,
  XHCI_SPEED_FULL,
  XHCI_SPEED_HIGH,
  XHCI_SPEED_SUPER,
};

/* Assuming the caller has made the PCI device a bus master (or else MMIO
 * access doesn't work) and that BARs has been assigned/programmed. PC firmware
 * should do the latter.
 *
 * mmio_base_addr is the address stored in BAR0.
 * seqno is an arbitrary integer used for identification purposes in logs.
 */
struct xhci *xhci_create (volatile void *mmio_base_addr, int seqno);

/* De-initialize controller and free memory */
void xhci_destroy (struct xhci *xhci);

int xhci_halt (struct xhci *xhci);

/* TODO: handle missing "transfer" parameter */
int xhci_setup_transfer (struct xhci *xhci);

/* TODO: handle missing "transfer" parameter */
int xhci_check_transfer (struct xhci *xhci);

/* TODO: handle missing "transfer" parameter */
int xhci_cancel_transfer (struct xhci *xhci);

/* Return the number of ports the root hub has */
int xhci_hubports (struct xhci *xhci);

int xhci_portstatus (struct xhci *xhci,
                                unsigned int port,
                                unsigned int enable);

enum xhci_speed xhci_detect_dev (struct xhci *xhci,
                                  int port, int *changed);

/* for debugging */
int xhci_nop(struct xhci *xhci);
int xhci_status(struct xhci *xhci, int verbose);

#endif /* XHCI_H */
