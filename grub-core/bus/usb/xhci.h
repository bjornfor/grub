#ifndef XHCI_H
#define XHCI_H

#include <grub/types.h>
#include <grub/usb.h>
#include <grub/pci.h>

struct xhci; /* hidden */

void xhci_driver_init(void);

struct xhci *xhci_new(void);

struct xhci *xhci_list_first(int *iter);
struct xhci *xhci_list_last(void);
int xhci_list_add(struct xhci *xhci);
struct xhci *xhci_list_next(int *iter);

int xhci_init (struct xhci *xhci, volatile void *mmio_base_addr,
    grub_pci_device_t dev, int seqno);

/* PCI iteration function, to be passed to grub_pci_iterate.
 *
 * grub_pci_iterate will invoke this function for each PCI device that exists
 * in the system. This function checks if the device is an xHC and initializes
 * it. Return 0 to continue iterating over devices, != 0 to abort.
 */
int xhci_pci_iter (grub_pci_device_t dev, grub_pci_id_t pciid, void *data);

grub_usb_err_t xhci_setup_transfer (grub_usb_controller_t dev,
                                    grub_usb_transfer_t transfer);

grub_usb_err_t xhci_check_transfer (grub_usb_controller_t dev,
                                    grub_usb_transfer_t transfer,
                                    grub_size_t *actual);

grub_usb_err_t xhci_cancel_transfer (grub_usb_controller_t dev,
                                     grub_usb_transfer_t transfer);

int xhci_hubports (grub_usb_controller_t dev);

grub_usb_err_t xhci_portstatus (grub_usb_controller_t dev,
                                unsigned int port,
                                unsigned int enable);

grub_usb_speed_t xhci_detect_dev (grub_usb_controller_t dev,
                                  int port, int *changed);

#endif /* XHCI_H */
