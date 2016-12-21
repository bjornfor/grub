/* xHCI driver for GRUB, adaptation layer for coreboot xHCI driver */
/*
 *  GRUB  --  GRand Unified Bootloader
 *  Copyright (C) 2016 Hiddn Security AS
 *
 *  GRUB is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  GRUB is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with GRUB.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <grub/dl.h>
#include <grub/mm.h>
#include <grub/usb.h>
#include <grub/usbtrans.h>
#include <grub/misc.h>
#include <grub/pci.h>
#include <grub/cpu/pci.h>
#include <grub/cpu/io.h>
#include <grub/time.h>
#include <grub/loader.h>
#include <grub/disk.h>
#include <grub/cache.h>

#include "xhci.h"

GRUB_MOD_LICENSE ("GPLv3+");

/* This simple GRUB implementation of xHCI driver:
 *      - assumes no IRQ
 *      - is not supporting isochronous transfers (iTD, siTD)
 *      - is not supporting interrupt transfers
 */

#define GRUB_XHCI_PCI_SBRN_REG  0x60

static void *xhci_list[16];
static int xhci_list_num_elems;

/* PCI iteration function... */
static int
grub_xhci_pci_iter (grub_pci_device_t dev, grub_pci_id_t pciid,
		    void *data __attribute__ ((unused)))
{
  (void)pciid;
  grub_uint8_t release;
  grub_uint32_t class_code;
  grub_uint32_t interf;
  grub_uint32_t subclass;
  grub_uint32_t class;
  grub_uint32_t base, base_h;
  grub_pci_address_t addr;
  xhci_t *xhci;

  grub_dprintf ("xhci", "xHCI grub_xhci_pci_iter: begin\n");

  addr = grub_pci_make_address (dev, GRUB_PCI_REG_CLASS);
  class_code = grub_pci_read (addr) >> 8;
  interf = class_code & 0xFF;
  subclass = (class_code >> 8) & 0xFF;
  class = class_code >> 16;

  /* If this is not an xHCI controller, just return.  */
  if (class != 0x0c || subclass != 0x03 || interf != 0x30)
    return 0;

  grub_dprintf ("xhci", "xHCI grub_xhci_pci_iter: class OK\n");

  /* Check Serial Bus Release Number */
  addr = grub_pci_make_address (dev, GRUB_XHCI_PCI_SBRN_REG);
  release = grub_pci_read_byte (addr);
  if (release != 0x30)
    {
      grub_dprintf ("xhci", "xHCI grub_xhci_pci_iter: Wrong SBRN: %0x\n",
    		release);
      return 0;
    }
  grub_dprintf ("xhci", "xHCI grub_xhci_pci_iter: bus rev. num. OK\n");
 
  /* Determine xHCI registers base address. */
  addr = grub_pci_make_address (dev, GRUB_PCI_REG_ADDRESS_REG0);
  base = grub_pci_read (addr);
  addr = grub_pci_make_address (dev, GRUB_PCI_REG_ADDRESS_REG1);
  base_h = grub_pci_read (addr);
  /* Stop if registers are mapped above 4G - GRUB does not currently
   * work with registers mapped above 4G */
  if (((base & GRUB_PCI_ADDR_MEM_TYPE_MASK) != GRUB_PCI_ADDR_MEM_TYPE_32)
      && (base_h != 0))
    {
      grub_dprintf ("xhci",
    		"xHCI grub_xhci_pci_iter: registers above 4G are not supported\n");
      return 0;
    }
  base &= GRUB_PCI_ADDR_MEM_MASK;
  if (!base)
    {
      grub_dprintf ("xhci",
    		"xHCI: xHCI is not mapped\n");
      return 0;
    }

  /* Set bus master - needed for coreboot, VMware, broken BIOSes etc. */
  addr = grub_pci_make_address (dev, GRUB_PCI_REG_COMMAND);
  grub_pci_write_word(addr,
    		  GRUB_PCI_COMMAND_MEM_ENABLED
    		  | GRUB_PCI_COMMAND_BUS_MASTER
    		  | grub_pci_read_word(addr));
  
  grub_dprintf ("xhci", "xHCI grub_xhci_pci_iter: 32-bit xHCI OK\n");

  xhci = xhci_init(base);
  if (!xhci)
    return -1; /* OOM */

  xhci_list[xhci_list_num_elems] = xhci;
  xhci_list_num_elems++;

  return 0;
}

static int
grub_xhci_iterate (grub_usb_controller_iterate_hook_t hook, void *hook_data)
{
  (void)hook;
  (void)hook_data;
  struct grub_xhci *x;
  struct grub_usb_controller dev;
  int i;

  for (i = 0, x = xhci_list[i]; x; i++)
    {
      dev.data = x;
      if (hook (&dev, hook_data))
	return 1;
    }

  return 0;
}

static grub_usb_err_t
grub_xhci_setup_transfer (grub_usb_controller_t dev,
			  grub_usb_transfer_t transfer)
{
  (void)dev;
  (void)transfer;
  //struct grub_xhci *e = (struct grub_xhci *) dev->data;
  //int i;
  //struct grub_xhci_transfer_controller_data *cdata;
  //grub_uint32_t status;

  return GRUB_USB_ERR_NONE;
}

static grub_usb_err_t
grub_xhci_check_transfer (grub_usb_controller_t dev,
			  grub_usb_transfer_t transfer, grub_size_t * actual)
{
  (void)dev;
  (void)transfer;
  (void)actual;
  //struct grub_xhci *e = dev->data;
  //struct grub_xhci_transfer_controller_data *cdata =
  //  transfer->controller_data;

//  grub_dprintf ("xhci",
//		"check_transfer: EHCI STATUS=%08x, cdata=%p, qh=%p\n",
//		grub_xhci_oper_read32 (e, GRUB_EHCI_STATUS),
//		cdata, cdata->qh_virt);
  return GRUB_USB_ERR_WAIT;
}

static grub_usb_err_t
grub_xhci_cancel_transfer (grub_usb_controller_t dev,
			   grub_usb_transfer_t transfer)
{
  (void)dev;
  (void)transfer;
  //struct grub_xhci *e = dev->data;
  //struct grub_xhci_transfer_controller_data *cdata =
  //  transfer->controller_data;

  grub_dprintf ("xhci", "cancel_transfer: begin\n");

  return GRUB_USB_ERR_NONE;
}

static int
grub_xhci_hubports (grub_usb_controller_t dev)
{
  xhci_t *xhci = dev->data;
  grub_uint32_t portinfo = 0;

  portinfo = xhci_num_ports(xhci);
  //grub_dprintf ("xhci", "root hub ports=%d\n", portinfo);
  return portinfo;
}

static grub_usb_err_t
grub_xhci_portstatus (grub_usb_controller_t dev,
		      unsigned int port, unsigned int enable)
{
  struct grub_xhci *e = (struct grub_xhci *) dev->data;
  (void)e;
  (void)port;
  (void)enable;

//  grub_dprintf ("xhci", "portstatus: EHCI STATUS: %08x\n",
//		grub_xhci_oper_read32 (e, GRUB_EHCI_STATUS));
  return GRUB_USB_ERR_NONE;
}

static grub_usb_speed_t
grub_xhci_detect_dev (grub_usb_controller_t dev, int port, int *changed)
{
  struct grub_xhci *e = (struct grub_xhci *) dev->data;
  (void)e;
  (void)port;
  (void)changed;

  return GRUB_USB_SPEED_NONE;
}

static void
grub_xhci_inithw (void)
{
  grub_pci_iterate (grub_xhci_pci_iter, NULL);
}

static grub_err_t
grub_xhci_restore_hw (void)
{

  /* We should re-enable all EHCI HW similarly as on inithw */
  //for (e = ehci; e; e = e->next)

  return GRUB_ERR_NONE;
}

static grub_err_t
grub_xhci_fini_hw (int noreturn __attribute__ ((unused)))
{
//  struct grub_xhci *e;
//
//  /* We should disable all xHCI HW to prevent any DMA access etc. */
//  for (e = xhci; e; e = e->next)
//    {
//      /* Disable both lists */
//      grub_xhci_oper_write32 (e, GRUB_XHCI_COMMAND,
//        ~(GRUB_XHCI_CMD_AS_ENABL | GRUB_XHCI_CMD_PS_ENABL)
//        & grub_xhci_oper_read32 (e, GRUB_XHCI_COMMAND));
//
//      /* Check if xHCI is halted and halt it if not */
//      grub_xhci_halt (e);
//
//      /* Reset xHCI */
//      grub_xhci_reset (e);
//    }

  return GRUB_ERR_NONE;
}

static struct grub_usb_controller_dev usb_controller = {
  .name = "xhci",
  .iterate = grub_xhci_iterate,
  .setup_transfer = grub_xhci_setup_transfer,
  .check_transfer = grub_xhci_check_transfer,
  .cancel_transfer = grub_xhci_cancel_transfer,
  .hubports = grub_xhci_hubports,
  .portstatus = grub_xhci_portstatus,
  .detect_dev = grub_xhci_detect_dev,
  /* estimated max. count of TDs for one bulk transfer */
  //.max_bulk_tds = GRUB_EHCI_N_TD * 3 / 4 
};

GRUB_MOD_INIT (ehci)
{
  //COMPILE_TIME_ASSERT (sizeof (struct grub_xhci_td) == 64);
  //COMPILE_TIME_ASSERT (sizeof (struct grub_xhci_qh) == 96);

  grub_stop_disk_firmware ();

  grub_boot_time ("Initing xHCI hardware");
  grub_xhci_inithw ();
  grub_boot_time ("Registering xHCI driver");
  grub_usb_controller_dev_register (&usb_controller);
  grub_boot_time ("xHCI driver registered");
  grub_loader_register_preboot_hook (grub_xhci_fini_hw, grub_xhci_restore_hw,
				     GRUB_LOADER_PREBOOT_HOOK_PRIO_DISK);
}

GRUB_MOD_FINI (xhci)
{
  grub_xhci_fini_hw (0);
  grub_usb_controller_dev_unregister (&usb_controller);
}
