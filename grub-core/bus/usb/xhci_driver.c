/* Glue code to connect xHCI driver to GRUB */

#include <grub/types.h> /* grub_uint32_t */
#include <grub/pci.h> /* ? */
#include <grub/usb.h> /* grub_usb_controller_dev */
#include <grub/mm.h> /* grub_zalloc */
#include <grub/time.h> /* grub_millisleep */
#include <grub/dl.h> /* GRUB_MOD_INIT */
#include <grub/disk.h> /* grub_stop_disk_firmware */
#include <grub/loader.h> /* grub_loader_register_preboot_hook */

#include "xhci.h"
#include "xhci_io.h"
#include "xhci_debug.h" // FIXME: strictly unneeded, remove before submitting upstream

static unsigned int cur_xhci_id;
static struct xhci *xhci_list[16];
static int xhci_list_num_elems;

static struct xhci *xhci_list_first(int *iter)
{
  if (xhci_list_num_elems == 0)
  {
    return NULL;
  }

  *iter = 0;
  return xhci_list[0];
}

//static struct xhci *xhci_list_last(void)
//{
//  if (xhci_list_num_elems == 0)
//  {
//    return NULL;
//  }
//
//  return xhci_list[xhci_list_num_elems - 1];
//}

static int xhci_list_add(struct xhci *xhci)
{
  if (xhci_list_num_elems >= (int)(sizeof (xhci_list) / sizeof (xhci_list[0])))
  {
    return -1;
  }

  xhci_list[xhci_list_num_elems] = xhci;
  xhci_list_num_elems++;
  return 0;
}

static struct xhci *xhci_list_next(int *iter)
{
  if (*iter >= xhci_list_num_elems - 1)
  {
    return NULL;
  }

  *iter += 1;
  return xhci_list[*iter];
}


/* PCI iteration function, to be passed to grub_pci_iterate.
 *
 * grub_pci_iterate will invoke this function for each PCI device that exists
 * in the system. This function checks if the device is an xHC and initializes
 * it. Return 0 to continue iterating over devices, != 0 to abort.
 */
static int pci_iter (grub_pci_device_t dev, grub_pci_id_t pciid, void *data)
{
  (void)data;
  int err;
  int ac64;
  struct xhci *xhci;
  grub_uint32_t class_code;
  grub_uint32_t base;
  volatile grub_uint32_t *mmio_base_addr;
  grub_uint32_t base_h;

  /* Exit if not USB3.0 xHCI controller */
  class_code = pci_config_read32 (dev, GRUB_PCI_REG_CLASS) >> 8;
  if (class_code != 0x0c0330)
    return 0;

  xhci_dbg ("XHCI controller at %d:%02x.%d, vendor:device %04x:%04x\n",
      dev.bus, dev.device, dev.function,
      (grub_le_to_cpu32(pciid) & 0xffff),
      (grub_le_to_cpu32(pciid) >> 16) & 0xffff);

  /* Determine xHCI MMIO registers base address */
  base = pci_config_read32 (dev, GRUB_PCI_REG_ADDRESS_REG0);
  base_h = pci_config_read32 (dev, GRUB_PCI_REG_ADDRESS_REG1);
  /* Stop if registers are mapped above 4G - GRUB does not currently
   * work with registers mapped above 4G */
  if (((base & GRUB_PCI_ADDR_MEM_TYPE_MASK) != GRUB_PCI_ADDR_MEM_TYPE_32)
      && (base_h != 0))
    {
      xhci_err ("registers above 4G are not supported\n");
      return 0;
    }
  base &= GRUB_PCI_ADDR_MEM_MASK;
  if (!base)
    {
      xhci_err ("xHCI BARs not programmed (broken PC firmware)\n");
      return 0;
    }

  /* Set bus master - needed for coreboot, VMware, broken BIOSes etc. or else
   * MMIO access doesn't work (no effect).
   */
  pci_config_write32(dev, GRUB_PCI_REG_COMMAND,
      pci_config_read32(dev, GRUB_PCI_REG_COMMAND)
      | GRUB_PCI_COMMAND_MEM_ENABLED
      | GRUB_PCI_COMMAND_BUS_MASTER);

  mmio_base_addr = grub_pci_device_map_range (dev,
      (base & XHCI_ADDR_MEM_MASK),
      0x100); /* PCI config space is 256 bytes */

  xhci = xhci_new();
  if (!xhci)
    {
      xhci_err ("out of memory\n");
      return GRUB_USB_ERR_INTERNAL;
    }

  err = xhci_init (xhci, mmio_base_addr, cur_xhci_id);
  if (err)
  {
    grub_free(xhci);
    return err;
  }
  xhci_dbg("XHCI-%s: REGS: cap=0x%08x oper=0x%08x run=0x%08x db=0x%08x\n",
      xhci->name, xhci->cap_regs, xhci->oper_regs, xhci->run_regs, xhci->db_regs);
  ac64 = mmio_read_bits(&xhci->cap_regs->hccparams1, XHCI_CAP_HCCPARAMS1_AC64);
  xhci_dbg("XHCI-%s: SBRN=%02x scratch_bufs=%d (arr @ 0x%08x) pagesize=%d AC64=%d\n",
      xhci->name, xhci->sbrn, xhci->num_scratch_bufs, xhci->scratchpad_arr,
      xhci->pagesize, ac64);

  /* Initialise USB legacy support and claim ownership */
  //xhci_legacy_init(xhci);
  //xhci_legacy_claim(xhci);
  //xhci_extended_capabilities_foreach(xhci);

  grub_millisleep(10000);

  /* Build list of xHCI controllers */
  xhci_list_add(xhci);
  cur_xhci_id += 1;

  return 0;
}

static grub_err_t
xhci_fini_hw (int noreturn __attribute__ ((unused)))
{
  struct xhci *xhci;
  int iter;

  xhci_trace ("grub_xhci_fini_hw enter\n");

  /* We should disable all xHCI HW to prevent any DMA access etc. */
  for (xhci = xhci_list_first(&iter); xhci; xhci = xhci_list_next(&iter))
    {
      /* Disable both lists */
      //grub_xhci_oper_write32 (xhci, GRUB_XHCI_OPER_USBCMD,
       // ~(GRUB_XHCI_CMD_AS_ENABL | GRUB_XHCI_CMD_PS_ENABL)
        //& grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD));

      /* Check if xHCI is halted and halt it if not */
      //grub_xhci_halt (xhci);

      /* Reset xHCI */
      //grub_xhci_reset (xhci);
    }

  return GRUB_ERR_NONE;
}

static grub_err_t
xhci_restore_hw (void)
{
  //struct xhci *xhci;
  //grub_uint32_t n_ports;
  //int i;

  xhci_trace("grub_xhci_restore_hw enter\n");
  /* We should re-enable all xHCI HW similarly as on inithw */
//  for (xhci = xhci_list; xhci; xhci = xhci->next)
//    {
//      /* Check if xHCI is halted and halt it if not */
//      if (xhci_halt (xhci) != GRUB_USB_ERR_NONE)
//	grub_error (GRUB_ERR_TIMEOUT, "restore_hw: xHCI halt timeout");
//
//      /* Reset xHCI */
//      if (xhci_reset (xhci) != GRUB_USB_ERR_NONE)
//	grub_error (GRUB_ERR_TIMEOUT, "restore_hw: xHCI reset timeout");
//
//      /* Setup some xHCI registers and enable xHCI */
////      grub_xhci_oper_write32 (xhci, GRUB_XHCI_OPER_USBCMD,
////			      XHCI_USBCMD_RUNSTOP |
////			      grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD));
//
//      /* Now should be possible to power-up and enumerate ports etc. */
//	  /* Power on all ports */
//    }

  return GRUB_ERR_NONE;
}

static int
xhci_iterate (grub_usb_controller_iterate_hook_t hook, void *hook_data)
{
  struct xhci *xhci;
  struct grub_usb_controller dev;
  (void)dev;
  int iter;

  xhci_trace ("xhci_iterate enter\n");
  for (xhci = xhci_list_first(&iter); xhci; xhci = xhci_list_next(&iter))
    {
      dev.data = xhci;
      if (hook (&dev, hook_data))
          return 1;
    }

  return 0;
}

static grub_usb_err_t
setup_transfer (grub_usb_controller_t dev,
                 grub_usb_transfer_t transfer)
{
  struct xhci *xhci = dev->data;
  int rc;
  (void)transfer;

  rc = xhci_setup_transfer(xhci);
  return rc == 0 ? GRUB_USB_ERR_NONE : GRUB_USB_ERR_INTERNAL;
}

static grub_usb_err_t
check_transfer (grub_usb_controller_t dev,
                 grub_usb_transfer_t transfer, grub_size_t *actual)
{
  struct xhci *xhci = dev->data;
  int rc;
  (void)transfer;
  (void)actual;

  rc = xhci_check_transfer(xhci);
  return rc == 0 ? GRUB_USB_ERR_NONE : GRUB_USB_ERR_INTERNAL;
}

static grub_usb_err_t
cancel_transfer (grub_usb_controller_t dev,
                 grub_usb_transfer_t transfer)
{
  struct xhci *xhci = dev->data;
  int rc;
  (void)transfer;

  /* TODO: convert "transfer" to something non-GRUB and pass it on */
  rc = xhci_cancel_transfer(xhci);
  return rc == 0 ? GRUB_USB_ERR_NONE : GRUB_USB_ERR_INTERNAL;
}

static int hubports (grub_usb_controller_t dev)
{
  struct xhci *xhci = (struct xhci *) dev->data;

  return xhci_hubports(xhci);
}

static grub_usb_err_t
portstatus (grub_usb_controller_t dev,
		      unsigned int port, unsigned int enable)
{
  int rc;
  struct xhci *xhci = (struct xhci *) dev->data;

  rc = xhci_portstatus(xhci, port, enable);
  return rc == 0 ? GRUB_USB_ERR_NONE : GRUB_USB_ERR_INTERNAL;
}

static grub_usb_speed_t
detect_dev (grub_usb_controller_t dev, int port, int *changed)
{
  struct xhci *xhci = (struct xhci *) dev->data;
  enum xhci_speed speed;

  speed = xhci_detect_dev(xhci, port, changed);

  switch (speed)
  {
    case XHCI_SPEED_NONE:
      return GRUB_USB_SPEED_NONE;

    case XHCI_SPEED_LOW:
      return GRUB_USB_SPEED_LOW;

    case XHCI_SPEED_FULL:
      return GRUB_USB_SPEED_FULL;

    case XHCI_SPEED_HIGH:
      return GRUB_USB_SPEED_HIGH;

    case XHCI_SPEED_SUPER:
      return GRUB_USB_SPEED_SUPER;
  }

  return GRUB_USB_SPEED_NONE;
}

static struct grub_usb_controller_dev usb_controller_dev = {
  .name = "xhci",
  .iterate = xhci_iterate,
  .setup_transfer = setup_transfer, /* give data to HW, let it go */

  .check_transfer = check_transfer, /* check if HW has completed transfer,
                                          * polled by USB framework (see
                                          * usbtrans.c)
                                          */

  .cancel_transfer = cancel_transfer, /* called if/when check_transfer has
                                            * failed over a period of time
                                            */
  .hubports = hubports,
  .portstatus = portstatus,
  .detect_dev = detect_dev,

  /* estimated max. count of TDs for one bulk transfer */
  .max_bulk_tds = 16, //GRUB_EHCI_N_TD * 3 / 4
};

GRUB_MOD_INIT (xhci)
{
  /* Sanity check register addresses.
   * No limits.h or CHAR_BIT available, use GRUB_CHAR_BIT.
   */
  COMPILE_TIME_ASSERT(GRUB_CHAR_BIT == 8);
  COMPILE_TIME_ASSERT(OFFSETOF(struct xhci_cap_regs, hccparams2) == 0x1c);
  COMPILE_TIME_ASSERT(OFFSETOF(struct xhci_oper_regs, config) == 0x38);

  //xhci_trace ("[loading]\n");

  xhci_list_num_elems = 0;
  cur_xhci_id = 0;

  grub_stop_disk_firmware ();
  grub_boot_time ("Initing xHCI hardware");
  grub_pci_iterate (pci_iter, NULL);
  grub_boot_time ("Registering xHCI driver");
  grub_usb_controller_dev_register (&usb_controller_dev);
  grub_boot_time ("xHCI driver registered");
  xhci_trace ("xHCI driver is registered, register preboot hook\n");
  grub_loader_register_preboot_hook (xhci_fini_hw, xhci_restore_hw,
				     GRUB_LOADER_PREBOOT_HOOK_PRIO_DISK);
  xhci_trace ("GRUB_MOD_INIT completed\n");
}

GRUB_MOD_FINI (xhci)
{
  //xhci_trace ("[unloading]\n");
  xhci_fini_hw (0);
  grub_usb_controller_dev_unregister (&usb_controller_dev);
}
