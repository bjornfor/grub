/* Glue code to connect xHCI driver to GRUB */

#include <grub/types.h> /* grub_uint32_t */
#include <grub/pci.h> /* ? */
#include <grub/usb.h> /* grub_usb_controller_dev */
#include <grub/mm.h> /* grub_zalloc */
#include <grub/time.h> /* grub_millisleep */
#include <grub/dl.h> /* GRUB_MOD_INIT */
#include <grub/disk.h> /* grub_stop_disk_firmware */
#include <grub/loader.h> /* grub_loader_register_preboot_hook */
#include <grub/env.h> /* grub_env_get */
#include <grub/command.h> /* struct grub_command_t */
#include <grub/extcmd.h> /* grub_register_extcmd */
#include <grub/lib/arg.h> /* struct grub_arg_option */

#include "xhci.h"
//#include "xhci_io.h"

GRUB_MOD_LICENSE ("GPLv3+");

static grub_extcmd_t cmd_xhci_status;

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

static void dbg(const char *fmt, ...)
{
  va_list ap;
  const char *debug = grub_env_get ("debug");

  va_start (ap, fmt);
  va_end (ap);

  if (debug && (grub_strword (debug, "all") || grub_strword (debug, "xhci")))
  {
    grub_vprintf (fmt, ap);
    va_end (ap);
  }
}

static const struct grub_arg_option cmd_options[] =
  {
    {"verbose", 'v', 0,
     N_("Be verbose."), 0, ARG_TYPE_NONE},
    {0, 0, 0, 0, 0, 0}
  };


static grub_err_t
do_cmd_xhci_status (grub_extcmd_context_t ctxt, int argc, char *argv[])
{
  int iter;
  struct xhci *xhci;
  int verbose = 0;
  (void)argc;
  (void)argv;

  struct grub_arg_list *state = ctxt->state;
 
  verbose = state[0].set;

  for (xhci = xhci_list_first(&iter); xhci; xhci = xhci_list_next(&iter))
    xhci_status(xhci, verbose);

  return 0;
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
  struct xhci *xhci;
  grub_uint32_t class_code;
  grub_uint32_t base;
  volatile grub_uint32_t *mmio_base_addr;
  grub_uint32_t base_h;
  grub_pci_address_t addr;

  /* Exit if not USB3.0 xHCI controller */
  addr = grub_pci_make_address (dev, GRUB_PCI_REG_CLASS);
  class_code = grub_pci_read (addr) >> 8;
  if (class_code != 0x0c0330)
    return 0;

  dbg ("xhci: controller at %d:%02x.%d, vendor:device %04x:%04x\n",
      dev.bus, dev.device, dev.function,
      (grub_le_to_cpu32(pciid) & 0xffff),
      (grub_le_to_cpu32(pciid) >> 16) & 0xffff);

  /* Determine xHCI MMIO registers base address */
  addr = grub_pci_make_address (dev, GRUB_PCI_REG_ADDRESS_REG0);
  base = grub_pci_read (addr);
  addr = grub_pci_make_address (dev, GRUB_PCI_REG_ADDRESS_REG1);
  base_h = grub_pci_read (addr);
  /* Stop if registers are mapped above 4G - GRUB does not currently
   * work with registers mapped above 4G */
  if (((base & GRUB_PCI_ADDR_MEM_TYPE_MASK) != GRUB_PCI_ADDR_MEM_TYPE_32)
      && (base_h != 0))
    {
      dbg ("xhci: registers above 4G are not supported\n");
      return 0;
    }
  base &= GRUB_PCI_ADDR_MEM_MASK;
  if (!base)
    {
      dbg ("xhci: BARs not programmed (broken PC firmware)\n");
      return 0;
    }

  /* Set bus master - needed for coreboot, VMware, broken BIOSes etc. or else
   * MMIO access doesn't work (no effect).
   */
  addr = grub_pci_make_address (dev, GRUB_PCI_REG_COMMAND);
  grub_pci_write_word(addr,
      GRUB_PCI_COMMAND_MEM_ENABLED
      | GRUB_PCI_COMMAND_BUS_MASTER
      | grub_pci_read_word(addr));

  /* PCI config space is 256 bytes */
  mmio_base_addr = grub_pci_device_map_range (dev, base, 0x100);

  xhci = xhci_create(mmio_base_addr, cur_xhci_id);
  if (!xhci)
    {
      dbg ("out of memory\n");
      return GRUB_USB_ERR_INTERNAL;
    }

  /* Initialise USB legacy support and claim ownership */
  //xhci_legacy_init(xhci);
  //xhci_legacy_claim(xhci);
  //xhci_extended_capabilities_foreach(xhci);

  //grub_millisleep(10000);

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

  dbg ("grub_xhci_fini_hw enter\n");

  /* We should disable all xHCI HW to prevent any DMA access etc. */
  for (xhci = xhci_list_first(&iter); xhci; xhci = xhci_list_next(&iter))
    xhci_destroy(xhci);

  return GRUB_ERR_NONE;
}

static grub_err_t
xhci_restore_hw (void)
{
  //struct xhci *xhci;
  //grub_uint32_t n_ports;
  //int i;

  dbg("grub_xhci_restore_hw enter\n");
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

  dbg ("xhci_iterate enter\n");
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
  //dbg ("[loading]\n");

  xhci_list_num_elems = 0;
  cur_xhci_id = 0;

  grub_stop_disk_firmware ();
  grub_boot_time ("Initing xHCI hardware");
  grub_pci_iterate (pci_iter, NULL);
  grub_boot_time ("Registering xHCI driver");
  grub_usb_controller_dev_register (&usb_controller_dev);
  grub_boot_time ("xHCI driver registered");
  dbg ("xHCI driver is registered, register preboot hook\n");
  grub_loader_register_preboot_hook (xhci_fini_hw, xhci_restore_hw,
				     GRUB_LOADER_PREBOOT_HOOK_PRIO_DISK);

  cmd_xhci_status =
    grub_register_extcmd ("xhci", do_cmd_xhci_status, 0,
        N_("[-v|--verbose]"),
        N_("Print xHCI driver status."),
        cmd_options);
  dbg ("GRUB_MOD_INIT completed\n");
}

GRUB_MOD_FINI (xhci)
{
  //dbg ("[unloading]\n");
  grub_unregister_extcmd (cmd_xhci_status);
  xhci_fini_hw (0);
  grub_usb_controller_dev_unregister (&usb_controller_dev);
}
