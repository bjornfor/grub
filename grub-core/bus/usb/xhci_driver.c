/* Glue code to connect xHCI driver to GRUB */

#include <grub/types.h> /* grub_uint32_t */
#include <grub/pci.h> /* grub_pci_device_t */
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

#include "xhci/usb/xhci.h"
#include "xhci/usb/generic_hub.h"

GRUB_MOD_LICENSE ("GPLv3+");

#define XHCI_PCI_SBRN_REG  0x60

static grub_extcmd_t cmd_xhci_status;
struct grub_preboot *preboot_hook;

/* xHC's set device addresses. But GRUB also want to do that!
 * This adds an address mapping between GRUB and xHCI driver to keep everyone
 * happy and not confused about the addresses.
 * The index is the device address from GRUB, the value is the address assigned
 * by xHC. */
int usbdev_xhc_addr[128];
/* Hack: remember the current device between detect_dev and control_transfer,
 * so that we know which device to talk to.
 * Possible fix: create a "super" structure with two fields: hci_t and usbdev_t
 */
static usbdev_t *last_detected_dev = NULL;

/* List of found xHCI controllers */
static unsigned int cur_xhci_id;
static hci_t *xhci_list[16];
static size_t xhci_list_num_elems;

static hci_t *xhci_list_first(int *iter)
{
  if (xhci_list_num_elems == 0)
  {
    return NULL;
  }

  *iter = 0;
  return xhci_list[0];
}

//static hci_t *xhci_list_last(void)
//{
//  if (xhci_list_num_elems == 0)
//  {
//    return NULL;
//  }
//
//  return xhci_list[xhci_list_num_elems - 1];
//}

static int xhci_list_add(hci_t *xhci)
{
  if (xhci_list_num_elems >= (sizeof (xhci_list) / sizeof (xhci_list[0])))
  {
    return -1;
  }

  xhci_list[xhci_list_num_elems] = xhci;
  xhci_list_num_elems++;
  return 0;
}

static hci_t *xhci_list_next(int *iter)
{
  *iter += 1;
  if (*iter >= (int)xhci_list_num_elems)
    return NULL;

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
    {"verbose", 'v', 0, N_("Be verbose."), 0, ARG_TYPE_NONE},
    {"id", 'i', 0, N_("Operate on this instance [0..n]"), 0, ARG_TYPE_INT},
    {0, 0, 0, 0, 0, 0}
  };

static int get_int_arg (const struct grub_arg_list *state)
{
  int default_value = 0; /* if arg not set */
  return (state->set ? (int)grub_strtoul (state->arg, 0, 0) : default_value);
}

static grub_err_t
do_cmd_xhci_status (grub_extcmd_context_t ctxt, int argc, char *argv[])
{
  int iter;
  hci_t *xhci;
  enum op { CMD_NOP, STATUS } op;
  struct grub_arg_list *state = ctxt->state;
  int i = 0;
  int verbose     = state[i++].set;
  int id          = get_int_arg (&state[i++]);

  /* Get the operation */
  op = STATUS;
  for (i = 0; i < argc; i++)
  {
    if (grub_strcmp(argv[i], "cmd-nop") == 0)
    {
      op = CMD_NOP;
    }
  }

  /* Get the device */
  for (i = 0, xhci = xhci_list_first(&iter); xhci; xhci = xhci_list_next(&iter), i++)
  {
    if (i == id)
      break;
  }

  if (!xhci)
  {
    grub_printf("no such device (bad --id value: %d)\n", id);
    return GRUB_ERR_UNKNOWN_DEVICE;
  }

  /* Do the operation */
  switch (op)
  {
    case STATUS:
      (void)verbose;
      //xhci_status(xhci, verbose);
      break;

    case CMD_NOP:
      //xhci_nop(xhci);
      break;
  }

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
  hci_t *xhci;
  grub_uint32_t class_code;
  grub_uint32_t base;
  grub_uint32_t release;
  volatile grub_uint32_t *mmio_base_addr;
  grub_uint32_t base_h;
  grub_pci_address_t addr;

  /* Exit if not USB3.0 xHCI controller */
  addr = grub_pci_make_address (dev, GRUB_PCI_REG_CLASS);
  class_code = grub_pci_read (addr) >> 8;
  if (class_code != 0x0c0330)
    return 0;

  /* Check Serial Bus Release Number */
  addr = grub_pci_make_address (dev, XHCI_PCI_SBRN_REG);
  release = grub_pci_read_byte (addr);
  if (release != 0x30)
  {
    grub_dprintf ("xhci", "Wrong SBRN: 0x%0x (expected 0x%0x)\n",
        release, 0x30);
    return 0;
  }

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
  (void)mmio_base_addr;

  grub_uint32_t pciaddr = grub_pci_make_address (dev, 0);
  xhci = xhci_pci_init (pciaddr);
  if (!xhci)
    {
      dbg ("out of memory\n");
      return GRUB_USB_ERR_INTERNAL;
    }

  /* Build list of xHCI controllers */
  xhci_list_add(xhci);
  cur_xhci_id += 1;

  /* Hack */
  while (0)
  {
    usb_poll();
    grub_millisleep (50);
  }

  return 0;
}

static grub_err_t
xhci_fini_hw (int noreturn __attribute__ ((unused)))
{
  hci_t *hci;
  int iter;

  /* We should disable all xHCI HW to prevent any DMA access etc. */
  for (hci = xhci_list_first(&iter); hci; hci = xhci_list_next(&iter))
  {
    /* FIXME: this segfault + reboots machine */
    //grub_dprintf ("xhci", "shutting down controller %p\n", hci);
    //hci->shutdown(hci);
  }

  return GRUB_ERR_NONE;
}

static grub_err_t
xhci_restore_hw (void)
{
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
  hci_t *xhci;
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
  (void)dev;
  (void)transfer;
  int rc = -1;
#if 0
  hci_t *hci = (hci_t *) dev->data;
  usbdev_t *roothub = hci->devices[0];
  generic_hub_t *hub = GEN_HUB(roothub);
  int rc = -1;
  /* "u" prefix for coreboot usb stack */
  direction_t udir =
       (transfer->dir == GRUB_USB_TRANSFER_TYPE_IN) ? IN
    : ((transfer->dir == GRUB_USB_TRANSFER_TYPE_OUT) ? OUT
    : SETUP);
  usbdev_t *udev;
  int udrlen = 0;
  int udalen = 0;

  grub_dprintf("xhci", "%s\n", __func__);

  if (transfer->type == GRUB_USB_TRANSACTION_TYPE_CONTROL)
  {
    /* TODO: create udev */
    // transfer->addr is already a USB address/slot_id we can use, but we have
    // to put it into xhci stack somehow
    hci->control(udev, udir, udrlen, udevreq, udalen, usrc);
  }
  else if (transfer->type == GRUB_USB_TRANSACTION_TYPE_BULK)
  {
  }
  else
  {
    /* Unsupported */
    rc = -1;
  }
#endif

  return rc == 0 ? GRUB_USB_ERR_NONE : GRUB_USB_ERR_INTERNAL;
}

static grub_usb_err_t
check_transfer (grub_usb_controller_t dev,
                 grub_usb_transfer_t transfer, grub_size_t *actual)
{
  (void)dev;
  (void)transfer;
  (void)actual;

  grub_dprintf("xhci", "%s\n", __func__);
  /* We do everything in setup_transfer */
  return GRUB_USB_ERR_NONE;
}

static grub_usb_err_t
cancel_transfer (grub_usb_controller_t dev,
                 grub_usb_transfer_t transfer)
{
  (void)dev;
  (void)transfer;

  grub_dprintf("xhci", "%s\n", __func__);
  /* We do everything in setup_transfer */
  return GRUB_USB_ERR_NONE;
}

static grub_usb_err_t
control_transfer (grub_usb_device_t dev,
		      grub_uint8_t reqtype,
		      grub_uint8_t request,
		      grub_uint16_t value,
		      grub_uint16_t index,
		      grub_size_t size0, char *data_in)
{
  hci_t *hci = (hci_t *) dev->controller.data;
  int ret = 0;
  int slot_id; /* the xHC slot used for addressing the device (assigned by xHC) */
  dev_req_t dr;
  direction_t dir = (reqtype & 128) ? IN : OUT;

  dr.bmRequestType = reqtype;
  dr.bRequest = request;
  dr.wValue = value;
  dr.wIndex = index;
  dr.wLength = size0;

  /* xHCI controllers are made so that *they* set the device address. This
   * conflicts with the GRUB USB driver which assumes it can set the device
   * address by doing a control message.
   * We work around that by (1) using a temporary global variable,
   * last_detected_dev, until GRUB tells us the address for the newly connected
   * device. Then we store the GRUB -> xHC address mapping for later.
   */
  if (dev->initialized)
  {
    slot_id = usbdev_xhc_addr[dev->addr];
  }
  else
  {
    slot_id = last_detected_dev->address;
  }

  if (request == GRUB_USB_REQ_SET_ADDRESS)
  {
    /* Setting the address has already been handled by xHC */
    grub_dprintf("xhci", "creating address map from GRUB to xHC: %d -> %d\n",
        value, slot_id);
    usbdev_xhc_addr[value] = slot_id;
  }
  else
  {
    ret = hci->control(hci->devices[slot_id], dir, sizeof(dr), &dr, size0,
        (unsigned char*)data_in);
  }
  return ret >= 0 ? GRUB_USB_ERR_NONE : GRUB_USB_ERR_INTERNAL;
}

static int hubports (grub_usb_controller_t dev)
{
  hci_t *hci = (hci_t *) dev->data;
  usbdev_t *roothub = hci->devices[0];
  generic_hub_t *hub = GEN_HUB(roothub);

  grub_dprintf("xhci", "%s: num_ports=%d\n", __func__, hub->num_ports);
  return hub->num_ports;
}

static grub_usb_err_t
portstatus (grub_usb_controller_t dev,
    unsigned int port, unsigned int enable)
{
  //hci_t *hci = (hci_t *) dev->data;
  int rc;

  grub_dprintf("xhci", "%s: port=%d enable=%d\n", __func__, port, enable);
  /* Enabling/disabled is handled in detect_dev (easier to match with Coreboot
   * xHCI driver) */
  rc = 0;
  return rc == 0 ? GRUB_USB_ERR_NONE : GRUB_USB_ERR_INTERNAL;
}

static grub_usb_speed_t
detect_dev (grub_usb_controller_t dev, int port, int *changed)
{
  int status_changed;
  int connected;
  usb_speed speed;
  grub_usb_speed_t grub_speed = GRUB_USB_SPEED_NONE;
  hci_t *hci = (hci_t *) dev->data;
  usbdev_t *roothub = hci->devices[0];
  generic_hub_t *hub = GEN_HUB(roothub);

  status_changed = hub->ops->port_status_changed(roothub, port);
  connected = hub->ops->port_connected(roothub, port);
  if (status_changed)
  {
    *changed = 1;

    if (connected)
    {
      /* GRUB (usually) handles debouncing (stable power), but here we let the
       * xHCI driver do that itself. Also, let it set the device address,
       * something GRUB typically does (e.g. for EHCI). XHCI controllers are
       * written that way; *they* set the device address and the host software
       * reads the address back from the controller HW.
       *
       * Because of how the Coreboot xHCI driver is written, and that we want
       * to change it as little as possible (maintainability), it seems
       * (way) simpler this way. We have to pay attention to filter out control
       * messages of SET_ADDRESS type from GRUB. We have to go just low enough
       * into the Coreboot driver so that we can register the device without it
       * getting to the point where it complains about missing driver support
       * for USB class XYZ.
       */
      if (generic_hub_debounce(roothub, port) < 0)
        return GRUB_USB_SPEED_NONE;

      hub->ops->reset_port(roothub, port);
      speed = hub->ops->port_speed(roothub, port);
      /* set_address() "bypasses" GRUB (it sends SET_ADDRESS and GET_DESCRIPTOR
       * control messages). Those extra messages do no harm. */
      usbdev_t *udev = hci->set_address(hci, speed, port, roothub->address);
      last_detected_dev = udev;
      //grub_dprintf("xhci", "ep[0].maxpacketsize: %d\n", udev->endpoints[0].maxpacketsize);
    }
  }

  if (connected)
  {
    speed = hub->ops->port_speed(roothub, port);
    switch (speed)
    {
      case LOW_SPEED:
        grub_speed = GRUB_USB_SPEED_LOW;
        break;

      case FULL_SPEED:
        grub_speed = GRUB_USB_SPEED_FULL;
        break;

      case HIGH_SPEED:
        grub_speed = GRUB_USB_SPEED_HIGH;
        break;

      case SUPER_SPEED:
        /* unsupported, so disable it */
        grub_printf("warning: USB 3.0 devices are unsupported\n");
        grub_speed = GRUB_USB_SPEED_NONE;
        break;
    }
  }

  if (status_changed)
  {
    grub_dprintf("xhci", "%s: port=%d *changed=%d connected=%d speed=%d\n",
        __func__, port, *changed, connected, grub_speed);
  }

  return grub_speed;
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
  .control_transfer = control_transfer,
  .hubports = hubports,
  .portstatus = portstatus,
  .detect_dev = detect_dev,

  /* estimated max. count of TDs for one bulk transfer */
  .max_bulk_tds = 16, //GRUB_EHCI_N_TD * 3 / 4
};

GRUB_MOD_INIT (xhci)
{
  //dbg ("[loading]\n");
  int i;

  xhci_list_num_elems = 0;
  cur_xhci_id = 0;

  for (i = 0; i < sizeof(usbdev_xhc_addr) / sizeof(usbdev_xhc_addr[0]); i++)
  {
    usbdev_xhc_addr[i] = -1;
  }

  grub_stop_disk_firmware ();
  grub_boot_time ("Initing xHCI hardware");
  grub_pci_iterate (pci_iter, NULL);
  grub_boot_time ("Registering xHCI driver");
  grub_usb_controller_dev_register (&usb_controller_dev);
  grub_boot_time ("xHCI driver registered");
  //dbg ("xHCI driver is registered, register preboot hook\n");
  preboot_hook = grub_loader_register_preboot_hook (xhci_fini_hw, xhci_restore_hw,
				     GRUB_LOADER_PREBOOT_HOOK_PRIO_DISK);

  cmd_xhci_status =
    grub_register_extcmd ("xhci", do_cmd_xhci_status, 0,
        N_("[-v|--verbose] [-i|--id N] [cmd-nop]"),
        N_("Print xHCI driver status."),
        cmd_options);
  //dbg ("GRUB_MOD_INIT completed\n");
}

GRUB_MOD_FINI (xhci)
{
  //dbg ("[unloading]\n");
  grub_unregister_extcmd (cmd_xhci_status);
  grub_usb_controller_dev_unregister (&usb_controller_dev);
  grub_loader_unregister_preboot_hook (preboot_hook);
  xhci_fini_hw (0);
}
