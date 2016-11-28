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
//#include "xhci_io.h"
#include "xhci_debug.h" // FIXME: strictly unneeded, remove before submitting upstream

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


static struct grub_usb_controller_dev usb_controller_dev = {
  .name = "xhci",
  .iterate = xhci_iterate,
  .setup_transfer = xhci_setup_transfer, /* give data to HW, let it go */

  .check_transfer = xhci_check_transfer, /* check if HW has completed transfer,
                                          * polled by USB framework (see
                                          * usbtrans.c)
                                          */

  .cancel_transfer = xhci_cancel_transfer, /* called if/when check_transfer has
                                            * failed over a period of time
                                            */
  .hubports = xhci_hubports,
  .portstatus = xhci_portstatus,
  .detect_dev = xhci_detect_dev,

  /* estimated max. count of TDs for one bulk transfer */
  .max_bulk_tds = 16, //GRUB_EHCI_N_TD * 3 / 4
};

GRUB_MOD_INIT (xhci)
{
  //xhci_trace ("[loading]\n");
  xhci_driver_init();
  grub_stop_disk_firmware ();
  grub_boot_time ("Initing xHCI hardware");
  grub_pci_iterate (xhci_pci_iter, NULL);
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
