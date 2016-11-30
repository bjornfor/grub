/* xhci.h - xHCI driver */
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

/*
 * Simple xHCI driver for GRUB.
 *
 * Limitations:
 *  - No 64-bit adddressing (only 32-bit)
 *  - No interrupts (just polling)
 *  - Only bulk transfer
 *
 * Status: INCOMPLETE
 *
 * [spec] http://www.intel.com/content/www/us/en/io/universal-serial-bus/extensible-host-controler-interface-usb-xhci.html
 */

#include <grub/dl.h>
#include <grub/mm.h>
#include <grub/usb.h>
#include <grub/usbtrans.h>
#include <grub/misc.h>
#include <grub/env.h>
#include <grub/term.h>
#include <grub/pci.h>
#include <grub/cpu/pci.h>
#include <grub/cpu/io.h>
#include <grub/time.h>
#include <grub/loader.h>
#include <grub/disk.h>
#include <grub/cache.h>

//#include "xhci-ipxe.h"

#include "xhci.h"
#include "xhci_io.h"
#include "xhci_debug.h"
#include "xhci_private.h"

GRUB_MOD_LICENSE ("GPLv3+");

#if 0
/**
 * Transcribe port speed (for debugging)
 *
 * @v psi		Protocol speed ID
 * @ret speed		Transcribed speed
 */
static const char *
xhci_speed_name (uint32_t psi)
{
  static const char *exponents[4] = { "", "k", "M", "G" };
  static char buf[10 /* "xxxxxXbps" + NUL */];
  unsigned int mantissa;
  unsigned int exponent;

  /* Extract mantissa and exponent */
  mantissa = XHCI_SUPPORTED_PSI_MANTISSA ( psi );
  exponent = XHCI_SUPPORTED_PSI_EXPONENT ( psi );

  /* Transcribe speed */
  snprintf ( buf, sizeof ( buf ), "%d%sbps",
      mantissa, exponents[exponent] );
  return buf;
}
#endif

/* Read Port Register Set n of given type */
grub_uint32_t
xhci_read_portrs(struct xhci *xhci, unsigned int port, enum xhci_portrs_type type)
{
  grub_uint8_t *addr;

  if (port > xhci->max_ports)
  {
    xhci_err ("too big port number\n");
    return 0;
  }

  addr = (grub_uint8_t*)xhci->oper_regs + 0x400 + (0x10 * (port - 1)) + type;
  return mmio_read32 ((grub_uint32_t *)addr);
}

static void xhci_doorbell_notify (struct xhci_trb_ring *ring)
{
  // TODO: write barrier
  mmio_write32(ring->db_reg, ring->db_val);
}

/* Convert raw PAGESIZE value from Operational Register to a size in bytes */
grub_size_t
xhci_pagesize_to_bytes(int pagesize)
{
  return 1 << (pagesize + 12);
}

/* Halt if xHCI HC not halted */
static grub_usb_err_t
xhci_halt (struct xhci *xhci)
{
  grub_uint64_t maxtime;
  unsigned int is_halted;

  is_halted = mmio_read_bits(&xhci->oper_regs->usbsts, XHCI_OP_USBSTS_HCH);
  if (is_halted == 0)
    {
      xhci_trace ("grub_xhci_halt not halted - halting now\n");
      mmio_write_bits(&xhci->oper_regs->usbcmd, XHCI_OP_USBCMD_RUNSTOP, 0);
      /* Ensure command is written */
      mmio_read32(&xhci->oper_regs->usbcmd);
      maxtime = grub_get_time_ms () + 16; /* spec says 16ms max */
      while (((mmio_read_bits(&xhci->oper_regs->usbsts,
               XHCI_OP_USBSTS_HCH) == 0)
             && (grub_get_time_ms () < maxtime)));
      if ((mmio_read_bits(&xhci->oper_regs->usbsts,
           XHCI_OP_USBSTS_HCH)) == 0)
        return GRUB_USB_ERR_TIMEOUT;
    }
  else
  {
    xhci_trace ("grub_xhci_halt already halted\n");
  }

  return GRUB_USB_ERR_NONE;
}

/* xHCI HC reset */
static grub_usb_err_t
xhci_reset (struct xhci *xhci)
{
  grub_uint64_t maxtime;

  xhci_trace ("xhci_reset enter\n");

  //sync_all_caches (xhci);

  mmio_write_bits(&xhci->oper_regs->usbcmd, XHCI_OP_USBCMD_HCRST, 1);
  /* Ensure command is written */
  mmio_read32(&xhci->oper_regs->usbcmd);
  /* XXX: How long time could take reset of HC ? */
  maxtime = grub_get_time_ms () + 1000;
  while (((mmio_read_bits(&xhci->oper_regs->usbsts,
           XHCI_OP_USBCMD_HCRST)) == 0)
         && (grub_get_time_ms () < maxtime));
  if ((mmio_read_bits (&xhci->oper_regs->usbsts,
       XHCI_OP_USBCMD_HCRST)) == 0)
    return GRUB_USB_ERR_TIMEOUT;

  return GRUB_USB_ERR_NONE;
}

static void
xhci_run (struct xhci *xhci)
{
  (void)xhci;
  //grub_uint32_t config;
  //grub_uint32_t usbcmd;

  //if (mmio_read32(&xhci->oper_regs->usbsts) & )
  {
    //xhci_err("programming error\n");
  }

  /* Configure number of device ports */
  //mmio_set_bits(&xhci->oper_regs->config, xhci->max_ports);

  //config = readl ( xhci->op + XHCI_OP_CONFIG );
  //config &= ~XHCI_CONFIG_MAX_SLOTS_EN_MASK;
  //config |= XHCI_CONFIG_MAX_SLOTS_EN ( xhci->slots );
  //writel ( config, xhci->op + XHCI_OP_CONFIG );

  /* Set run/stop bit */
  //usbcmd = readl ( xhci->op + XHCI_OP_USBCMD );
  //usbcmd |= XHCI_USBCMD_RUN;
  //writel ( usbcmd, xhci->op + XHCI_OP_USBCMD );
}

#if 0

static void
sync_all_caches (struct xhci *xhci)
{
  (void)xhci;
  xhci_trace("sync_all_caches enter\n");
  return;
#if 0
  if (!xhci)
    return;
  if (xhci->td_virt)
    grub_arch_sync_dma_caches (xhci->td_virt, sizeof (struct grub_xhci_td) *
			       GRUB_XHCI_N_TD);
  if (xhci->qh_virt)
    grub_arch_sync_dma_caches (xhci->qh_virt, sizeof (struct grub_xhci_qh) *
			       GRUB_XHCI_N_QH);
  if (xhci->framelist_virt)
    grub_arch_sync_dma_caches (xhci->framelist_virt, 4096);
#endif
}

#endif

int
xhci_cancel_transfer (struct xhci *xhci)
{
  (void)xhci;
  xhci_trace ("xhci_cancel_transfer: begin\n");
  return GRUB_USB_ERR_NONE;

#if 0
  grub_size_t actual;
  int i;
  grub_uint64_t maxtime;
  grub_uint32_t qh_phys;

  sync_all_caches (xhci);

  grub_uint32_t interrupt =
    cdata->qh_virt->ep_cap & GRUB_XHCI_SMASK_MASK;

  /* QH can be active and should be de-activated and halted */

  xhci_trace ("cancel_transfer: begin\n");

  /* First check if xHCI is running - if not, there is no problem */
  /* to cancel any transfer. Or, if transfer is asynchronous, check */
  /* if AL is enabled - if not, transfer can be canceled also. */
  if (((grub_xhci_oper_read32 (xhci, GRUB_XHCI_STATUS) &
      GRUB_XHCI_ST_HC_HALTED) != 0) ||
    (!interrupt && ((grub_xhci_oper_read32 (xhci, GRUB_XHCI_STATUS) &
      (GRUB_XHCI_ST_AS_STATUS | GRUB_XHCI_ST_PS_STATUS)) == 0)))
    {
      grub_xhci_pre_finish_transfer (transfer);
      grub_free (cdata);
      sync_all_caches (xhci);
      xhci_trace ("cancel_transfer: end - xHCI not running\n");
      return GRUB_USB_ERR_NONE;
    }

  /* xHCI and (AL or SL) are running. What to do? */
  /* Try to Halt QH via de-scheduling QH. */
  /* Find index of previous QH */
  qh_phys = grub_dma_virt2phys(cdata->qh_virt, xhci->qh_chunk);
  for (i = 0; i < GRUB_XHCI_N_QH; i++)
    {
      if ((grub_le_to_cpu32(xhci->qh_virt[i].qh_hptr)
        & GRUB_XHCI_QHTDPTR_MASK) == qh_phys)
        break;
    }
  if (i == GRUB_XHCI_N_QH)
    {
      grub_printf ("%s: prev not found, queues are corrupt\n", __func__);
      return GRUB_USB_ERR_UNRECOVERABLE;
    }
  /* Unlink QH from AL */
  xhci->qh_virt[i].qh_hptr = cdata->qh_virt->qh_hptr;

  sync_all_caches (xhci);

  /* If this is an interrupt transfer, we just wait for the periodic
   * schedule to advance a few times and then assume that the xHCI
   * controller has read the updated QH. */
  if (cdata->qh_virt->ep_cap & GRUB_XHCI_SMASK_MASK)
    {
      grub_millisleep(20);
    }
  else
    {
      /* For the asynchronous schedule we use the advance doorbell to find
       * out when the xHCI controller has read the updated QH. */

      /* Ring the doorbell */
      grub_xhci_oper_write32 (xhci, GRUB_XHCI_OPER_USBCMD,
                              GRUB_XHCI_CMD_AS_ADV_D
                              | grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD));
      /* Ensure command is written */
      grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD);
      /* Wait answer with timeout */
      maxtime = grub_get_time_ms () + 2;
      while (((grub_xhci_oper_read32 (xhci, GRUB_XHCI_STATUS)
               & GRUB_XHCI_ST_AS_ADVANCE) == 0)
             && (grub_get_time_ms () < maxtime));

      /* We do not detect the timeout because if timeout occurs, it most
       * probably means something wrong with xHCI - maybe stopped etc. */

      /* Shut up the doorbell */
      grub_xhci_oper_write32 (xhci, GRUB_XHCI_OPER_USBCMD,
                              ~GRUB_XHCI_CMD_AS_ADV_D
                              & grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD));
      grub_xhci_oper_write32 (xhci, GRUB_XHCI_STATUS,
                              GRUB_XHCI_ST_AS_ADVANCE
                              | grub_xhci_oper_read32 (xhci, GRUB_XHCI_STATUS));
      /* Ensure command is written */
      grub_xhci_oper_read32 (xhci, GRUB_XHCI_STATUS);
    }

  /* Now is QH out of AL and we can do anything with it... */
  grub_xhci_pre_finish_transfer (transfer);

  /* "Free" the QH - link it to itself */
  cdata->qh_virt->ep_char = 0;
  cdata->qh_virt->qh_hptr =
    grub_cpu_to_le32 ((grub_dma_virt2phys (cdata->qh_virt,
                                           xhci->qh_chunk)
                       & XHCI_POINTER_MASK) | GRUB_XHCI_HPTR_TYPE_QH);

  grub_free (cdata);

  xhci_trace ("cancel_transfer: end\n");

  sync_all_caches (xhci);

#endif
  return GRUB_USB_ERR_NONE;
}

enum xhci_speed
xhci_detect_dev (struct xhci *xhci, int port, int *changed)
{
  grub_uint32_t status, line_state;

  (void)port;
  (void)changed;
  (void)xhci;
  (void)line_state;
  (void)status;
  static int state;
  grub_uint32_t portsc;

  //xhci_trace ("xhci_detect_dev port=%d\n", port);
  if (xhci_debug_enabled())
  {


    for (unsigned int i=0; i<xhci->max_ports; i++)
    {
      xhci_dump_oper_portsc(xhci, i);
      grub_printf ("\n");
    }
    grub_printf ("\n");
    grub_millisleep(2000);
    return 0;


    xhci_dump_oper_portsc(xhci, port);
    grub_printf ("\n");
  }
  portsc = xhci_read_portrs (xhci, port, PORTSC);
  if (parse_reg(portsc, XHCI_OP_PORTSC_CCS))
  {
    grub_printf ("xHCI port %d IS CONNECTED!!!\n", port);
    grub_millisleep (1000);
  }

  if (xhci_debug_enabled())
    grub_millisleep (500);

  switch (state) {
    case 0:
      state = 0;
      *changed = 1;
      return GRUB_USB_SPEED_SUPER;

    case 1:
      state = 2;
      *changed = 0;
      return GRUB_USB_SPEED_NONE;

    case 2:
      state = 0;
      *changed = 1;
      return GRUB_USB_SPEED_SUPER;
      break;

    default:
      break;
  }

  return GRUB_USB_SPEED_NONE;

#if 0
  status = grub_xhci_port_read (xhci, port);

  /* Connect Status Change bit - it detects change of connection */
  if (status & GRUB_XHCI_PORTSC_PED)
    {
      *changed = 1;
      /* Reset bit Connect Status Change */
      grub_xhci_port_setbits (xhci, port, GRUB_XHCI_PORTSC_PED);
    }
  else
    *changed = 0;

  if (!(status & GRUB_XHCI_PORTSC_CCS))
    {				/* We should reset related "reset" flag in not connected state */
      xhci->reset &= ~(1 << port);
      return GRUB_USB_SPEED_NONE;
    }
  /* Detected connected state, so we should return speed.
   * But we can detect only LOW speed device and only at connection
   * time when PortEnabled=FALSE. FULL / HIGH speed detection is made
   * later by xHCI-specific reset procedure.
   * Another thing - if detected speed is LOW at connection time,
   * we should change port ownership to companion controller.
   * So:
   * 1. If we detect connected and enabled and xHCI-owned port,
   * we can say it is HIGH speed.
   * 2. If we detect connected and not xHCI-owned port, we can say
   * NONE speed, because such devices are not handled by xHCI.
   * 3. If we detect connected, not enabled but reset port, we can say
   * NONE speed, because it means FULL device connected to port and
   * such devices are not handled by xHCI.
   * 4. If we detect connected, not enabled and not reset port, which
   * has line state != "K", we will say HIGH - it could be FULL or HIGH
   * device, we will see it later after end of xHCI-specific reset
   * procedure.
   * 5. If we detect connected, not enabled and not reset port, which
   * has line state == "K", we can say NONE speed, because LOW speed
   * device is connected and we should change port ownership. */
  if ((status & GRUB_XHCI_PORT_ENABLED) != 0)	/* Port already enabled, return high speed. */
    return GRUB_USB_SPEED_HIGH;
  if ((status & GRUB_XHCI_PORT_OWNER) != 0)	/* xHCI is not port owner */
    return GRUB_USB_SPEED_NONE;	/* xHCI driver is ignoring this port. */
  if ((xhci->reset & (1 << port)) != 0)	/* Port reset was done = FULL speed */
    return GRUB_USB_SPEED_NONE;	/* xHCI driver is ignoring this port. */
  else				/* Port connected but not enabled - test port speed. */
    {
      line_state = status & GRUB_XHCI_PORT_LINE_STAT;
      if (line_state != GRUB_XHCI_PORT_LINE_LOWSP)
	return GRUB_USB_SPEED_HIGH;
      /* Detected LOW speed device, we should change
       * port ownership.
       * XXX: Fix it!: There should be test if related companion
       * controler is available ! And what to do if it does not exist ? */
      grub_xhci_port_setbits (xhci, port, GRUB_XHCI_PORT_OWNER);
      return GRUB_USB_SPEED_NONE;	/* Ignore this port */
      /* Note: Reset of PORT_OWNER bit is done by xHCI HW when
       * device is really disconnected from port.
       * Don't do PORT_OWNER bit reset by SW when not connected signal
       * is detected in port register ! */
    }

#endif
  return GRUB_USB_SPEED_NONE;
}

int
xhci_portstatus (struct xhci *xhci,
		 unsigned int port, unsigned int enable)
{
  (void)xhci;
  (void)port;
  (void)enable;
  xhci_trace ("xhci_portstatus enter (port=%d, enable=%d)\n",
      port, enable);
  return GRUB_USB_ERR_NONE;

#if 0
  grub_uint64_t endtime;

  xhci_trace ("portstatus: xHCI USBSTS: %08x\n",
		grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBSTS));
  xhci_trace (
		"portstatus: begin, iobase_cap=%p, port=%d, status=0x%02x\n",
		xhci->iobase_cap, port, grub_xhci_port_read (xhci, port));

  /* In any case we need to disable port:
   * - if enable==false - we should disable port
   * - if enable==true we will do the reset and the specification says
   *   PortEnable should be FALSE in such case */
  /* Disable the port and wait for it. */
  xhci_cap_read32 (xhci, XHCI_PORTSC(port));
  endtime = grub_get_time_ms () + 1000;
  while (grub_xhci_port_read (xhci, port) & GRUB_XHCI_PORT_ENABLED)
    if (grub_get_time_ms () > endtime)
      return GRUB_USB_ERR_TIMEOUT;

  if (!enable)			/* We don't need reset port */
    {
      xhci_trace ("portstatus: Disabled.\n");
      xhci_trace ("portstatus: end, status=0x%02x\n",
		    grub_xhci_port_read (xhci, port));
      return GRUB_USB_ERR_NONE;
    }

  xhci_trace ("portstatus: enable\n");

  grub_boot_time ("Resetting port %d", port);

  /* Now we will do reset - if HIGH speed device connected, it will
   * result in Enabled state, otherwise port remains disabled. */
  /* Set RESET bit for 50ms */
  grub_xhci_port_setbits (xhci, port, GRUB_XHCI_PORT_RESET);
  grub_millisleep (50);

  /* Reset RESET bit and wait for the end of reset */
  grub_xhci_port_resbits (xhci, port, GRUB_XHCI_PORT_RESET);
  endtime = grub_get_time_ms () + 1000;
  while (grub_xhci_port_read (xhci, port) & GRUB_XHCI_PORT_RESET)
    if (grub_get_time_ms () > endtime)
      return GRUB_USB_ERR_TIMEOUT;
  grub_boot_time ("Port %d reset", port);
  /* Remember "we did the reset" - needed by detect_dev */
  xhci->reset |= (1 << port);
  /* Test if port enabled, i.xhci. HIGH speed device connected */
  if ((grub_xhci_port_read (xhci, port) & GRUB_XHCI_PORT_ENABLED) != 0)	/* yes! */
    {
      xhci_trace ("portstatus: Enabled!\n");
      /* "Reset recovery time" (USB spec.) */
      grub_millisleep (10);
    }
  else				/* no... */
    {
      /* FULL speed device connected - change port ownership.
       * It results in disconnected state of this xHCI port. */
      grub_xhci_port_setbits (xhci, port, GRUB_XHCI_PORT_OWNER);
      return GRUB_USB_ERR_BADDEVICE;
    }

  /* XXX: Fix it! There is possible problem - we can say to calling
   * function that we lost device if it is FULL speed onlu via
   * return value <> GRUB_ERR_NONE. It (maybe) displays also error
   * message on screen - but this situation is not error, it is normal
   * state! */

  xhci_trace ("portstatus: end, status=0x%02x\n",
		grub_xhci_port_read (xhci, port));

#endif
  return GRUB_USB_ERR_NONE;
}

int
xhci_hubports (struct xhci *xhci)
{
  unsigned int nports = 0;

  nports = xhci->max_ports;
  xhci_trace ("xhci_hubports nports=%d\n", nports);

  //xhci_trace ("xhci_hubports force nports=0 (prevent hang)\n");
  //nports = 0;
  //xhci->max_ports = nports;
  return nports;
}

int
xhci_check_transfer (struct xhci *xhci)
{
  (void)xhci;

  //xhci_trace ("xhci_check_transfer enter (TODO: implement)\n");
  return 0;
#if 0
  grub_uint32_t token, token_ftd;

  sync_all_caches (xhci);

  xhci_trace (
		"check_transfer: xHCI STATUS=%08x, cdata=%p, qh=%p\n",
		grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBSTS),
		cdata, cdata->qh_virt);
  xhci_trace ("check_transfer: qh_hptr=%08x, ep_char=%08x\n",
		grub_le_to_cpu32 (cdata->qh_virt->qh_hptr),
		grub_le_to_cpu32 (cdata->qh_virt->ep_char));
  xhci_trace ("check_transfer: ep_cap=%08x, td_current=%08x\n",
		grub_le_to_cpu32 (cdata->qh_virt->ep_cap),
		grub_le_to_cpu32 (cdata->qh_virt->td_current));
  xhci_trace ("check_transfer: next_td=%08x, alt_next_td=%08x\n",
		grub_le_to_cpu32 (cdata->qh_virt->td_overlay.next_td),
		grub_le_to_cpu32 (cdata->qh_virt->td_overlay.alt_next_td));
  xhci_trace ("check_transfer: token=%08x, buffer[0]=%08x\n",
		grub_le_to_cpu32 (cdata->qh_virt->td_overlay.token),
		grub_le_to_cpu32 (cdata->qh_virt->td_overlay.buffer_page[0]));

  /* Check if xHCI is running and AL is enabled */
  if ((grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBSTS)
       & GRUB_XHCI_USBSTS_HCH) != 0)
    return grub_xhci_parse_notrun (dev, transfer, actual);
  if ((grub_xhci_oper_read32 (xhci, GRUB_XHCI_STATUS)
       & (GRUB_XHCI_ST_AS_STATUS | GRUB_XHCI_ST_PS_STATUS)) == 0)
    return grub_xhci_parse_notrun (dev, transfer, actual);

  token = grub_le_to_cpu32 (cdata->qh_virt->td_overlay.token);
  /* If the transfer consist from only one TD, we should check */
  /* if the TD was really executed and deactivated - to prevent */
  /* false detection of transfer finish. */
  token_ftd = grub_le_to_cpu32 (cdata->td_first_virt->token);

  /* Detect QH halted */
  if ((token & GRUB_XHCI_STATUS_HALTED) != 0)
    return grub_xhci_parse_halt (dev, transfer, actual);

  /* Detect QH not active - QH is not active and no next TD */
  if (token && ((token & GRUB_XHCI_STATUS_ACTIVE) == 0)
	&& ((token_ftd & GRUB_XHCI_STATUS_ACTIVE) == 0))
    {
      /* It could be finish at all or short packet condition */
      if ((grub_le_to_cpu32 (cdata->qh_virt->td_overlay.next_td)
	   & GRUB_XHCI_TERMINATE) &&
	  ((grub_le_to_cpu32 (cdata->qh_virt->td_current)
	    & GRUB_XHCI_QHTDPTR_MASK) == cdata->td_last_phys))
	/* Normal finish */
	return grub_xhci_parse_success (dev, transfer, actual);
      else if ((token & GRUB_XHCI_TOTAL_MASK) != 0)
	/* Short packet condition */
	/* But currently we don't handle it - higher level will do it */
	return grub_xhci_parse_success (dev, transfer, actual);
    }

#endif
  return GRUB_USB_ERR_WAIT;
}


int
xhci_setup_transfer (struct xhci *xhci)
{
  (void)xhci;
  //xhci_trace ("xhci_setup_transfer enter (TODO: implement)\n");
  /* pretend we managed to start sending data */
  return 0;

#if 0
  struct xhci *xhci = (struct xhci *) dev->data;
  grub_xhci_td_t td = NULL;
  grub_xhci_td_t td_prev = NULL;
  int i;
  struct grub_xhci_transfer_controller_data *cdata;
  grub_uint32_t status;

  sync_all_caches (xhci);

  /* Check if xHCI is running and AL is enabled */
  status = grub_xhci_oper_read32 (xhci, GRUB_XHCI_STATUS);
  if ((status & GRUB_XHCI_ST_HC_HALTED) != 0)
    /* XXX: Fix it: Currently we don't do anything to restart xHCI */
    {
      xhci_trace ("setup_transfer: halted, status = 0x%x\n",
		    status);
      return GRUB_USB_ERR_INTERNAL;
    }
  status = grub_xhci_oper_read32 (xhci, GRUB_XHCI_STATUS);
  if ((status
       & (GRUB_XHCI_ST_AS_STATUS | GRUB_XHCI_ST_PS_STATUS)) == 0)
    /* XXX: Fix it: Currently we don't do anything to restart xHCI */
    {
      xhci_trace ("setup_transfer: no AS/PS, status = 0x%x\n",
		    status);
      return GRUB_USB_ERR_INTERNAL;
    }

  /* Allocate memory for controller transfer data.  */
  cdata = grub_malloc (sizeof (*cdata));
  if (!cdata)
    return GRUB_USB_ERR_INTERNAL;
  cdata->td_first_virt = NULL;

  /* Allocate a queue head for the transfer queue.  */
  cdata->qh_virt = 0; //grub_xhci_find_qh (xhci, transfer);
  if (!cdata->qh_virt)
    {
      xhci_trace ("setup_transfer: no QH\n");
      grub_free (cdata);
      return GRUB_USB_ERR_INTERNAL;
    }

  /* To detect short packet we need some additional "alternate" TD,
   * allocate it first. */
  cdata->td_alt_virt = 0; //grub_xhci_alloc_td (xhci);
  if (!cdata->td_alt_virt)
    {
      xhci_trace ("setup_transfer: no TDs\n");
      grub_free (cdata);
      return GRUB_USB_ERR_INTERNAL;
    }
  /* Fill whole alternate TD by zeros (= inactive) and set
   * Terminate bits and Halt bit */
  grub_memset ((void *) cdata->td_alt_virt, 0, sizeof (struct grub_xhci_td));
  cdata->td_alt_virt->next_td = grub_cpu_to_le32_compile_time (GRUB_XHCI_TERMINATE);
  cdata->td_alt_virt->alt_next_td = grub_cpu_to_le32_compile_time (GRUB_XHCI_TERMINATE);
  cdata->td_alt_virt->token = grub_cpu_to_le32_compile_time (GRUB_XHCI_STATUS_HALTED);

  /* Allocate appropriate number of TDs and set */
  for (i = 0; i < transfer->transcnt; i++)
    {
      grub_usb_transaction_t tr = &transfer->transactions[i];

      td = grub_xhci_transaction (xhci, tr->pid, tr->toggle, tr->size,
				  tr->data, cdata->td_alt_virt);

      if (!td)			/* de-allocate and free all */
	{
	  grub_size_t actual = 0;

	  if (cdata->td_first_virt)
	    grub_xhci_free_tds (xhci, cdata->td_first_virt, NULL, &actual);

	  grub_free (cdata);
	  xhci_trace ("setup_transfer: no TD\n");
	  return GRUB_USB_ERR_INTERNAL;
	}

      /* Register new TD in cdata or previous TD */
      if (!cdata->td_first_virt)
	cdata->td_first_virt = td;
      else
	{
	  td_prev->link_td = grub_dma_virt2phys (td, xhci->td_chunk);
	  td_prev->next_td =
	    grub_cpu_to_le32 (grub_dma_virt2phys (td, xhci->td_chunk));
	}
      td_prev = td;
    }

  /* Remember last TD */
  cdata->td_last_virt = td;
  cdata->td_last_phys = grub_dma_virt2phys (td, xhci->td_chunk);
  /* Last TD should not have set alternate TD */
  cdata->td_last_virt->alt_next_td = grub_cpu_to_le32_compile_time (GRUB_XHCI_TERMINATE);

  xhci_trace ("setup_transfer: cdata=%p, qh=%p\n",
		cdata,cdata->qh_virt);
  xhci_trace ("setup_transfer: td_first=%p, td_alt=%p\n",
		cdata->td_first_virt,
		cdata->td_alt_virt);
  xhci_trace ("setup_transfer: td_last=%p\n",
		cdata->td_last_virt);

  /* Start transfer: */
  /* Unlink possible alternate pointer in QH */
  cdata->qh_virt->td_overlay.alt_next_td =
    grub_cpu_to_le32_compile_time (GRUB_XHCI_TERMINATE);
  /* Link new TDs with QH via next_td */
  cdata->qh_virt->td_overlay.next_td =
    grub_cpu_to_le32 (grub_dma_virt2phys
		      (cdata->td_first_virt, xhci->td_chunk));
  /* Reset Active and Halted bits in QH to activate Advance Queue,
   * i.e. reset token */
  cdata->qh_virt->td_overlay.token = grub_cpu_to_le32_compile_time (0);

  sync_all_caches (xhci);

  /* Finito */
  transfer->controller_data = cdata;

#endif
  //return GRUB_USB_ERR_TIMEOUT;
  return GRUB_USB_ERR_NONE;
}


/**
 * Dequeue a transfer request block
 *
 * @v ring		TRB ring
 * @ret iobuf		I/O buffer
 */
static void
xhci_dequeue (struct xhci_trb_ring *ring)
{
  volatile union xhci_trb *trb;
  unsigned int cons;
  unsigned int mask;
  unsigned int index;

  /* Sanity check */
  //assert ( xhci_ring_fill ( ring ) != 0 );

  /* Update consumer counter */
  cons = ring->cons++;
  mask = ring->mask;
  index = cons & mask;

  /* Retrieve TRB */
  trb = &ring->trbs[index];
  /* TODO: now what? */
  (void)trb;
}

/**
 * Handle command completion event
 *
 * @v xhci		xHCI device
 * @v trb		Command completion event
 */
static void
xhci_complete (struct xhci *xhci, volatile struct xhci_trb_complete *trb)
{
  int rc;

  /* Ignore "command ring stopped" notifications */
  if ( trb->code == XHCI_CMPLT_CMD_STOPPED ) {
    xhci_dbg("command ring stopped\n");
    return;
  }

  /* Ignore unexpected completions */
  if ( ! xhci->pending ) {
    rc = -1;
    xhci_dbg("unexpected completion (code %d): %d\n",
        trb->code, rc);
    return;
  }

  /* Dequeue command TRB */
  xhci_dequeue ( &xhci->command_ring );

  /* Sanity check */
  //assert ( xhci_ring_consumed ( &xhci->command_ring ) ==
      //le64_to_cpu ( trb->command ) );

  /* Record completion */
  *trb = xhci->pending->complete;

  xhci->pending = NULL;
}

#if 0
/**
 * Handle port status event
 *
 * @v xhci		xHCI device
 * @v trb		Port status event
 */
static void xhci_port_status ( struct xhci *xhci,
			       struct xhci_trb_port_status *trb )
{
  struct usb_port *port = usb_port ( xhci->bus->hub, trb->port );
  uint32_t portsc;

  /* Sanity check */
  assert ( ( trb->port > 0 ) && ( trb->port <= xhci->ports ) );

  /* Record disconnections and clear changes */
  portsc = readl ( xhci->op + XHCI_OP_PORTSC ( trb->port ) );
  port->disconnected |= ( portsc & XHCI_PORTSC_CSC );
  portsc &= ( XHCI_PORTSC_PRESERVE | XHCI_PORTSC_CHANGE );
  writel ( portsc, xhci->op + XHCI_OP_PORTSC ( trb->port ) );

  /* Report port status change */
  usb_port_changed ( port );
}
#endif

/**
 * Handle host controller event
 *
 * @v xhci		xHCI device
 * @v trb		Host controller event
 */
static void xhci_host_controller ( struct xhci *xhci,
				   volatile struct xhci_trb_host_controller *trb )
{
  int rc;
  (void)xhci;

  /* Construct error */
  rc = -1;
  xhci_dbg("XHCI host controller event (code %d): %d\n",
      trb->code, rc);
}

/**
 * Poll event ring
 *
 * @v xhci		xHCI device
 */
static void
xhci_event_poll (struct xhci *xhci)
{
  struct xhci_event_ring *event = &xhci->event_ring;
  volatile union xhci_trb *trb;
  unsigned int shift = XHCI_EVENT_TRBS_LOG2;
  unsigned int count = ( 1 << shift );
  unsigned int mask = ( count - 1 );
  unsigned int consumed;
  unsigned int type;

  /* Poll for events */
  for ( consumed = 0 ; ; consumed++ ) {

    /* Stop if we reach an empty TRB */
    //rmb();
    trb = &event->trb[ event->cons & mask ];
    if ( ! ( ( trb->common.flags ^
            ( event->cons >> shift ) ) & XHCI_TRB_C ) )
      break;

    /* Handle TRB */
    type = parse_reg(trb->templ.control, XHCI_TRB_CTRL__TRB_TYPE);
    xhci_dbg("processing event type %d\n", type);
    switch (type)
    {
      case XHCI_TRB_TYPE_TRANSFER_EVENT:
        //xhci_transfer(xhci, &trb->transfer);
        break;

      case XHCI_TRB_TYPE_COMMAND_COMPLETION_EVENT:
        xhci_complete(xhci, &trb->complete);
        break;

      case XHCI_TRB_TYPE_PORT_STATUS_CHANGE_EVENT:
        //xhci_port_status(xhci, &trb->port);
        break;

      case XHCI_TRB_TYPE_HOST_CONTROLLER_EVENT:
        xhci_host_controller(xhci, &trb->host);
        break;

      default:
        xhci_dbg("unrecognised event 0x%x\n:", event->cons );
        break;
    }

    xhci_dbg("event->cons: %d\n", event->cons);

    /* Consume this TRB */
    event->cons++;
  }

  /* Update dequeue pointer if applicable */
  if (consumed) {
    // TODO
    //mmio_write32(&xhci->run_regs->, grub_dma_virt2phys(trb));
        //xhci->run + XHCI_RUN_ERDP ( 0 ) );
  }
}

/**
 * Calculate space used in TRB ring
 *
 * @v ring		TRB ring
 * @ret fill		Number of entries used
 */
static unsigned int
xhci_ring_fill(struct xhci_trb_ring *ring)
{
  return ring->prod - ring->cons;
}

/**
 * Calculate space remaining in TRB ring
 *
 * @v ring		TRB ring
 * @ret remaining	Number of entries remaining
 *
 * xHCI does not allow us to completely fill a ring; there must be at
 * least one free entry (excluding the Link TRB).
 */
static unsigned int
xhci_ring_remaining ( struct xhci_trb_ring *ring )
{
  unsigned int fill = xhci_ring_fill(ring);

  /* We choose to utilise rings with ( 2^n + 1 ) entries, with
   * the final entry being a Link TRB.  The maximum fill level
   * is therefore
   *
   *   ( ( 2^n + 1 ) - 1 (Link TRB) - 1 (one slot always empty)
   *       == ( 2^n - 1 )
   *
   * which is therefore equal to the ring mask.
   */
  xhci_dbg("xhci_ring_remaining: mask=%d fill=%d\n",
      ring->mask, fill);
  return ring->mask - fill;
}

/**
 * Enqueue a transfer request block
 *
 * @v ring		TRB ring
 * @v iobuf		I/O buffer (if any)
 * @v trb		Transfer request block (with empty Cycle flag)
 * @ret rc		Return status code
 *
 * This operation does not implicitly ring the doorbell register.
 */
static int
xhci_enqueue(struct xhci_trb_ring *ring, /* struct io_buffer *iobuf, */
    const volatile union xhci_trb *trb)
{
  volatile union xhci_trb *dest;
  unsigned int prod;
  unsigned int mask;
  unsigned int index;
  unsigned int cycle;

  /* Sanity check */
  //assert ( ! ( trb->common.flags & XHCI_TRB_C ) );

  /* Fail if ring is full */
  if (!xhci_ring_remaining(ring))
    return -1;

  /* Update producer counter (and link TRB, if applicable) */
  prod = ring->prod++;
  mask = ring->mask;
  cycle = ( ( ~( prod >> ring->shift ) ) & XHCI_TRB_C );
  index = ( prod & mask );
  /* TODO: */
  //if ( index == 0 )
  //  ring->link->flags = ( XHCI_TRB_TC | ( cycle ^ XHCI_TRB_C ) );

  /* Record I/O buffer */
  //ring->iobuf[index] = iobuf;

  /* Enqueue TRB */
  dest = &ring->trbs[index];
  dest->templ.parameter = trb->templ.parameter;
  dest->templ.status = trb->templ.status;
  //TODO: write barrier
  dest->templ.control = ( trb->templ.control |
      grub_cpu_to_le32 ( cycle ) );

  return 0;
}

/**
 * Issue command and wait for completion
 *
 * @v xhci		xHCI device
 * @v trb		Transfer request block (with empty Cycle flag)
 * @ret rc		Return status code
 *
 * On a successful completion, the TRB will be overwritten with the
 * completion.
 */
static int
xhci_command(struct xhci *xhci, union xhci_trb *trb)
{
  struct xhci_trb_complete *complete = &trb->complete;
  unsigned int i;
  int rc;

  /* Record the pending command */
  xhci->pending = trb;

  /* Enqueue the command */
  rc = xhci_enqueue(&xhci->command_ring, /*NULL,*/ trb);
  if (rc != 0) {
    xhci_dbg("xhci_enqueue failed\n");
    goto err_enqueue;
  }

  /* Ring the command doorbell */
  xhci_doorbell_notify(&xhci->command_ring);

  /* Wait for the command to complete */
  for (i = 0; i < XHCI_COMMAND_MAX_WAIT_MS; i++) {

    /* Poll event ring */
    xhci_event_poll(xhci);

    /* Check for completion */
    if (!xhci->pending) {
      if (complete->code != XHCI_CMPLT_SUCCESS) {
        rc = -1;
        xhci_dbg("command failed, completion code %d\n",
            complete->code, rc);
        return rc;
      }
      return 0;
    }

    /* Delay */
    grub_millisleep (1);
  }

  /* Timeout */
  xhci_dbg("timed out waiting for completion\n");
  rc = -1;

  /* Abort command */
  //xhci_abort(xhci);

err_enqueue:
  xhci->pending = NULL;
  return rc;
}

/** Transfer request block interrupt on completion flag */
#define XHCI_TRB_IOC 0x20

/**
 * Issue NOP and wait for completion
 *
 * @v xhci		xHCI device
 * @ret rc		Return status code
 */
static int xhci_nop(struct xhci *xhci)
{
  union xhci_trb trb;
  struct xhci_trb_common *nop = &trb.common;
  int rc;

  /* Construct command */
  grub_memset(nop, 0, sizeof (*nop));
  nop->flags = XHCI_TRB_IOC;
  nop->type = XHCI_TRB_TYPE_NO_OP_COMMAND;

  /* Issue command and wait for completion */
  rc = xhci_command(xhci, &trb);
  if (rc != 0)
    return rc;

  return 0;
}

/** Allocate memory for xHC Device Context Base Address Array.
 *
 * Store a copy of the pointer in the struct xhci instance. The allocated
 * memory must be physically contiguous, 64-byte aligned and the physical
 * address must be written to DCBAAP.
 */
static int
xhci_allocate_dcbaa(struct xhci *xhci)
{
  const int min_align = 64;

  if (xhci->dcbaa)
  {
    xhci_err ("dcbaa non-zero, possibly memory leak\n");
    return -1;
  }

  /* +1 to make room for the scratchpad pointer, at index 0 */
  xhci->dcbaa_len = (xhci->max_device_slots + 1) * sizeof (xhci->dcbaa[0]);
  xhci->dcbaa = (grub_uint64_t*)grub_memalign_dma32 (min_align, xhci->dcbaa_len);
  if (!xhci->dcbaa)
  {
    xhci_err ("out of memory, couldn't allocate DCBAA memory\n");
    return -1;
  }

  grub_memset(xhci->dcbaa, 0, xhci->dcbaa_len);
  return 0;
}

/** Program xHC DCBAAP register with physical DCBAA from 'xhci' instance */
static int
xhci_program_dcbaap(struct xhci *xhci)
{
  grub_uint32_t dcbaa_phys;

  /* only 32-bit support */
  dcbaa_phys = grub_dma_get_phys((struct grub_pci_dma_chunk*)xhci->dcbaa);
  xhci_trace ("DCBAA at 0x%08x (virt 0x%08x), len=%d\n",
      dcbaa_phys, xhci->dcbaa, xhci->dcbaa_len);
  /* DCBAAP lo+hi stores the _high order_ bits of the 64-bit address. The
   * reserved bits in "lo" are zero, so we can simply ignore them as our DCBAA
   * is 64-byte aligned (1<<6, where 6 is the number of reserved bits). IOW, don't
   * shift and write the higher bits to the "hi" register, just write the
   * 32-bit address to "lo".
   */
  mmio_write32(&xhci->oper_regs->dcbaap_lo, dcbaa_phys);
  return 0;
}

/** Allocate memory and write the pointer to DCBAAP register */
static int
xhci_setup_dcbaa(struct xhci *xhci)
{
  int rc;
  rc = xhci_allocate_dcbaa(xhci);
  if (rc)
    return rc;
  return xhci_program_dcbaap(xhci);
}

/*
 * The Scratchpad Buffer Array is host memory that are made available for
 * private use of the xHC. The host must not touch the memory after passing it
 * to the xHC.
 */
static int
xhci_setup_scratchpad(struct xhci *xhci)
{
  const int min_align = 64;
  int num_scratch_bufs;
  int i;
  grub_size_t pagesize;
  grub_uint32_t scratchpad_phys;
  grub_uint32_t scratchpad_arr_phys;

  num_scratch_bufs = mmio_read_bits(&xhci->cap_regs->hcsparams2,
      XHCI_CAP_HCSPARAMS2_MAX_SCRATCH_BUFS_LO)
    | (mmio_read_bits(&xhci->cap_regs->hcsparams2,
      XHCI_CAP_HCSPARAMS2_MAX_SCRATCH_BUFS_HI) << 5);

  xhci->num_scratch_bufs = num_scratch_bufs;

  if (!num_scratch_bufs)
  {
    xhci_trace("xHC needs %d scratchpad buffers\n", num_scratch_bufs);
    return 0;
  }

  /* Allocate Scratchpad Buffers */
  pagesize = xhci_pagesize_to_bytes(
      mmio_read_bits(&xhci->oper_regs->pagesize, XHCI_OP_PAGESIZE));
  xhci->pagesize = pagesize;
  xhci->scratchpads_len = num_scratch_bufs * pagesize;
  xhci->scratchpads = (grub_uint8_t*)grub_memalign_dma32 (min_align, xhci->scratchpads_len);
  if (!xhci->scratchpads)
  {
    xhci_err ("out of memory, couldn't allocate Scratchpad Buffer memory\n");
    return -1;
  }
  grub_memset(xhci->scratchpads, 0, xhci->scratchpads_len);

  /* Allocate Scratchpad Buffer Array, where each element points to a buffer */
  xhci->scratchpad_arr_len = num_scratch_bufs * sizeof (xhci->scratchpad_arr[0]);
  xhci->scratchpad_arr = (grub_uint64_t*)grub_memalign_dma32 (min_align, xhci->scratchpad_arr_len);
  if (!xhci->scratchpad_arr)
  {
    xhci_err ("out of memory, couldn't allocate Scratchpad Buffer Array memory\n");
    return -1;
  }
  grub_memset(xhci->scratchpad_arr, 0, xhci->scratchpad_arr_len);

  /* Fill Scratchpad Buffers Array with addresses of the scratch buffers */
  for (i = 0; i < num_scratch_bufs; i++)
  {
    scratchpad_phys = grub_dma_get_phys((struct grub_pci_dma_chunk*)
        (xhci->scratchpads + pagesize * i));
    xhci->scratchpad_arr[i] = scratchpad_phys;
  }

  /* Write Scratchpad Buffers Array base address to xHC */
  scratchpad_arr_phys = grub_dma_get_phys((struct grub_pci_dma_chunk*)xhci->scratchpad_arr);
  xhci_trace ("Scratchpad Buffer Array (nbuf=%d) at 0x%08x (virt 0x%08x), len=%d bytes\n",
      num_scratch_bufs, scratchpad_arr_phys, xhci->scratchpad_arr, xhci->scratchpad_arr_len);
   /* The location of the Scratcphad Buffer array is defined by entry 0 of the
    * DCBAA. We only support 32-bit.
    */
  mmio_write32((grub_uint32_t*)&xhci->dcbaa, scratchpad_arr_phys);

  return 0;
}

#if 0
/* Copied from iPXE */
static int
xhci_alloc_ring(struct xhci_device *xhci,
    struct xhci_trb_ring *ring,
    unsigned int shift, unsigned int slot,
    unsigned int target, unsigned int stream)
{
  struct xhci_trb_link *link;
  unsigned int count;
  int rc;

  /* Sanity check */
  assert ( shift > 0 );

  /* Initialise structure */
  memset ( ring, 0, sizeof ( *ring ) );
  ring->shift = shift;
  count = ( 1U << shift );
  ring->mask = ( count - 1 );
  ring->len = ( ( count + 1 /* Link TRB */ ) * sizeof ( ring->trb[0] ) );
  ring->db = ( xhci->db + ( slot * sizeof ( ring->dbval ) ) );
  ring->dbval = XHCI_DBVAL ( target, stream );

  /* Allocate I/O buffers */
  ring->iobuf = zalloc ( count * sizeof ( ring->iobuf[0] ) );
  if ( ! ring->iobuf ) {
    rc = -ENOMEM;
    goto err_alloc_iobuf;
  }

  /* Allocate TRBs */
  ring->trb = malloc_dma ( ring->len, xhci_align ( ring->len ) );
  if ( ! ring->trb ) {
    rc = -ENOMEM;
    goto err_alloc_trb;
  }
  memset ( ring->trb, 0, ring->len );

  /* Initialise Link TRB */
  link = &ring->trb[count].link;
  link->next = cpu_to_le64 ( virt_to_phys ( ring->trb ) );
  link->flags = XHCI_TRB_TC;
  link->type = XHCI_TRB_LINK;
  ring->link = link;

  return 0;

  free_dma ( ring->trb, ring->len );
err_alloc_trb:
  free ( ring->iobuf );
err_alloc_iobuf:
  return rc;
}
#endif

/*
 *
 */
static int
xhci_setup_ring(struct xhci *xhci,
    struct xhci_trb_ring *ring,
    unsigned int shift, unsigned int slot,
    unsigned int target, unsigned int stream)
{
  int min_align = 64;
  int count = 1 << shift;
  int len = (count + 1 /* Link TRB */) * sizeof (ring->trbs[0]);
  (void)xhci;
  //xhci_trace("%s: TODO: implement\n", __func__);
  ring->mask = count - 1;
  ring->shift = shift;
  ring->slot = slot;
  ring->db_reg = &xhci->db_regs->doorbell[slot];
  ring->db_val = XHCI_DBVAL(target, stream);

  /* Allocate (physically contiguous) memory for the TRBs */
  ring->trbs = (union xhci_trb *)grub_memalign_dma32 (min_align, len);

  return -1;
}

/** Allocate command ring memory and program xHC */
static int
xhci_setup_command_ring(struct xhci *xhci)
{
  int ring_len_log2 = 3; // 2**ring_len_log2 entries
  grub_uint32_t trbs_phys;

  xhci_setup_ring(xhci, &xhci->command_ring, ring_len_log2, 0, 0, 0);
  trbs_phys = grub_dma_get_phys((struct grub_pci_dma_chunk*)xhci->command_ring.trbs);
  mmio_write64(&xhci->oper_regs->crcr, trbs_phys | BIT(XHCI_OP_CRCR_RCS));
  return 0;
}

static int
xhci_setup_event_ring(struct xhci *xhci)
{
  struct xhci_event_ring *event = &xhci->event_ring;
  unsigned int count;
  grub_size_t len;
  int min_align = 64;

  /* Allocate event ring */
  count = 1 << XHCI_EVENT_TRBS_LOG2;
  len = count * sizeof (event->trb[0]);
  event->trb = (volatile union xhci_trb *)grub_memalign_dma32(min_align, len);
  if (!event->trb) {
    return -1;
  }

  for (unsigned int i = 0; i < count; i++)
  {
    event->trb[i] = (volatile union xhci_trb){0};
  }

  /* Allocate event ring segment table */
  event->segment = (struct xhci_event_ring_segment *)grub_memalign_dma32(
      sizeof (event->segment[0]), sizeof (event->segment[0]));
  if (!event->segment) {
    return -1;
  }

  event->segment[0] = (volatile struct xhci_event_ring_segment){0};
  event->segment[0].base = grub_cpu_to_le64(grub_dma_virt2phys((void*)event->trb, (void*)event->trb));
  event->segment[0].count = grub_cpu_to_le32(count);

  /* Program event ring registers */
  mmio_write32(&xhci->run_regs->ir_set[0].erstsz, 1);
  mmio_write64(&xhci->run_regs->ir_set[0].erdp, grub_dma_virt2phys((void*)event->trb, (void*)event->trb));
  mmio_write64(&xhci->run_regs->ir_set[0].erstba, grub_dma_virt2phys((void*)event->segment, (void*)event->segment));

  xhci_dbg("event ring [%08lx,%08lx) table [%08lx,%08lx)\n",
      grub_dma_virt2phys((void*)event->trb, (void*)event->trb),
      ( grub_dma_virt2phys((void*)event->trb, (void*)event->trb) + len ),
      grub_dma_virt2phys((void*)event->segment, (void*)event->segment),
      ( grub_dma_virt2phys((void*)event->segment, (void*)event->segment) +
        sizeof (event->segment[0])));
  return 0;

  /* TODO: free resources in case of error? */
}

struct xhci *xhci_new(void)
{
  return grub_zalloc (sizeof (struct xhci));
}

int xhci_init (struct xhci *xhci, volatile void *mmio_base_addr, int seqno)
{
  (void)seqno;
  int rc;
  grub_int32_t hcsparams1;
  //grub_uint32_t hcsparams2;
  //grub_uint32_t hccparams1;
  //grub_uint32_t pagesize;
  grub_uint64_t maxtime;

  grub_snprintf(xhci->name, sizeof(xhci->name), "%d", seqno);
  xhci->sbrn = 0; //pci_config_read8 (dev, XHCI_PCI_SBRN_REG);

  /* Locate capability, operational, runtime, and doorbell registers */
  xhci->cap_regs = mmio_base_addr;
  xhci->oper_regs = (struct xhci_oper_regs *)
    ((grub_uint8_t *)xhci->cap_regs +
     mmio_read_bits (&xhci->cap_regs->caplength_and_hciversion,
       XHCI_CAP_CAPLENGTH));
  xhci->run_regs = (struct xhci_run_regs *)
    ((grub_uint8_t *)xhci->cap_regs +
     RTSOFF_TO_BYTES(mmio_read_bits (&xhci->cap_regs->rtsoff, XHCI_CAP_RTSOFF)));
  xhci->db_regs = (struct xhci_doorbell_regs *)
    ((grub_uint8_t *)xhci->cap_regs +
     DBOFF_TO_BYTES(mmio_read_bits (&xhci->cap_regs->dboff, XHCI_CAP_DBOFF)));

  /* Paranoia/sanity check: wait until controller is ready */
  maxtime = grub_get_time_ms () + 1000;
  while (1)
  {
    if ((mmio_read_bits(&xhci->oper_regs->usbsts, XHCI_OP_USBSTS_CNR)) == 0)
      break;

    if (grub_get_time_ms () > maxtime)
    {
      xhci_err ("timeout waiting for USBSTS_CNR to clear\n");
      return -1;
    }
  }

  /* Get some structural info */
  hcsparams1 = mmio_read32 (&xhci->cap_regs->hcsparams1);
  xhci->max_device_slots = parse_reg(hcsparams1, XHCI_CAP_HCSPARAMS1_MAX_DEVICE_SLOTS);
  xhci->max_ports = parse_reg(hcsparams1, XHCI_CAP_HCSPARAMS1_MAX_PORTS);

  /* Enable all slots */
  xhci->num_enabled_slots = xhci->max_device_slots;
  mmio_write_bits(&xhci->oper_regs->config, XHCI_OP_CONFIG_MAX_SLOTS_EN,
      xhci->num_enabled_slots);

  /* Allocate memory and write the pointer to DCBAAP register */
  xhci_setup_dcbaa(xhci);

  /* Allocate Scratchpad Buffer memory for the xHC */
  xhci_setup_scratchpad(xhci);

  /* Setup Command Ring */
  xhci_setup_command_ring(xhci);

  /* Setup Event Ring */
  rc = xhci_setup_event_ring(xhci);
  grub_printf("xhci_setup_event_ring returned %d\n", rc);

  /* Interrupts is not supported by this driver, so skipped */

  /* Start the controller so that it accepts dorbell notifications.
   * We can run commands and the root hub ports will begin reporting device
   * connects etc.
   *
   * USB2 devices require the port reset process to advance the port to the
   * Enabled state. Once USB2 ports are Enabled, the port is active with SOFs
   * occurring on the port, but the Pipe Schedules have not yet been enabled.
   *
   * SS ports automatically advance to the Enabled state if a successful device
   * attach is detected.
   */
  mmio_write_bits(&xhci->oper_regs->usbcmd, XHCI_OP_USBCMD_RUNSTOP, 1);

  (void)rc;
  rc = xhci_nop(xhci);
  grub_printf("xhci_nop returned %d\n", rc);

  if (0 && xhci_debug_enabled())
  {
    xhci_dump_cap(xhci);
    xhci_dump_oper(xhci);
  }

#if 0

  /* TODO: initialize the various "rings" and TRBs */

  /* Determine and change ownership. */

  /* TODO: Check if handover is supported */
  addr = grub_pci_make_address (dev, GRUB_XHCI_CAP_HCCPARAMS1);
  hccparams1 = grub_pci_read(addr);
  xecp = hccparams1 >> 16;
  if (xecp)
    {
      addr = grub_pci_make_address (dev, 0);
      addr += sizeof(uint32_t) *
      hccparams1 = grub_pci_read(addr);
    }

  /* XECP offset valid in HCCPARAMS1 */
  /* Ownership can be changed via XECP only */
  if (eecp_offset >= 0x40)
  {
    grub_pci_address_t pciaddr_eecp;
    pciaddr_eecp = grub_pci_make_address (dev, eecp_offset);

    usblegsup = grub_pci_read (pciaddr_eecp);
    if (usblegsup & XHCI_USBLEGSUP_BIOS_OWNED)
      {
        grub_boot_time ("Taking ownership of xHCI controller");
        xhci_trace (
                      "xHCI grub_xhci_pci_iter: xHCI owned by: BIOS\n");
        /* Ownership change - set OS_OWNED bit */
        grub_pci_write (pciaddr_eecp, usblegsup | XHCI_USBLEGSUP_OS_OWNED);
        /* Ensure PCI register is written */
        grub_pci_read (pciaddr_eecp);

        /* Wait for finish of ownership change, xHCI specification
         * says it can take up to 16 ms
         */
        maxtime = grub_get_time_ms () + 1000;
        while ((grub_pci_read (pciaddr_eecp) & XHCI_USBLEGSUP_BIOS_OWNED)
               && (grub_get_time_ms () < maxtime));
        if (grub_pci_read (pciaddr_eecp) & XHCI_USBLEGSUP_BIOS_OWNED)
          {
            xhci_trace (
                          "xHCI grub_xhci_pci_iter: xHCI change ownership timeout");
            /* Change ownership in "hard way" - reset BIOS ownership */
            grub_pci_write (pciaddr_eecp, XHCI_USBLEGSUP_OS_OWNED);
            /* Ensure PCI register is written */
            grub_pci_read (pciaddr_eecp);
          }
      }
    else if (usblegsup & XHCI_USBLEGSUP_OS_OWNED)
      /* XXX: What to do in this case - nothing ? Can it happen ? */
      xhci_trace ("xHCI grub_xhci_pci_iter: xHCI owned by: OS\n");
    else
      {
        xhci_trace (
                      "xHCI grub_xhci_pci_iter: xHCI owned by: NONE\n");
        /* XXX: What to do in this case ? Can it happen ?
         * Is code below correct ? */
        /* Ownership change - set OS_OWNED bit */
        grub_pci_write (pciaddr_eecp, XHCI_USBLEGSUP_OS_OWNED);
        /* Ensure PCI register is written */
        grub_pci_read (pciaddr_eecp);
      }

    /* Disable SMI, just to be sure.  */
    pciaddr_eecp = grub_pci_make_address (dev, eecp_offset + 4);
    grub_pci_write (pciaddr_eecp, 0);
    /* Ensure PCI register is written */
    grub_pci_read (pciaddr_eecp);
  }

  xhci_trace ("inithw: xHCI grub_xhci_pci_iter: ownership OK\n");

  /* Now we can setup xHCI (maybe...) */

  /* Check if xHCI is halted and halt it if not */
  if (grub_xhci_halt (xhci) != GRUB_USB_ERR_NONE)
    {
      grub_error (GRUB_ERR_TIMEOUT,
		  "xHCI grub_xhci_pci_iter: xHCI halt timeout");
      goto fail;
    }

  xhci_trace ("xHCI grub_xhci_pci_iter: halted OK\n");

  /* Reset xHCI */
  if (grub_xhci_reset (xhci) != GRUB_USB_ERR_NONE)
    {
      grub_error (GRUB_ERR_TIMEOUT,
		  "xHCI grub_xhci_pci_iter: xHCI reset timeout");
      goto fail;
    }

  xhci_trace ("xHCI grub_xhci_pci_iter: reset OK\n");

  /* Now should be possible to power-up and enumerate ports etc. */
  if ((xhci_cap_read32 (xhci, GRUB_XHCI_EHCC_SPARAMS)
       & GRUB_XHCI_SPARAMS_PPC) != 0)
    {				/* xHCI has port powering control */
      /* Power on all ports */
      n_ports = xhci_cap_read32 (xhci, GRUB_XHCI_EHCC_SPARAMS)
	& GRUB_XHCI_SPARAMS_N_PORTS;
      for (i = 0; i < (int) n_ports; i++)
	grub_xhci_oper_write32 (xhci, XHCI_PORTSC(port),
				GRUB_XHCI_PORT_POWER
				| grub_xhci_oper_read32 (xhci,
							 XHCI_PORTSC(i)));
    }

  /* Ensure all commands are written */
  grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD);

  /* Enable xHCI */
  grub_xhci_oper_write32 (xhci, GRUB_XHCI_OPER_USBCMD,
			  XHCI_USBCMD_RUNSTOP
			  | grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD));

  /* Ensure command is written */
  grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD);

  /* Link to xhci now that initialisation is successful.  */
  xhci_list_add(xhci);

  sync_all_caches (xhci);

  xhci_trace ("xHCI grub_xhci_pci_iter: OK at all\n");

  xhci_trace (
		"xHCI grub_xhci_pci_iter: iobase of oper. regs: %08x\n",
		((unsigned int)xhci->iobase_oper));
  xhci_trace ("xHCI grub_xhci_pci_iter: USBCMD: %08x\n",
		grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD));
  xhci_trace ("xHCI grub_xhci_pci_iter: USBSTS: %08x\n",
		grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBSTS));

#endif
  return 0;

//fail:
//  if (xhci)
//    {
////      if (xhci->td_chunk)
////	grub_dma_free ((void *) xhci->td_chunk);
////      if (xhci->qh_chunk)
////	grub_dma_free ((void *) xhci->qh_chunk);
////      if (xhci->framelist_chunk)
////	grub_dma_free (xhci->framelist_chunk);
//    }
  grub_free (xhci);
  return 0;
}

/*
 * Read PCI BAR
 *
 * @v pci		PCI device
 * @v reg		PCI register number
 * @ret bar		Base address register
 *
 * Reads the specified PCI base address register, including the flags
 * portion.  64-bit BARs will be handled automatically.  If the value
 * of the 64-bit BAR exceeds the size of an unsigned long (i.e. if the
 * high dword is non-zero on a 32-bit platform), then the value
 * returned will be zero plus the flags for a 64-bit BAR.  Unreachable
 * 64-bit BARs are therefore returned as uninitialised 64-bit BARs.
 */
static unsigned long pci_bar ( struct grub_pci_device *dev, unsigned int reg ) {
  grub_uint32_t low;
  grub_uint32_t high;

  low = pci_config_read32 (*dev, reg);
  if ( ( low & (GRUB_PCI_ADDR_SPACE_IO|GRUB_PCI_ADDR_MEM_TYPE_MASK))
      == GRUB_PCI_ADDR_MEM_TYPE_64 )
    {
      high = pci_config_read32 (*dev, reg + 4);
      if ( high )
        {
          if ( sizeof ( unsigned long ) > sizeof ( grub_uint32_t ) ) {
            return ( ( ( grub_uint64_t ) high << 32 ) | low );
          }
        else
          {
            xhci_trace ("unhandled 64-bit BAR\n");
            return GRUB_PCI_ADDR_MEM_TYPE_64;
          }
        }
    }
  return low;
}

/**
 * Find the start of a PCI BAR
 *
 * @v pci		PCI device
 * @v reg		PCI register number
 * @ret start		BAR start address
 *
 * Reads the specified PCI base address register, and returns the
 * address portion of the BAR (i.e. without the flags).
 *
 * If the address exceeds the size of an unsigned long (i.e. if a
 * 64-bit BAR has a non-zero high dword on a 32-bit machine), the
 * return value will be zero.
 */
static unsigned long pci_bar_start ( struct grub_pci_device *dev, unsigned int reg )
{
  unsigned long bar;

  bar = pci_bar (dev, reg);
  if ( bar & GRUB_PCI_ADDR_SPACE_IO )
    {
      return ( bar & ~GRUB_PCI_ADDR_IO_MASK );
    }
  else
    {
      return ( bar & ~GRUB_PCI_ADDR_MEM_MASK );
    }
}

/** xHCI extended capability ID */
#define XHCI_XECP_ID(xecp) (((xecp) >> 0) & 0xff)

/**
 * Find extended capability
 *
 * @v xhci		xHCI device
 * @v cap_id		Capability ID
 * @v offset		Offset to previous extended capability instance, or zero
 * @ret extended capability register value, or 0 if not found
 */
static grub_uint32_t
xhci_get_extended_capability (struct xhci *xhci, unsigned int cap_id)
{
  grub_uint32_t xecp_offset;
  const volatile grub_uint32_t *xecp;
  grub_uint32_t cap;
  grub_size_t max_caps = 1024; /* guard against infinite loop */

  xecp_offset = mmio_read_bits(&xhci->cap_regs->hccparams1, XHCI_CAP_HCCPARAMS1_XECP);
  if (!xecp_offset)
    return 0;

  xecp = &xhci->cap_regs->caplength_and_hciversion + xecp_offset;

  /* Locate the extended capability */
  for (grub_size_t i=0; i<max_caps; i++)
  {
    /* Get current capability */
    cap = mmio_read32(xecp);
    if (XHCI_XECP_ID(cap) == cap_id)
    {
      xhci_dbg("Found capability id %d at position %d in xecp list\n");
      return cap;
    } else if (XHCI_XECP_ID(cap) == 0)
    {
      return 0;
    } else {
      xecp += (cap >> 8) & 0xff;
    }
  }

  xhci_err("xhci_get_extended_capability error\n");
  return 0;
}

static int xhci_extended_capabilities_foreach(struct xhci *xhci,
    int (*callback)(struct xhci *xhci, grub_uint32_t capreg))
{
  grub_uint32_t xecp_offset;
  const volatile grub_uint32_t *xecp;
  grub_uint32_t cap;
  grub_size_t max_caps = 1024; /* guard against infinite loop */
  int rc;

  xecp_offset = mmio_read_bits(&xhci->cap_regs->hccparams1, XHCI_CAP_HCCPARAMS1_XECP);
  if (!xecp_offset)
    return 0;

  xecp = &xhci->cap_regs->caplength_and_hciversion + xecp_offset;

  /* Locate the extended capability */
  for (grub_size_t i=0; i<max_caps; i++)
  {
    /* Get current capability */
    cap = mmio_read32(xecp);
    rc = callback(xhci, cap);
    if (rc)
    {
      return rc;
    } else if (XHCI_XECP_ID(cap) == 0)
    {
      return 0;
    } else {
      xecp += (cap >> 8) & 0xff;
    }
  }

  xhci_err("xhci_get_extended_capability error\n");
  return -1;
}

/** USB legacy support extended capability */
#define XHCI_XECP_ID_LEGACY 1

/**
 * Initialise USB legacy support
 *
 * @v xhci		xHCI device
 */
static void
xhci_legacy_init (struct xhci *xhci)
{
  grub_uint32_t legacy;
  //grub_uint8_t bios;

  /* Locate USB legacy support capability (if present) */
  legacy = xhci_get_extended_capability (xhci, XHCI_XECP_ID_LEGACY);
  if (!legacy) {
    /* Not an error; capability may not be present */
    xhci_dbg("XHCI-%s has no USB legacy support capability\n",
        xhci->name );
    return;
  }

  ///* Check if legacy USB support is enabled */
  //bios = readb ( xhci->cap + legacy + XHCI_USBLEGSUP_BIOS );
  //if ( ! ( bios & XHCI_USBLEGSUP_BIOS_OWNED ) ) {
  //  /* Not an error; already owned by OS */
  //  DBGC ( xhci, "XHCI %s USB legacy support already disabled\n",
  //      xhci->name );
  //  return;
  //}

  ///* Record presence of USB legacy support capability */
  //xhci->legacy = legacy;
}

/**
 * Claim ownership from BIOS
 *
 * @v xhci		xHCI device
 */
//static void xhci_legacy_claim ( struct xhci *xhci )
//{
//  grub_uint32_t ctlsts;
//  grub_uint8_t bios;
//  unsigned int i;
//
//  /* Do nothing unless legacy support capability is present */
//  if ( ! xhci->legacy )
//    return;
//
//  /* Claim ownership */
//  writeb ( XHCI_USBLEGSUP_OS_OWNED,
//      xhci->cap + xhci->legacy + XHCI_USBLEGSUP_OS );
//
//  /* Wait for BIOS to release ownership */
//  for ( i = 0 ; i < XHCI_USBLEGSUP_MAX_WAIT_MS ; i++ ) {
//
//    /* Check if BIOS has released ownership */
//    bios = readb ( xhci->cap + xhci->legacy + XHCI_USBLEGSUP_BIOS );
//    if ( ! ( bios & XHCI_USBLEGSUP_BIOS_OWNED ) ) {
//      DBGC ( xhci, "XHCI %s claimed ownership from BIOS\n",
//          xhci->name );
//      ctlsts = readl ( xhci->cap + xhci->legacy +
//          XHCI_USBLEGSUP_CTLSTS );
//      if ( ctlsts ) {
//        DBGC ( xhci, "XHCI %s warning: BIOS retained "
//            "SMIs: %08x\n", xhci->name, ctlsts );
//      }
//      return;
//    }
//
//    /* Delay */
//    mdelay ( 1 );
//  }
//
//  /* BIOS did not release ownership.  Claim it forcibly by
//   * disabling all SMIs.
//   */
//  DBGC ( xhci, "XHCI %s could not claim ownership from BIOS: forcibly "
//      "disabling SMIs\n", xhci->name );
//  writel ( 0, xhci->cap + xhci->legacy + XHCI_USBLEGSUP_CTLSTS );
//}
