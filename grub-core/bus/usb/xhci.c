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

#include <stdint.h> /* uint32_t */
#include <stdarg.h> /* va_list */

#include "xhci.h"
#include "xhci_io.h"
#include "xhci_private.h"

//static void
//mmio_set_bits(volatile uint32_t *addr, uint32_t bits)
//{
//  mmio_write32(addr, mmio_read32(addr) | bits);
//}

//static void
//mmio_clear_bits(volatile uint32_t *addr, uint32_t bits)
//{
//  mmio_write32(addr, mmio_read32(addr) & ~bits);
//}

static uint32_t
parse_reg(uint32_t regval, const enum bits32 bits)
{
  const uint32_t bitno = bits >> 16;
  const uint32_t width = bits & 0xff;

  regval >>= bitno;
  regval &= ((1 << width) - 1);
  return regval;
}

/**
 * Read a MMIO register. Masking and shifting is done automatically with
 * 'bits'.
 */
static uint32_t
mmio_read_bits(const volatile uint32_t *addr, const enum bits32 bits)
{
  uint32_t regval;

  regval = mmio_read32(addr);
  return parse_reg(regval, bits);
}

/* Return modified copy of regval, shifting and maskin 'val' according to
 * 'bits'.
 */
static uint32_t
build_reg(uint32_t regval, const enum bits32 bits, uint32_t val)
{
  const uint32_t bitno = bits >> 16;
  const uint32_t width = bits & 0xff;

  regval &= ~(((1 << width) - 1) << bitno);
  regval |= val << bitno;
  return regval;
}

/**
 * Write a MMIO register. Masking and shifting is done automatically with
 * 'bits'. The register is read first, so existing bits are preserved.
 */
static void
mmio_write_bits(volatile uint32_t *addr, const enum bits32 bits, uint32_t val)
{
  uint32_t regval;
  regval = mmio_read32(addr);
  mmio_write32(addr, build_reg(regval, bits, val));
}

/**
 * A printf function which prefixes output with "FILE:LINENO: "
 * Only output anything if the GRUB debug variable contains "all" or "xhci".
 */
static void xhci_trace(const char *fmt, ...)
{
  va_list args;

  if (xhci_debug_enabled())
    {
      /* TODO: create macro, this gets hardcoded to the current file/line! */
      xhci_printf ("%s:%d: ", __FILE__, __LINE__);
      va_start (args, fmt);
      xhci_vprintf (fmt, args);
      va_end (args);
    }
}

/**
 * A printf function which does not prefix output.
 * Only output anything if the GRUB debug variable contains "all" or "xhci".
 */
static void xhci_dbg(const char *fmt, ...)
{
  va_list args;

  if (xhci_debug_enabled())
    {
      va_start (args, fmt);
      xhci_vprintf (fmt, args);
      va_end (args);
    }
}

static void xhci_err(const char *fmt, ...)
{
  va_list args;

  va_start (args, fmt);
  xhci_vprintf (fmt, args);
  va_end (args);
}

/* Read Port Register Set n of given type */
static uint32_t xhci_read_portrs(struct xhci *xhci, unsigned int port, enum xhci_portrs_type type)
{
  static int num_warnings;
  uint8_t *addr;

  if (port > xhci->max_ports)
  {
    if (num_warnings == 0)
    {
      xhci_err ("too big port number (port=%d, max_ports=%d)\n", port, xhci->max_ports);
      num_warnings += 1;
    }
    return 0;
  }

  addr = (uint8_t*)xhci->oper_regs + 0x400 + (0x10 * (port - 0)) + type;
  return mmio_read32 ((uint32_t *)addr);
}

/* Read Port Register Set n of given type */
static void xhci_write_portrs(struct xhci *xhci, unsigned int port, enum xhci_portrs_type type, uint32_t value)
{
  static int num_warnings;
  uint8_t *addr;

  if (port > xhci->max_ports)
  {
    if (num_warnings == 0)
    {
      xhci_err ("too big port number (port=%d, max_ports=%d)\n", port, xhci->max_ports);
      num_warnings += 1;
    }
    return;
  }

  addr = (uint8_t*)xhci->oper_regs + 0x400 + (0x10 * (port - 1)) + type;
  mmio_write32 ((uint32_t *)addr, value);
}

/* Convert raw PAGESIZE value from Operational Register to a size in bytes */
static size_t xhci_pagesize_to_bytes(int pagesize)
{
  return 1 << (pagesize + 12);
}

static void xhci_dump_oper_portsc(struct xhci *xhci, int port)
{
  uint32_t portsc;
  char pls_str[16];
  char ps_str[16];

  portsc = xhci_read_portrs (xhci, port, PORTSC);

  xhci_snprintf(pls_str, sizeof (pls_str), " PLS=%d",
      parse_reg(portsc, XHCI_OP_PORTSC_PLS));

  xhci_snprintf(ps_str, sizeof (ps_str), " PS=%d",
      parse_reg(portsc, XHCI_OP_PORTSC_PS));

  xhci_printf (" PORTSC(%02d)=0x%08x%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s",
      port, portsc
      , parse_reg(portsc, XHCI_OP_PORTSC_CCS) ? " CCS" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_PED) ? " PED" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_OCA) ? " OCA" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_PR) ? " PR" : ""
      , pls_str
      , parse_reg(portsc, XHCI_OP_PORTSC_PP) ? " PP" : ""
      , ps_str
      , parse_reg(portsc, XHCI_OP_PORTSC_PIC) ? " PIC" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_LWS) ? " LWS" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_CSC) ? " CSC" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_PEC) ? " PEC" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_WRC) ? " WRC" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_OCC) ? " OCC" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_PRC) ? " PRC" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_PLC) ? " PLC" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_CEC) ? " CEC" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_CAS) ? " CAS" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_WCE) ? " WCE" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_WDE) ? " WDE" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_WOE) ? " WOE" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_DR) ? " DR" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_WPR) ? " WPR" : ""
      );
}

static int xhci_dump_oper(struct xhci *xhci)
{
  int i;
  uint32_t val32;
  char extra[256];

  if (!xhci_debug_enabled())
    return 0;

  val32 = mmio_read32(&xhci->oper_regs->usbcmd);
  xhci_snprintf (extra, sizeof (extra),
		 "%s%s"
                 , parse_reg(val32, XHCI_OP_USBCMD_RUNSTOP) ? " RUN" : ""
                 , parse_reg(val32, XHCI_OP_USBCMD_HCRST) ? " HCRST" : ""
      );
  xhci_dbg ("USBCMD=0x%08x%s\n",
      val32, extra);

  val32 = mmio_read32(&xhci->oper_regs->usbsts);
  xhci_snprintf (extra, sizeof (extra),
		 "%s%s%s%s%s%s%s%s%s"
                 , parse_reg(val32, XHCI_OP_USBSTS_HCH) ? " HCH" : ""
                 , parse_reg(val32, XHCI_OP_USBSTS_HSE) ? " HCE" : ""
                 , parse_reg(val32, XHCI_OP_USBSTS_EINT) ? " EINT" : ""
                 , parse_reg(val32, XHCI_OP_USBSTS_PCD) ? " PCD" : ""
                 , parse_reg(val32, XHCI_OP_USBSTS_SSS) ? " SSS" : ""
                 , parse_reg(val32, XHCI_OP_USBSTS_RSS) ? " RSS" : ""
                 , parse_reg(val32, XHCI_OP_USBSTS_SRE) ? " SRE" : ""
                 , parse_reg(val32, XHCI_OP_USBSTS_CNR) ? " CNR" : ""
                 , parse_reg(val32, XHCI_OP_USBSTS_HCE) ? " HCE" : ""
      );
  xhci_dbg ("USBSTS=0x%08x%s\n", val32, extra);

  val32 = mmio_read32 (&xhci->oper_regs->pagesize);
  xhci_dbg ("PAGESIZE=%d (%d bytes)\n",
      val32, xhci_pagesize_to_bytes(val32));

  xhci_dbg ("DNCTRL=0x%08x\n",
      mmio_read32 (&xhci->oper_regs->dnctrl));

  xhci_dbg ("CRCR=0x%08lx\n", /* TODO: implement PRIxuint64_T like macros in standard header */
      mmio_read64 (&xhci->oper_regs->crcr));

  xhci_dbg ("DCBAAP=0x%08lx\n",
      mmio_read64 (&xhci->oper_regs->dcbaap));

  xhci_dbg ("CONFIG=0x%08x\n",
      mmio_read32 (&xhci->oper_regs->config));

  xhci_printf ("PORTSC registers:\n");
  for (i = 0; i < xhci->max_ports; i++)
  {
    xhci_dump_oper_portsc(xhci, i);
    if ((i+1) % 5 == 0)
    {
      xhci_printf ("\n");
    }
  }
  xhci_printf ("\n");

  return 0;
}

static int xhci_dump_cap(struct xhci *xhci)
{
  if (!xhci_debug_enabled())
    return 0;

  xhci_dbg ("CAPLENGTH=%d\n",
      mmio_read_bits (&xhci->cap_regs->caplength_and_hciversion,
        XHCI_CAP_CAPLENGTH));

  xhci_dbg ("HCIVERSION=0x%04x\n",
      mmio_read_bits (&xhci->cap_regs->caplength_and_hciversion,
        XHCI_CAP_HCIVERSION));

  xhci_dbg ("HCSPARAMS1=0x%08x\n",
      mmio_read32 (&xhci->cap_regs->hcsparams1));

  xhci_dbg ("HCSPARAMS2=0x%08x\n",
      mmio_read32 (&xhci->cap_regs->hcsparams2));

  xhci_dbg ("HCSPARAMS3=0x%08x\n",
      mmio_read32 (&xhci->cap_regs->hcsparams3));

  xhci_dbg ("HCCPARAMS1=0x%08x\n",
      mmio_read32 (&xhci->cap_regs->hccparams1));

  xhci_dbg ("DBOFF=0x%08x\n",
      RTSOFF_TO_BYTES(mmio_read_bits (&xhci->cap_regs->dboff,
          XHCI_CAP_DBOFF)));

  xhci_dbg ("RTSOFF=0x%08x\n",
      RTSOFF_TO_BYTES(mmio_read_bits (&xhci->cap_regs->rtsoff,
          XHCI_CAP_RTSOFF)));

  xhci_dbg ("HCCPARAMS2=0x%08x\n",
      mmio_read32 (&xhci->cap_regs->hccparams2));

  return 0;
}

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

static void xhci_doorbell_notify (struct xhci_trb_ring *ring)
{
  // TODO: write barrier
  mmio_write32(ring->db_reg, ring->db_val);
}

/*
 * Wait until value at address "addr" matches given bits value. Return 0 on
 * error, >= 1 on success.
 */
static int xhci_handshake(volatile uint32_t *addr, enum bits32 bits,
    uint32_t value, int timeout_ms)
{
  while ((mmio_read_bits(addr, bits) != value && timeout_ms--))
  {
    xhci_mdelay(1);
  }
  return timeout_ms;
}

static int
xhci_wait_ready(struct xhci *xhci)
{
  if (!xhci_handshake(&xhci->oper_regs->usbsts, XHCI_OP_USBSTS_CNR, 0, 100)) {
    xhci_dbg("controller not ready timeout (100ms)!\n");
    return -1;
  }
  return 0;
}

static int
xhci_start (struct xhci *xhci)
{
  mmio_write_bits(&xhci->oper_regs->usbcmd, XHCI_OP_USBCMD_RUNSTOP, 1);
  if (!xhci_handshake(&xhci->oper_regs->usbsts, XHCI_OP_USBSTS_HCH, 0, 1000))
  {
    xhci_dbg("Controller didn't start within 1s\n");
    return -1;
  }
  return 0;
}

static int
xhci_stop (struct xhci *xhci)
{
  mmio_write_bits(&xhci->oper_regs->usbcmd, XHCI_OP_USBCMD_RUNSTOP, 0);
  if (!xhci_handshake(&xhci->oper_regs->usbsts, XHCI_OP_USBSTS_HCH, 1, 1000))
  {
    xhci_dbg("Controller didn't stop within 1s\n");
    return -1;
  }
  return 0;
}

/* xHCI HC reset */
static int
xhci_reset (struct xhci *xhci)
{
  mmio_write_bits(&xhci->oper_regs->usbcmd, XHCI_OP_USBCMD_HCRST, 1);
  if (!xhci_handshake(&xhci->oper_regs->usbcmd, XHCI_OP_USBCMD_HCRST, 1, 1000))
  {
    return -1;
  }
  return 0;
}

/* Start the xHC by setting the RUN bit and wait for the controller to
 * acknowledge.
 */
static void
xhci_run (struct xhci *xhci)
{
  (void)xhci;
  //uint32_t config;
  //uint32_t usbcmd;

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
  xhci_dbg("sync_all_caches enter\n");
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
  xhci_dbg ("xhci_cancel_transfer: begin\n");
  return 0;

#if 0
  size_t actual;
  int i;
  uint64_t maxtime;
  uint32_t qh_phys;

  sync_all_caches (xhci);

  uint32_t interrupt =
    cdata->qh_virt->ep_cap & GRUB_XHCI_SMASK_MASK;

  /* QH can be active and should be de-activated and halted */

  xhci_dbg ("cancel_transfer: begin\n");

  /* First check if xHCI is running - if not, there is no problem */
  /* to cancel any transfer. Or, if transfer is asynchronous, check */
  /* if AL is enabled - if not, transfer can be canceled also. */
  if (((grub_xhci_oper_read32 (xhci, GRUB_XHCI_STATUS) &
      GRUB_XHCI_ST_HC_HALTED) != 0) ||
    (!interrupt && ((grub_xhci_oper_read32 (xhci, GRUB_XHCI_STATUS) &
      (GRUB_XHCI_ST_AS_STATUS | GRUB_XHCI_ST_PS_STATUS)) == 0)))
    {
      grub_xhci_pre_finish_transfer (transfer);
      xhci_free (cdata);
      sync_all_caches (xhci);
      xhci_dbg ("cancel_transfer: end - xHCI not running\n");
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
      xhci_printf ("%s: prev not found, queues are corrupt\n", __func__);
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
      mdelay(20);
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

  xhci_free (cdata);

  xhci_dbg ("cancel_transfer: end\n");

  sync_all_caches (xhci);

#endif
  return 0;
}

enum xhci_speed
xhci_detect_dev (struct xhci *xhci, int port, int *changed)
{
  uint32_t status, line_state;

  (void)port;
  (void)changed;
  (void)xhci;
  (void)line_state;
  (void)status;
  uint32_t portsc;

  //xhci_dbg ("xhci_detect_dev port=%d\n", port);
  portsc = xhci_read_portrs (xhci, port, PORTSC);
  if (parse_reg(portsc, XHCI_OP_PORTSC_CCS))
  {
    /* TODO: */
    *changed = 1;
    return XHCI_SPEED_SUPER;
  }
  else
  {
    *changed = 0;
    return XHCI_SPEED_NONE;
  }

  return XHCI_SPEED_NONE;

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
  return XHCI_SPEED_NONE;
}

int
xhci_portstatus (struct xhci *xhci,
		 unsigned int port, unsigned int enable)
{
  (void)xhci;
  (void)port;
  (void)enable;
  //xhci_dbg ("xhci_portstatus enter (port=%d, enable=%d)\n",
  //    port, enable);
  return 0;

#if 0
  uint64_t endtime;

  xhci_dbg ("portstatus: xHCI USBSTS: %08x\n",
		grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBSTS));
  xhci_dbg (
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
      xhci_dbg ("portstatus: Disabled.\n");
      xhci_dbg ("portstatus: end, status=0x%02x\n",
		    grub_xhci_port_read (xhci, port));
      return GRUB_USB_ERR_NONE;
    }

  xhci_dbg ("portstatus: enable\n");

  grub_boot_time ("Resetting port %d", port);

  /* Now we will do reset - if HIGH speed device connected, it will
   * result in Enabled state, otherwise port remains disabled. */
  /* Set RESET bit for 50ms */
  grub_xhci_port_setbits (xhci, port, GRUB_XHCI_PORT_RESET);
  mdelay (50);

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
      xhci_dbg ("portstatus: Enabled!\n");
      /* "Reset recovery time" (USB spec.) */
      mdelay (10);
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

  xhci_dbg ("portstatus: end, status=0x%02x\n",
		grub_xhci_port_read (xhci, port));

#endif
  return 0;
}

int
xhci_hubports (struct xhci *xhci)
{
  xhci_dbg ("xhci_hubports: max_ports=%d\n", xhci->max_ports);
  return xhci->max_ports;
}

int
xhci_check_transfer (struct xhci *xhci)
{
  (void)xhci;

  //xhci_dbg ("xhci_check_transfer enter (TODO: implement)\n");
  return 0;
#if 0
  uint32_t token, token_ftd;

  sync_all_caches (xhci);

  xhci_dbg (
		"check_transfer: xHCI STATUS=%08x, cdata=%p, qh=%p\n",
		grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBSTS),
		cdata, cdata->qh_virt);
  xhci_dbg ("check_transfer: qh_hptr=%08x, ep_char=%08x\n",
		grub_le_to_cpu32 (cdata->qh_virt->qh_hptr),
		grub_le_to_cpu32 (cdata->qh_virt->ep_char));
  xhci_dbg ("check_transfer: ep_cap=%08x, td_current=%08x\n",
		grub_le_to_cpu32 (cdata->qh_virt->ep_cap),
		grub_le_to_cpu32 (cdata->qh_virt->td_current));
  xhci_dbg ("check_transfer: next_td=%08x, alt_next_td=%08x\n",
		grub_le_to_cpu32 (cdata->qh_virt->td_overlay.next_td),
		grub_le_to_cpu32 (cdata->qh_virt->td_overlay.alt_next_td));
  xhci_dbg ("check_transfer: token=%08x, buffer[0]=%08x\n",
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
  return -1;
}


int
xhci_setup_transfer (struct xhci *xhci)
{
  (void)xhci;
  //xhci_dbg ("xhci_setup_transfer enter (TODO: implement)\n");
  /* pretend we managed to start sending data */
  return 0;

#if 0
  struct xhci *xhci = (struct xhci *) dev->data;
  grub_xhci_td_t td = NULL;
  grub_xhci_td_t td_prev = NULL;
  int i;
  struct grub_xhci_transfer_controller_data *cdata;
  uint32_t status;

  sync_all_caches (xhci);

  /* Check if xHCI is running and AL is enabled */
  status = grub_xhci_oper_read32 (xhci, GRUB_XHCI_STATUS);
  if ((status & GRUB_XHCI_ST_HC_HALTED) != 0)
    /* XXX: Fix it: Currently we don't do anything to restart xHCI */
    {
      xhci_dbg ("setup_transfer: halted, status = 0x%x\n",
		    status);
      return GRUB_USB_ERR_INTERNAL;
    }
  status = grub_xhci_oper_read32 (xhci, GRUB_XHCI_STATUS);
  if ((status
       & (GRUB_XHCI_ST_AS_STATUS | GRUB_XHCI_ST_PS_STATUS)) == 0)
    /* XXX: Fix it: Currently we don't do anything to restart xHCI */
    {
      xhci_dbg ("setup_transfer: no AS/PS, status = 0x%x\n",
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
      xhci_dbg ("setup_transfer: no QH\n");
      xhci_free (cdata);
      return GRUB_USB_ERR_INTERNAL;
    }

  /* To detect short packet we need some additional "alternate" TD,
   * allocate it first. */
  cdata->td_alt_virt = 0; //grub_xhci_alloc_td (xhci);
  if (!cdata->td_alt_virt)
    {
      xhci_dbg ("setup_transfer: no TDs\n");
      xhci_free (cdata);
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
	  size_t actual = 0;

	  if (cdata->td_first_virt)
	    grub_xhci_free_tds (xhci, cdata->td_first_virt, NULL, &actual);

	  xhci_free (cdata);
	  xhci_dbg ("setup_transfer: no TD\n");
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

  xhci_dbg ("setup_transfer: cdata=%p, qh=%p\n",
		cdata,cdata->qh_virt);
  xhci_dbg ("setup_transfer: td_first=%p, td_alt=%p\n",
		cdata->td_first_virt,
		cdata->td_alt_virt);
  xhci_dbg ("setup_transfer: td_last=%p\n",
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
  return 0;
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
      cpu_to_le32 ( cycle ) );

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
    xhci_mdelay (1);
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
int xhci_nop(struct xhci *xhci)
{
  union xhci_trb trb;
  struct xhci_trb_common *nop = &trb.common;
  int rc;

  /* Construct command */
  xhci_memset(nop, 0, sizeof (*nop));
  nop->flags = XHCI_TRB_IOC;
  nop->type = XHCI_TRB_TYPE_NO_OP_COMMAND;

  /* Issue command and wait for completion */
  rc = xhci_command(xhci, &trb);
  if (rc != 0)
    return rc;

  return 0;
}

int xhci_status(struct xhci *xhci, int verbose)
{
  uint32_t portsc;

  /* Print connected ports */
  xhci_printf("XHCI-%s cap=0x%08x PORTSC(n):", xhci->name, xhci->cap_regs);
  for (int port=0; port<xhci->max_ports; port++)
  {
    portsc = xhci_read_portrs (xhci, port, PORTSC);
    if (parse_reg(portsc, XHCI_OP_PORTSC_CCS))
    {
      xhci_printf(" %d", port);
    }
  }
  xhci_printf("\n");

  if (verbose)
  {
    /* Full dump of PORTSC regs */
    for (int i=0; i<xhci->max_ports; i++)
    {
      xhci_dump_oper_portsc(xhci, i);
      xhci_printf ("\n");
    }
  }

  return 0;
}

/**
 * Determine alignment requirement for an xHCI data structure that must not
 * cross a page boundary. Solved by rounding up to nearest power of 2 size.
 *
 * The only datastructure with alignment of more than 64 bytes is the
 * scratchpad buffers, but they have only one size:
 *
 *   size = pagesize = alignment = boundary
 */
static size_t xhci_align (size_t size)
{
  const size_t min_align = 64;
  size_t align;
  int n;

  if (size < min_align || size == 0)
  {
    align = min_align;
  }
  else
  {
    /* Align to own length rounded up to a power of two */
    size -= 1;
    n = 1;
    while (size >>= 1)
      n++;
    align = 1 << n;
  }

  return align;
}

/*
 * Allocate DMA memory while taking care of alignment and boundary
 * requirements.
 */
static void *xhci_memalign(size_t size)
{
  void *ptr;
  ptr = xhci_dma_alloc(xhci_align(size), size);
  if (!ptr)
    xhci_err("xhci: out of memory (failed to allocate %d bytes)\n", size);

  return ptr;
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
  if (xhci->dcbaa)
  {
    xhci_err ("dcbaa non-zero, possibly memory leak\n");
    return -1;
  }

  /* +1 to make room for the scratchpad pointer, at index 0 */
  xhci->dcbaa_len = (xhci->max_device_slots + 1) * sizeof (xhci->dcbaa[0]);
  xhci->dcbaa = xhci_memalign(xhci->dcbaa_len);
  if (!xhci->dcbaa)
    return -1;

  xhci_memset(xhci->dcbaa, 0, xhci->dcbaa_len);
  return 0;
}

/** Program xHC DCBAAP register with physical DCBAA from 'xhci' instance */
static int
xhci_program_dcbaap(struct xhci *xhci)
{
  uint32_t dcbaa_phys;

  /* only 32-bit support */
  dcbaa_phys = xhci_dma_get_phys(xhci->dcbaa);
  xhci_dbg ("DCBAA at 0x%08x (virt 0x%08x), len=%d\n",
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
  int i;
  uint32_t scratchpad_phys;
  uint32_t scratchpad_arr_phys;

  xhci->num_scratch_bufs = mmio_read_bits(&xhci->cap_regs->hcsparams2,
      XHCI_CAP_HCSPARAMS2_MAX_SCRATCH_BUFS_LO)
    | (mmio_read_bits(&xhci->cap_regs->hcsparams2,
      XHCI_CAP_HCSPARAMS2_MAX_SCRATCH_BUFS_HI) << 5);

  if (!xhci->num_scratch_bufs)
  {
    xhci_dbg("xHC needs %d scratchpad buffers\n", xhci->num_scratch_bufs);
    return 0;
  }

  /* Allocate Scratchpad Buffers */
  xhci->pagesize = xhci_pagesize_to_bytes(
      mmio_read_bits(&xhci->oper_regs->pagesize, XHCI_OP_PAGESIZE));
  xhci->scratchpads_len = xhci->num_scratch_bufs * xhci->pagesize;
  xhci->scratchpads = xhci_memalign(xhci->scratchpads_len);
  if (!xhci->scratchpads)
  {
    return -1;
  }
  xhci_memset(xhci->scratchpads, 0, xhci->scratchpads_len);

  /* Allocate Scratchpad Buffer Array, where each element points to a buffer */
  xhci->scratchpad_arr_len = xhci->num_scratch_bufs * sizeof (xhci->scratchpad_arr[0]);
  xhci->scratchpad_arr = xhci_memalign(xhci->scratchpad_arr_len);
  if (!xhci->scratchpad_arr)
    return -1;
  xhci_memset(xhci->scratchpad_arr, 0, xhci->scratchpad_arr_len);

  /* Fill Scratchpad Buffers Array with addresses of the scratch buffers */
  for (i = 0; i < xhci->num_scratch_bufs; i++)
  {
    scratchpad_phys = xhci_dma_get_phys(xhci->scratchpads + xhci->pagesize * i);
    xhci->scratchpad_arr[i] = scratchpad_phys;
  }

  /* Write Scratchpad Buffers Array base address to xHC */
  scratchpad_arr_phys = xhci_dma_get_phys(xhci->scratchpad_arr);
   /* The location of the Scratcphad Buffer array is defined by entry 0 of the
    * DCBAA. We only support 32-bit.
    */
  mmio_write32((uint32_t*)&xhci->dcbaa, scratchpad_arr_phys);

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
  int count = 1 << shift;
  int len = (count + 1 /* Link TRB */) * sizeof (ring->trbs[0]);
  (void)xhci;
  //xhci_dbg("%s: TODO: implement\n", __func__);
  ring->mask = count - 1;
  ring->shift = shift;
  ring->slot = slot;
  ring->db_reg = &xhci->db_regs->doorbell[slot];
  ring->db_val = XHCI_DBVAL(target, stream);

  /* Allocate (physically contiguous) memory for the TRBs */
  ring->trbs = xhci_memalign(len);

  return -1;
}

/** Allocate command ring memory and program xHC */
static int
xhci_setup_command_ring(struct xhci *xhci)
{
  int ring_len_log2 = 3; // 2**ring_len_log2 entries
  uintptr_t trbs_phys;

  xhci_setup_ring(xhci, &xhci->command_ring, ring_len_log2, 0, 0, 0);
  trbs_phys = xhci_dma_get_phys((void*)xhci->command_ring.trbs);
  mmio_write64(&xhci->oper_regs->crcr, trbs_phys | BIT(XHCI_OP_CRCR_RCS));
  return 0;
}

static int
xhci_setup_event_ring(struct xhci *xhci)
{
  struct xhci_event_ring *event = &xhci->event_ring;
  unsigned int count;
  size_t len;

  /* Allocate event ring */
  count = 1 << XHCI_EVENT_TRBS_LOG2;
  len = count * sizeof (event->trb[0]);
  event->trb = xhci_memalign(len);
  if (!event->trb) {
    return -1;
  }

  for (unsigned int i = 0; i < count; i++)
  {
    event->trb[i] = (volatile union xhci_trb){0};
  }

  /* Allocate event ring segment table */
  event->segment = xhci_memalign(sizeof (event->segment[0]));
  if (!event->segment) {
    return -1;
  }

  event->segment[0] = (volatile struct xhci_event_ring_segment){0};
  event->segment[0].base = cpu_to_le64(xhci_dma_get_phys((void*)event->trb));
  event->segment[0].count = cpu_to_le32(count);

  /* Program event ring registers */
  mmio_write32(&xhci->run_regs->ir_set[0].erstsz, 1);
  mmio_write64(&xhci->run_regs->ir_set[0].erdp, xhci_dma_get_phys((void*)event->trb));
  mmio_write64(&xhci->run_regs->ir_set[0].erstba, xhci_dma_get_phys((void*)event->segment));

  xhci_dbg("event ring [%08lx,%08lx) table [%08lx,%08lx)\n",
      xhci_dma_get_phys((void*)event->trb),
      (xhci_dma_get_phys((void*)event->trb) + len),
      xhci_dma_get_phys((void*)event->segment),
      (xhci_dma_get_phys((void*)event->segment) + sizeof (event->segment[0])));
  return 0;

  /* TODO: free resources in case of error? */
}

struct xhci *xhci_create (volatile void *mmio_base_addr, int seqno)
{
  int rc;
  struct xhci *xhci = NULL;
  int32_t hcsparams1;
  //uint32_t hcsparams2;
  //uint32_t hccparams1;

  /* Sanity check register addresses.
   * No limits.h or CHAR_BIT available, use GRUB_CHAR_BIT.
   */
  //COMPILE_TIME_ASSERT(GRUB_CHAR_BIT == 8); /* cannot depend on GRUB in this file */
  COMPILE_TIME_ASSERT(OFFSETOF(struct xhci_cap_regs, hccparams2) == 0x1c);
  COMPILE_TIME_ASSERT(OFFSETOF(struct xhci_oper_regs, config) == 0x38);

  xhci = xhci_calloc (1, sizeof (struct xhci));
  xhci_snprintf(xhci->name, sizeof(xhci->name), "%d", seqno);
  //xhci->sbrn = pci_config_read8 (dev, XHCI_PCI_SBRN_REG);

  /* Locate capability, operational, runtime, and doorbell registers */
  xhci->cap_regs = mmio_base_addr;
  xhci->oper_regs = (struct xhci_oper_regs *)
    ((uint8_t *)xhci->cap_regs +
     mmio_read_bits (&xhci->cap_regs->caplength_and_hciversion,
       XHCI_CAP_CAPLENGTH));
  xhci->run_regs = (struct xhci_run_regs *)
    ((uint8_t *)xhci->cap_regs +
     RTSOFF_TO_BYTES(mmio_read_bits (&xhci->cap_regs->rtsoff, XHCI_CAP_RTSOFF)));
  xhci->db_regs = (struct xhci_doorbell_regs *)
    ((uint8_t *)xhci->cap_regs +
     DBOFF_TO_BYTES(mmio_read_bits (&xhci->cap_regs->dboff, XHCI_CAP_DBOFF)));

  if (xhci_wait_ready(xhci))
    return NULL;

  /* Get some structural info */
  hcsparams1 = mmio_read32 (&xhci->cap_regs->hcsparams1);
  xhci->max_device_slots = parse_reg(hcsparams1, XHCI_CAP_HCSPARAMS1_MAX_DEVICE_SLOTS);
  xhci->max_ports = parse_reg(hcsparams1, XHCI_CAP_HCSPARAMS1_MAX_PORTS);
  xhci_dbg("xhci_create: max_ports=%d\n", xhci->max_ports);

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
  xhci_setup_event_ring(xhci);

  /* Interrupts is not supported by this driver, so skipped */

  /* TODO: Take ownership of controller from BIOS, if supported */
  /* Initialise USB legacy support and claim ownership */
  //xhci_legacy_init(xhci);
  //xhci_legacy_claim(xhci);
  //xhci_extended_capabilities_foreach(xhci);

  xhci_dbg("XHCI-%s: cap=0x%08x oper=0x%08x run=0x%08x db=0x%08x\n",
      xhci->name, xhci->cap_regs, xhci->oper_regs, xhci->run_regs, xhci->db_regs);
  xhci->ac64 = mmio_read_bits(&xhci->cap_regs->hccparams1, XHCI_CAP_HCCPARAMS1_AC64);
  xhci_dbg("XHCI-%s: scratch_bufs=%d (arr @ 0x%08x) pagesize=%d AC64=%d\n",
      xhci->name, xhci->num_scratch_bufs, xhci->scratchpad_arr,
      xhci->pagesize, xhci->ac64);

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
  xhci_start(xhci);

  /* DEBUG */
  (void)rc;
  rc = xhci_nop(xhci);
  xhci_printf("xhci_nop returned %d\n", rc);

  if (0 && xhci_debug_enabled())
  {
    xhci_dump_cap(xhci);
    xhci_dump_oper(xhci);
  }

  return xhci;
}

void xhci_destroy (struct xhci *xhci)
{
  xhci_stop (xhci);
  xhci_reset (xhci);
  /* TODO: free sub-datastructures from xhci first */
  xhci_free (xhci);
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
static uint32_t
xhci_get_extended_capability (struct xhci *xhci, unsigned int cap_id)
{
  uint32_t xecp_offset;
  const volatile uint32_t *xecp;
  uint32_t cap;
  size_t max_caps = 1024; /* guard against infinite loop */

  xecp_offset = mmio_read_bits(&xhci->cap_regs->hccparams1, XHCI_CAP_HCCPARAMS1_XECP);
  if (!xecp_offset)
    return 0;

  xecp = &xhci->cap_regs->caplength_and_hciversion + xecp_offset;

  /* Locate the extended capability */
  for (size_t i=0; i<max_caps; i++)
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
    int (*callback)(struct xhci *xhci, uint32_t capreg))
{
  uint32_t xecp_offset;
  const volatile uint32_t *xecp;
  uint32_t cap;
  size_t max_caps = 1024; /* guard against infinite loop */
  int rc;

  xecp_offset = mmio_read_bits(&xhci->cap_regs->hccparams1, XHCI_CAP_HCCPARAMS1_XECP);
  if (!xecp_offset)
    return 0;

  xecp = &xhci->cap_regs->caplength_and_hciversion + xecp_offset;

  /* Locate the extended capability */
  for (size_t i=0; i<max_caps; i++)
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
  uint32_t legacy;
  //uint8_t bios;

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
//  uint32_t ctlsts;
//  uint8_t bios;
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
//    xhci_mdelay ( 1 );
//  }
//
//  /* BIOS did not release ownership.  Claim it forcibly by
//   * disabling all SMIs.
//   */
//  DBGC ( xhci, "XHCI %s could not claim ownership from BIOS: forcibly "
//      "disabling SMIs\n", xhci->name );
//  writel ( 0, xhci->cap + xhci->legacy + XHCI_USBLEGSUP_CTLSTS );
//}
