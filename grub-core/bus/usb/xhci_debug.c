#include <grub/misc.h> /* grub_snprintf */
#include <grub/env.h> /* grub_env_get */
#include <grub/term.h> /* grub_refresh */

#include "xhci_io.h"
#include "xhci_debug.h"
#include "xhci_private.h"

int xhci_debug_enabled(void)
{
  const char *debug = grub_env_get ("debug");

  return debug &&
    (grub_strword (debug, "all") || grub_strword (debug, "xhci"));
}

/**
 * A printf function which prefixes output with "FILE:LINENO: "
 * Only output anything if the GRUB debug variable contains "all" or "xhci".
 */
void xhci_trace(const char *fmt, ...)
{
  va_list args;

  if (xhci_debug_enabled())
    {
      grub_printf ("%s:%d: ", GRUB_FILE, __LINE__);
      va_start (args, fmt);
      grub_vprintf (fmt, args);
      va_end (args);
      grub_refresh ();
    }
}

/**
 * A printf function which does not prefix output.
 * Only output anything if the GRUB debug variable contains "all" or "xhci".
 */
void xhci_dbg(const char *fmt, ...)
{
  va_list args;

  if (xhci_debug_enabled())
    {
      va_start (args, fmt);
      grub_vprintf (fmt, args);
      va_end (args);
      grub_refresh ();
    }
}

void xhci_err(const char *fmt, ...)
{
  va_list args;

  va_start (args, fmt);
  grub_vprintf (fmt, args);
  va_end (args);
  grub_refresh ();
}

void xhci_dump_oper_portsc(struct xhci *xhci, int port)
{
  grub_uint32_t portsc;
  char pls_str[16];
  char ps_str[16];

  portsc = xhci_read_portrs (xhci, port, PORTSC);

  grub_snprintf(pls_str, sizeof (pls_str), " PLS=%d",
      parse_reg(portsc, XHCI_OP_PORTSC_PLS));

  grub_snprintf(ps_str, sizeof (ps_str), " PS=%d",
      parse_reg(portsc, XHCI_OP_PORTSC_PS));

  grub_printf (" PORTSC(%02d)=0x%08x%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s",
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

int xhci_dump_oper(struct xhci *xhci)
{
  int i;
  grub_uint32_t val32;
  char extra[256];

  if (!xhci_debug_enabled())
    return 0;

  val32 = mmio_read32(&xhci->oper_regs->usbcmd);
  grub_snprintf (extra, sizeof (extra),
		 "%s%s"
                 , parse_reg(val32, XHCI_OP_USBCMD_RUNSTOP) ? " RUN" : ""
                 , parse_reg(val32, XHCI_OP_USBCMD_HCRST) ? " HCRST" : ""
      );
  xhci_trace ("USBCMD=0x%08x%s\n",
      val32, extra);

  val32 = mmio_read32(&xhci->oper_regs->usbsts);
  grub_snprintf (extra, sizeof (extra),
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
  xhci_trace ("USBSTS=0x%08x%s\n", val32, extra);

  val32 = mmio_read32 (&xhci->oper_regs->pagesize);
  xhci_trace ("PAGESIZE=%d (%d bytes)\n",
      val32, xhci_pagesize_to_bytes(val32));

  xhci_trace ("DNCTRL=0x%08x\n",
      mmio_read32 (&xhci->oper_regs->dnctrl));

  xhci_trace ("CRCR=0x%08" PRIxGRUB_UINT64_T "\n",
      mmio_read64 (&xhci->oper_regs->crcr));

  xhci_trace ("DCBAAP=0x%08" PRIxGRUB_UINT64_T "\n",
      mmio_read64 (&xhci->oper_regs->dcbaap));

  xhci_trace ("CONFIG=0x%08x\n",
      mmio_read32 (&xhci->oper_regs->config));

  grub_printf ("PORTSC registers:\n");
  for (i = 0; i < xhci->max_ports; i++)
  {
    xhci_dump_oper_portsc(xhci, i);
    if ((i+1) % 5 == 0)
    {
      grub_printf ("\n");
    }
  }
  grub_printf ("\n");

  return 0;
}

int xhci_dump_cap(struct xhci *xhci)
{
  if (!xhci_debug_enabled())
    return 0;

  xhci_trace ("CAPLENGTH=%d\n",
      mmio_read_bits (&xhci->cap_regs->caplength_and_hciversion,
        XHCI_CAP_CAPLENGTH));

  xhci_trace ("HCIVERSION=0x%04x\n",
      mmio_read_bits (&xhci->cap_regs->caplength_and_hciversion,
        XHCI_CAP_HCIVERSION));

  xhci_trace ("HCSPARAMS1=0x%08x\n",
      mmio_read32 (&xhci->cap_regs->hcsparams1));

  xhci_trace ("HCSPARAMS2=0x%08x\n",
      mmio_read32 (&xhci->cap_regs->hcsparams2));

  xhci_trace ("HCSPARAMS3=0x%08x\n",
      mmio_read32 (&xhci->cap_regs->hcsparams3));

  xhci_trace ("HCCPARAMS1=0x%08x\n",
      mmio_read32 (&xhci->cap_regs->hccparams1));

  xhci_trace ("DBOFF=0x%08x\n",
      RTSOFF_TO_BYTES(mmio_read_bits (&xhci->cap_regs->dboff,
          XHCI_CAP_DBOFF)));

  xhci_trace ("RTSOFF=0x%08x\n",
      RTSOFF_TO_BYTES(mmio_read_bits (&xhci->cap_regs->rtsoff,
          XHCI_CAP_RTSOFF)));

  xhci_trace ("HCCPARAMS2=0x%08x\n",
      mmio_read32 (&xhci->cap_regs->hccparams2));

  return 0;
}

