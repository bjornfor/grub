/*
 * xhci.h - xHCI driver for GRUB
 *
 * [spec] http://www.intel.com/content/www/us/en/io/universal-serial-bus/extensible-host-controler-interface-usb-xhci.html
 */

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
//#include <grub/pciutils.h> // doesn't work
#include <grub/cpu/pci.h>
#include <grub/cpu/io.h>
#include <grub/time.h>
#include <grub/loader.h>
#include <grub/disk.h>
#include <grub/cache.h>

GRUB_MOD_LICENSE ("GPLv3+");

/* Host Controller Capability Registers. Section 5.3 in [spec]. */
enum
{
  GRUB_XHCI_CAP_CAPLENGTH  = 0x00, /* 1 byte */
  /* 1 byte reserved */
  GRUB_XHCI_CAP_HCIVERSION = 0x02, /* 2 bytes */
  GRUB_XHCI_CAP_HCSPARAMS1 = 0x04, /* 4 bytes */
  GRUB_XHCI_CAP_HCSPARAMS2 = 0x08, /* 4 bytes */
  GRUB_XHCI_CAP_HCSPARAMS3 = 0x0c, /* 4 bytes */
  GRUB_XHCI_CAP_HCCPARAMS1 = 0x10, /* 4 bytes */
  GRUB_XHCI_CAP_DBOFF      = 0x14, /* 4 bytes */
  GRUB_XHCI_CAP_RTSOFF     = 0x18, /* 4 bytes */
  GRUB_XHCI_CAP_HCCPARAMS2 = 0x1c, /* 4 bytes */
  /* (CAPLENGTH - 0x20) bytes reserved */
};

/* Host Controller Operational Registers. Section 5.4 in [spec]. */
enum
{
  GRUB_XHCI_OPER_USBCMD   = 0x00,
  GRUB_XHCI_OPER_USBSTS   = 0x04,
  GRUB_XHCI_OPER_PAGESIZE = 0x08,
  /* 0x0c - 0x13 reserved */
  GRUB_XHCI_OPER_DNCTRL   = 0x14,
  GRUB_XHCI_OPER_CRCR     = 0x18,
  /* 0x20 - 0x2f reserved */
  GRUB_XHCI_OPER_DCBAAP   = 0x30,
  GRUB_XHCI_OPER_CONFIG   = 0x38,
  /* 0x3c - 0x3ff reserved */
  /* 0x400 - 0x13ff Port Register Set 1-MaxPorts */
};

/* USB Command Register (USBCMD) bits. Section 5.4.1 in [spec]. */
enum
{
  GRUB_XHCI_OPER_USBCMD_RUNSTOP    = (1 <<  0),
  GRUB_XHCI_OPER_USBCMD_HCRST      = (1 <<  1), /* host controller reset */
  GRUB_XHCI_OPER_USBCMD_INTE       = (1 <<  2), /* interrupter enable */
  GRUB_XHCI_OPER_USBCMD_HSEE       = (1 <<  3), /* host system error enable */
  /* bit 6:4 reserved */
  GRUB_XHCI_OPER_USBCMD_LHCRST     = (1 <<  7), /* light host controller reset */
  GRUB_XHCI_OPER_USBCMD_CSS        = (1 <<  8), /* controller save state */
  GRUB_XHCI_OPER_USBCMD_CRS        = (1 <<  9), /* controller restore state */
  GRUB_XHCI_OPER_USBCMD_EWE        = (1 << 10), /* enable wrap event */
  /* ... */
};

/* USB Status Register (USBSTS) bits. Section 5.4.2 in [spec]. */
enum
{
  GRUB_XHCI_USBSTS_HCH  = (1 <<  0), /* host controller halted */
  /* reserved */
  GRUB_XHCI_USBSTS_HSE  = (1 <<  2), /* host system error */
  GRUB_XHCI_USBSTS_EINT = (1 <<  3), /* event interrupt */
  GRUB_XHCI_USBSTS_PCD  = (1 <<  4), /* port change detect */
  /* 7:5 reserved */
  GRUB_XHCI_USBSTS_SSS  = (1 <<  8), /* save state status */
  GRUB_XHCI_USBSTS_RSS  = (1 <<  9), /* restore state status */
  GRUB_XHCI_USBSTS_SRE  = (1 << 10), /* save/restore error */
  GRUB_XHCI_USBSTS_CNR  = (1 << 11), /* controller not ready */
  GRUB_XHCI_USBSTS_HCE  = (1 << 12), /* host controller error */
  /* 31:13 reserved */
};

/* Offset relative to Operational Base */
#define GRUB_XHCI_PORTSC(port) (0x400 + (0x10 * (port - 1)))

#define GRUB_XHCI_ADDR_MEM_MASK	(~0xff)
#define GRUB_XHCI_POINTER_MASK	(~0x1f)

/* USB Legacy Support Capability (USBLEGSUP) bits. Section 7.1.1 in [spec]. */
enum
{
  GRUB_XHCI_USBLEGSUP_BIOS_OWNED = (1 << 16),
  GRUB_XHCI_USBLEGSUP_OS_OWNED = (1 << 24)
};

/* Operational register PORTSC bits */
enum
{
  GRUB_XHCI_PORTSC_CCS = (1 << 0), /* current connect status */
  GRUB_XHCI_PORTSC_PED = (1 << 1), /* port enabled/disabled */
  /* reserved */
  GRUB_XHCI_PORT_ENABLED = (1 << 2),
  GRUB_XHCI_PORT_ENABLED_CH = (1 << 3),
  GRUB_XHCI_PORT_OVERCUR = (1 << 4),
  GRUB_XHCI_PORT_OVERCUR_CH = (1 << 5),
  GRUB_XHCI_PORT_RESUME = (1 << 6),
  GRUB_XHCI_PORT_SUSPEND = (1 << 7),
  GRUB_XHCI_PORT_RESET = (1 << 8),
  GRUB_XHCI_PORT_LINE_STAT = (3 << 10),
  GRUB_XHCI_PORT_POWER = (1 << 12),
  GRUB_XHCI_PORT_OWNER = (1 << 13),
  GRUB_XHCI_PORT_INDICATOR = (3 << 14),
  GRUB_XHCI_PORT_TEST = (0xf << 16),
  GRUB_XHCI_PORT_WON_CONN_E = (1 << 20),
  GRUB_XHCI_PORT_WON_DISC_E = (1 << 21),
  GRUB_XHCI_PORT_WON_OVER_E = (1 << 22),

  GRUB_XHCI_PORT_LINE_SE0 = (0 << 10),
  GRUB_XHCI_PORT_LINE_K = (1 << 10),
  GRUB_XHCI_PORT_LINE_J = (2 << 10),
  GRUB_XHCI_PORT_LINE_UNDEF = (3 << 10),
  GRUB_XHCI_PORT_LINE_LOWSP = GRUB_XHCI_PORT_LINE_K,	/* K state means low speed */
  GRUB_XHCI_PORT_WMASK = ~(GRUB_XHCI_PORTSC_PED
			   | GRUB_XHCI_PORT_ENABLED_CH
			   | GRUB_XHCI_PORT_OVERCUR_CH)
};

/* Operational register CONFIGFLAGS bits */
enum
{
  GRUB_XHCI_CF_XHCI_OWNER = (1 << 0)
};

/* Queue Head & Transfer Descriptor constants */
#define GRUB_XHCI_HPTR_OFF       5	/* Horiz. pointer bit offset */
enum
{
  GRUB_XHCI_HPTR_TYPE_MASK = (3 << 1),
  GRUB_XHCI_HPTR_TYPE_ITD = (0 << 1),
  GRUB_XHCI_HPTR_TYPE_QH = (1 << 1),
  GRUB_XHCI_HPTR_TYPE_SITD = (2 << 1),
  GRUB_XHCI_HPTR_TYPE_FSTN = (3 << 1)
};

enum
{
  GRUB_XHCI_C = (1 << 27),
  GRUB_XHCI_MAXPLEN_MASK = (0x7ff << 16),
  GRUB_XHCI_H = (1 << 15),
  GRUB_XHCI_DTC = (1 << 14),
  GRUB_XHCI_SPEED_MASK = (3 << 12),
  GRUB_XHCI_SPEED_FULL = (0 << 12),
  GRUB_XHCI_SPEED_LOW = (1 << 12),
  GRUB_XHCI_SPEED_HIGH = (2 << 12),
  GRUB_XHCI_SPEED_RESERVED = (3 << 12),
  GRUB_XHCI_EP_NUM_MASK = (0xf << 8),
  GRUB_XHCI_DEVADDR_MASK = 0x7f,
  GRUB_XHCI_TARGET_MASK = (GRUB_XHCI_EP_NUM_MASK | GRUB_XHCI_DEVADDR_MASK)
};

enum
{
  GRUB_XHCI_MAXPLEN_OFF = 16,
  GRUB_XHCI_SPEED_OFF = 12,
  GRUB_XHCI_EP_NUM_OFF = 8
};

enum
{
  GRUB_XHCI_MULT_MASK = (3 << 30),
  GRUB_XHCI_MULT_RESERVED = (0 << 30),
  GRUB_XHCI_MULT_ONE = (1 << 30),
  GRUB_XHCI_MULT_TWO = (2 << 30),
  GRUB_XHCI_MULT_THREE = (3 << 30),
  GRUB_XHCI_DEVPORT_MASK = (0x7f << 23),
  GRUB_XHCI_HUBADDR_MASK = (0x7f << 16),
  GRUB_XHCI_CMASK_MASK = (0xff << 8),
  GRUB_XHCI_SMASK_MASK = (0xff << 0),
};

enum
{
  GRUB_XHCI_MULT_OFF = 30,
  GRUB_XHCI_DEVPORT_OFF = 23,
  GRUB_XHCI_HUBADDR_OFF = 16,
  GRUB_XHCI_CMASK_OFF = 8,
  GRUB_XHCI_SMASK_OFF = 0,
};

#define GRUB_XHCI_TERMINATE      (1<<0)

#define GRUB_XHCI_TOGGLE         (1<<31)

enum
{
  GRUB_XHCI_TOTAL_MASK = (0x7fff << 16),
  GRUB_XHCI_CERR_MASK = (3 << 10),
  GRUB_XHCI_CERR_0 = (0 << 10),
  GRUB_XHCI_CERR_1 = (1 << 10),
  GRUB_XHCI_CERR_2 = (2 << 10),
  GRUB_XHCI_CERR_3 = (3 << 10),
  GRUB_XHCI_PIDCODE_OUT = (0 << 8),
  GRUB_XHCI_PIDCODE_IN = (1 << 8),
  GRUB_XHCI_PIDCODE_SETUP = (2 << 8),
  GRUB_XHCI_STATUS_MASK = 0xff,
  GRUB_XHCI_STATUS_ACTIVE = (1 << 7),
  GRUB_XHCI_STATUS_HALTED = (1 << 6),
  GRUB_XHCI_STATUS_BUFERR = (1 << 5),
  GRUB_XHCI_STATUS_BABBLE = (1 << 4),
  GRUB_XHCI_STATUS_TRANERR = (1 << 3),
  GRUB_XHCI_STATUS_MISSDMF = (1 << 2),
  GRUB_XHCI_STATUS_SPLITST = (1 << 1),
  GRUB_XHCI_STATUS_PINGERR = (1 << 0)
};

enum
{
  GRUB_XHCI_TOTAL_OFF = 16,
  GRUB_XHCI_CERR_OFF = 10
};

#define GRUB_XHCI_BUFPTR_MASK    (0xfffff<<12)
#define GRUB_XHCI_QHTDPTR_MASK   0xffffffe0

#define GRUB_XHCI_TD_BUF_PAGES   5

#define GRUB_XHCI_BUFPAGELEN     0x1000
#define GRUB_XHCI_MAXBUFLEN      0x5000

static grub_uint32_t
pci_config_read (grub_pci_device_t dev, int reg)
{
  grub_pci_address_t addr;
  addr = grub_pci_make_address (dev, reg);
  return grub_pci_read (addr);
}

struct grub_xhci
{
  volatile grub_uint32_t *cap;	   /* Capability registers */

  /* grub stuff */
  volatile grub_uint32_t *iobase_cap;	   /* Capability registers */
  volatile grub_uint32_t *iobase_oper;	   /* Operational registers */
  volatile grub_uint32_t *iobase_runtime;  /* Run-time registers */
  volatile grub_uint32_t *iobase_doorbell; /* Doorbell array */
  //unsigned int reset;

  struct grub_xhci *next;
};

static struct grub_xhci *xhci_list;

/* xHCI capability registers access functions */
static inline grub_uint32_t
grub_xhci_cap_read32 (struct grub_xhci *xhci, grub_uint32_t addr)
{
  return
    grub_le_to_cpu32 (*((volatile grub_uint32_t *) xhci->iobase_cap +
		       (addr / sizeof (grub_uint32_t))));
}

/* Halt if xHCI HC not halted */
static grub_usb_err_t
grub_xhci_halt (struct grub_xhci *xhci)
{
  (void)xhci;
  //grub_uint64_t maxtime;
  //unsigned int is_halted;

  grub_dprintf ("xhci", "grub_xhci_halt enter\n");
  //is_halted = grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBSTS) & GRUB_XHCI_USBSTS_HCH;
  //if (is_halted == 0)
  //  {
  //    grub_xhci_oper_write32 (xhci, GRUB_XHCI_OPER_USBCMD,
  //      		      ~GRUB_XHCI_OPER_USBCMD_RUNSTOP
  //      		      & grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD));
  //    /* Ensure command is written */
  //    grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD);
  //    maxtime = grub_get_time_ms () + 16000; /* spec says 16ms max */
  //    while (((grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBSTS)
  //             & GRUB_XHCI_USBSTS_HCH) == 0)
  //           && (grub_get_time_ms () < maxtime));
  //    if ((grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBSTS)
  //         & GRUB_XHCI_USBSTS_HCH) == 0)
  //      return GRUB_USB_ERR_TIMEOUT;
  //  }

  return GRUB_USB_ERR_NONE;
}

/* xHCI HC reset */
static grub_usb_err_t
grub_xhci_reset (struct grub_xhci *xhci)
{
  //grub_uint64_t maxtime;
  (void)xhci;

  grub_dprintf ("xhci", "grub_xhci_reset enter\n");

  //sync_all_caches (xhci);

  //grub_xhci_oper_write32 (xhci, GRUB_XHCI_OPER_USBCMD,
  //      		  GRUB_XHCI_OPER_USBCMD_HCRST
  //      		  | grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD));
  ///* Ensure command is written */
  //grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD);
  ///* XXX: How long time could take reset of HC ? */
  //maxtime = grub_get_time_ms () + 16000;
  //while (((grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD)
  //         & GRUB_XHCI_OPER_USBCMD_HCRST) != 0)
  //       && (grub_get_time_ms () < maxtime));
  //if ((grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD)
  //     & GRUB_XHCI_OPER_USBCMD_HCRST) != 0)
  //  return GRUB_USB_ERR_TIMEOUT;

  return GRUB_USB_ERR_NONE;
}


#if 0
/* This simple GRUB implementation of xHCI driver:
 *      - assumes no IRQ
 */

/* PCI Configuration Space */
#define GRUB_PCI_REG_BAR0 0x10
#define GRUB_PCI_REG_BAR1 0x14


struct grub_xhci_td;
struct grub_xhci_qh;
typedef volatile struct grub_xhci_td *grub_xhci_td_t;
typedef volatile struct grub_xhci_qh *grub_xhci_qh_t;

/* xHCI Isochronous Transfer Descriptor */
/* Currently not supported */

/* xHCI Split Transaction Isochronous Transfer Descriptor */
/* Currently not supported */

/* xHCI Queue Element Transfer Descriptor (qTD) */
/* Align to 32-byte boundaries */
struct grub_xhci_td
{
  /* xHCI HW part */
  grub_uint32_t next_td;	/* Pointer to next qTD */
  grub_uint32_t alt_next_td;	/* Pointer to alternate next qTD */
  grub_uint32_t token;		/* Toggle, Len, Interrupt, Page, Error, PID, Status */
  grub_uint32_t buffer_page[GRUB_XHCI_TD_BUF_PAGES];	/* Buffer pointer (+ cur. offset in page 0 */
  /* 64-bits part */
  grub_uint32_t buffer_page_high[GRUB_XHCI_TD_BUF_PAGES];
  /* xHCI driver part */
  grub_uint32_t link_td;	/* pointer to next free/chained TD */
  grub_uint32_t size;
  grub_uint32_t pad[1];		/* padding to some multiple of 32 bytes */
};

/* xHCI Queue Head */
/* Align to 32-byte boundaries */
/* QH allocation is made in the similar/same way as in OHCI driver,
 * because unlninking QH from the Asynchronous list is not so
 * trivial as on UHCI (at least it is time consuming) */
struct grub_xhci_qh
{
  /* xHCI HW part */
  grub_uint32_t qh_hptr;	/* Horiz. pointer & Terminate */
  grub_uint32_t ep_char;	/* EP characteristics */
  grub_uint32_t ep_cap;		/* EP capabilities */
  grub_uint32_t td_current;	/* current TD link pointer  */
  struct grub_xhci_td td_overlay;	/* TD overlay area = 64 bytes */
  /* xHCI driver part */
  grub_uint32_t pad[4];		/* padding to some multiple of 32 bytes */
};

static void
sync_all_caches (struct grub_xhci *xhci)
{
  (void)xhci;
  grub_dprintf("xhci", "sync_all_caches enter\n");
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

static inline grub_uint16_t
grub_xhci_cap_read16 (struct grub_xhci *xhci, grub_uint32_t addr)
{
  return
    grub_le_to_cpu16 (*((volatile grub_uint16_t *) xhci->iobase_cap +
		       (addr / sizeof (grub_uint16_t))));
}

static inline grub_uint8_t
grub_xhci_cap_read8 (struct grub_xhci *xhci, grub_uint32_t addr)
{
  return *((volatile grub_uint8_t *) xhci->iobase_cap + addr);
}

/* Operational registers access functions */
static inline grub_uint32_t
grub_xhci_oper_read32 (struct grub_xhci *xhci, grub_uint32_t addr)
{
  return
    grub_le_to_cpu32 (*
		      ((volatile grub_uint32_t *) xhci->iobase_oper +
		       (addr / sizeof (grub_uint32_t))));
}

static inline void
grub_xhci_oper_write32 (struct grub_xhci *xhci, grub_uint32_t addr,
			grub_uint32_t value)
{
  *((volatile grub_uint32_t *) xhci->iobase_oper + (addr / sizeof (grub_uint32_t))) =
    grub_cpu_to_le32 (value);
}

static inline grub_uint32_t
grub_xhci_port_read (struct grub_xhci *xhci, grub_uint32_t port)
{
  return grub_xhci_oper_read32 (xhci, GRUB_XHCI_PORTSC(port));
}

static inline void
grub_xhci_port_resbits (struct grub_xhci *xhci, grub_uint32_t port,
			grub_uint32_t bits)
{
  grub_xhci_oper_write32 (xhci, GRUB_XHCI_PORTSC(port),
			  grub_xhci_port_read (xhci,
					       port) & GRUB_XHCI_PORT_WMASK &
			  ~(bits));
  grub_xhci_port_read (xhci, port);
}

static inline void
grub_xhci_port_setbits (struct grub_xhci *xhci, grub_uint32_t port,
			grub_uint32_t bits)
{
  grub_xhci_oper_write32 (xhci, GRUB_XHCI_PORTSC(port),
			  (grub_xhci_port_read (xhci, port) &
			   GRUB_XHCI_PORT_WMASK) | bits);
  grub_xhci_port_read (xhci, port);
}

static int
grub_xhci_init (struct grub_xhci *xhci, void *regs)
{
  //pci_device_cfg_read_u16 (addr.dev, &ret, addr.pos);
#if 0
  grub_int32_t hcsparams1;
  grub_uint32_t hcsparams2;
  grub_uint32_t hccparams1;
  grub_uint32_t pagesize;
  unsigned int caplength;
  unsigned int rtsoff;
  unsigned int dboff;

  /* Locate capability, operational, runtime, and doorbell registers */
  xhci->cap = regs;
  caplength = readb ( xhci->cap + XHCI_CAP_CAPLENGTH );
  rtsoff = readl ( xhci->cap + XHCI_CAP_RTSOFF );
  dboff = readl ( xhci->cap + XHCI_CAP_DBOFF );
  xhci->op = ( xhci->cap + caplength );
  xhci->run = ( xhci->cap + rtsoff );
  xhci->db = ( xhci->cap + dboff );
  DBGC2 ( xhci, "XHCI %s cap %08lx op %08lx run %08lx db %08lx\n",
      xhci->name, virt_to_phys ( xhci->cap ),
      virt_to_phys ( xhci->op ), virt_to_phys ( xhci->run ),
      virt_to_phys ( xhci->db ) );

  /* Read structural parameters 1 */
  hcsparams1 = readl ( xhci->cap + XHCI_CAP_HCSPARAMS1 );
  xhci->slots = XHCI_HCSPARAMS1_SLOTS ( hcsparams1 );
  xhci->intrs = XHCI_HCSPARAMS1_INTRS ( hcsparams1 );
  xhci->ports = XHCI_HCSPARAMS1_PORTS ( hcsparams1 );
  DBGC ( xhci, "XHCI %s has %d slots %d intrs %d ports\n",
      xhci->name, xhci->slots, xhci->intrs, xhci->ports );

  /* Read structural parameters 2 */
  hcsparams2 = readl ( xhci->cap + XHCI_CAP_HCSPARAMS2 );
  xhci->scratchpads = XHCI_HCSPARAMS2_SCRATCHPADS ( hcsparams2 );
  DBGC2 ( xhci, "XHCI %s needs %d scratchpads\n",
      xhci->name, xhci->scratchpads );

  /* Read capability parameters 1 */
  hccparams1 = readl ( xhci->cap + XHCI_CAP_HCCPARAMS1 );
  xhci->addr64 = XHCI_HCCPARAMS1_ADDR64 ( hccparams1 );
  xhci->csz_shift = XHCI_HCCPARAMS1_CSZ_SHIFT ( hccparams1 );
  xhci->xecp = XHCI_HCCPARAMS1_XECP ( hccparams1 );

  /* Read page size */
  pagesize = readl ( xhci->op + XHCI_OP_PAGESIZE );
  xhci->pagesize = XHCI_PAGESIZE ( pagesize );
  assert ( xhci->pagesize != 0 );
  assert ( ( ( xhci->pagesize ) & ( xhci->pagesize - 1 ) ) == 0 );
  DBGC2 ( xhci, "XHCI %s page size %zd bytes\n",
      xhci->name, xhci->pagesize );
#endif

  /*** GRUB CODE BELOW ***/

  grub_uint32_t base, base_h;
  //grub_uint32_t n_ports;
  grub_uint8_t caplen;
  grub_pci_address_t addr;
  //grub_uint32_t hccparams1;
  int status;

  /*
   * Map MMIO registers
   */
  addr = grub_pci_make_address (dev, GRUB_PCI_REG_BAR0);
  base = grub_pci_read (addr);
  addr = grub_pci_make_address (dev, GRUB_PCI_REG_BAR1);
  base_h = grub_pci_read (addr);
  /* Stop if registers are mapped above 4G - GRUB does not currently
     work with registers mapped above 4G */
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

  grub_dprintf ("xhci", "xHCI grub_xhci_pci_iter: 32-bit xHCI OK\n");

  /* Allocate memory for the controller and fill basic values. */
  xhci = grub_zalloc (sizeof (*xhci));
  if (!xhci)
    return 1;

  xhci->iobase_cap = grub_pci_device_map_range (dev,
                  (base & GRUB_XHCI_ADDR_MEM_MASK),
                  0x100); /* PCI config space is 256 bytes */
  grub_dprintf ("xhci", "xHCI grub_xhci_pci_iter: iobase of CAP: %08x\n",
		(xhci->iobase_cap));

  /* Determine base address of xHCI operational registers */
  caplen = grub_xhci_cap_read8 (xhci, GRUB_XHCI_CAP_CAPLENGTH);
#ifndef GRUB_HAVE_UNALIGNED_ACCESS
  if (caplen & (sizeof (grub_uint32_t) - 1))
    {
      grub_dprintf ("xhci", "Unaligned caplen\n");
      return 0;
    }
  xhci->iobase_oper = ((volatile grub_uint32_t *) xhci->iobase_cap
	       + (caplen / sizeof (grub_uint32_t)));
#else
  xhci->iobase_oper = (volatile grub_uint32_t *)
    ((grub_uint8_t *) xhci->iobase_cap + caplen);
#endif

  /* TODO: initialize the various "rings" and TRBs */

#if 0
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
    if (usblegsup & GRUB_XHCI_USBLEGSUP_BIOS_OWNED)
      {
        grub_boot_time ("Taking ownership of xHCI controller");
        grub_dprintf ("xhci",
                      "xHCI grub_xhci_pci_iter: xHCI owned by: BIOS\n");
        /* Ownership change - set OS_OWNED bit */
        grub_pci_write (pciaddr_eecp, usblegsup | GRUB_XHCI_USBLEGSUP_OS_OWNED);
        /* Ensure PCI register is written */
        grub_pci_read (pciaddr_eecp);

        /* Wait for finish of ownership change, xHCI specification
         * says it can take up to 16 ms
         */
        maxtime = grub_get_time_ms () + 1000;
        while ((grub_pci_read (pciaddr_eecp) & GRUB_XHCI_USBLEGSUP_BIOS_OWNED)
               && (grub_get_time_ms () < maxtime));
        if (grub_pci_read (pciaddr_eecp) & GRUB_XHCI_USBLEGSUP_BIOS_OWNED)
          {
            grub_dprintf ("xhci",
                          "xHCI grub_xhci_pci_iter: xHCI change ownership timeout");
            /* Change ownership in "hard way" - reset BIOS ownership */
            grub_pci_write (pciaddr_eecp, GRUB_XHCI_USBLEGSUP_OS_OWNED);
            /* Ensure PCI register is written */
            grub_pci_read (pciaddr_eecp);
          }
      }
    else if (usblegsup & GRUB_XHCI_USBLEGSUP_OS_OWNED)
      /* XXX: What to do in this case - nothing ? Can it happen ? */
      grub_dprintf ("xhci", "xHCI grub_xhci_pci_iter: xHCI owned by: OS\n");
    else
      {
        grub_dprintf ("xhci",
                      "xHCI grub_xhci_pci_iter: xHCI owned by: NONE\n");
        /* XXX: What to do in this case ? Can it happen ?
         * Is code below correct ? */
        /* Ownership change - set OS_OWNED bit */
        grub_pci_write (pciaddr_eecp, GRUB_XHCI_USBLEGSUP_OS_OWNED);
        /* Ensure PCI register is written */
        grub_pci_read (pciaddr_eecp);
      }

    /* Disable SMI, just to be sure.  */
    pciaddr_eecp = grub_pci_make_address (dev, eecp_offset + 4);
    grub_pci_write (pciaddr_eecp, 0);
    /* Ensure PCI register is written */
    grub_pci_read (pciaddr_eecp);
  }

  grub_dprintf ("xhci", "inithw: xHCI grub_xhci_pci_iter: ownership OK\n");

#endif

  /* Now we can setup xHCI (maybe...) */

  /* Check if xHCI is halted and halt it if not */
  if (grub_xhci_halt (xhci) != GRUB_USB_ERR_NONE)
    {
      grub_error (GRUB_ERR_TIMEOUT,
		  "xHCI grub_xhci_pci_iter: xHCI halt timeout");
      goto fail;
    }

  grub_dprintf ("xhci", "xHCI grub_xhci_pci_iter: halted OK\n");

  /* Reset xHCI */
  if (grub_xhci_reset (xhci) != GRUB_USB_ERR_NONE)
    {
      grub_error (GRUB_ERR_TIMEOUT,
		  "xHCI grub_xhci_pci_iter: xHCI reset timeout");
      goto fail;
    }

  grub_dprintf ("xhci", "xHCI grub_xhci_pci_iter: reset OK\n");

#if 0
  /* Now should be possible to power-up and enumerate ports etc. */
  if ((grub_xhci_cap_read32 (xhci, GRUB_XHCI_EHCC_SPARAMS)
       & GRUB_XHCI_SPARAMS_PPC) != 0)
    {				/* xHCI has port powering control */
      /* Power on all ports */
      n_ports = grub_xhci_cap_read32 (xhci, GRUB_XHCI_EHCC_SPARAMS)
	& GRUB_XHCI_SPARAMS_N_PORTS;
      for (i = 0; i < (int) n_ports; i++)
	grub_xhci_oper_write32 (xhci, GRUB_XHCI_PORTSC(port),
				GRUB_XHCI_PORT_POWER
				| grub_xhci_oper_read32 (xhci,
							 GRUB_XHCI_PORTSC(i)));
    }

#endif
  /* Ensure all commands are written */
  grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD);

  /* Enable xHCI */
  grub_xhci_oper_write32 (xhci, GRUB_XHCI_OPER_USBCMD,
			  GRUB_XHCI_OPER_USBCMD_RUNSTOP
			  | grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD));

  /* Ensure command is written */
  grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD);

  /* Link to xhci now that initialisation is successful.  */
  xhci->next = xhci_list;
  xhci_list = xhci;

  sync_all_caches (xhci);

  grub_dprintf ("xhci", "xHCI grub_xhci_pci_iter: OK at all\n");

  grub_dprintf ("xhci",
		"xHCI grub_xhci_pci_iter: iobase of oper. regs: %08x\n",
		((unsigned int)xhci->iobase_oper));
  grub_dprintf ("xhci", "xHCI grub_xhci_pci_iter: USBCMD: %08x\n",
		grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD));
  grub_dprintf ("xhci", "xHCI grub_xhci_pci_iter: USBSTS: %08x\n",
		grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBSTS));

  return 0;

fail:
  if (xhci)
    {
//      if (xhci->td_chunk)
//	grub_dma_free ((void *) xhci->td_chunk);
//      if (xhci->qh_chunk)
//	grub_dma_free ((void *) xhci->qh_chunk);
//      if (xhci->framelist_chunk)
//	grub_dma_free (xhci->framelist_chunk);
    }
  grub_free (xhci);

}

static int
grub_pci_config_read (grub_pci_device_t dev, int reg)
{
  return *(volatile grub_uint32_t *) config_addr (addr);
}

/* PCI iteration function... */
static int
grub_xhci_pci_iter2 (grub_pci_device_t dev,
                    grub_pci_id_t pciid __attribute__ ((unused)),
		    void *data __attribute__ ((unused)))
{
  grub_uint8_t release;
  grub_uint32_t class_code;
  grub_uint32_t interf;
  grub_uint32_t subclass;
  grub_uint32_t class;
  grub_uint32_t base, base_h;
  struct grub_xhci *xhci;
  //grub_uint32_t eecp_offset;
  //grub_uint32_t fp;
  //int i;
  //grub_uint32_t usblegsup = 0;
  //grub_uint64_t maxtime;
  //grub_uint32_t n_ports;
  grub_uint8_t caplen;
  grub_pci_address_t addr;
  //grub_uint32_t hccparams1;

  int ret;
  int status = 0;
  grub_uint32_t class_code;

  (void)pciid;

#if 0
  addr = grub_pci_make_address (dev, GRUB_PCI_REG_CLASS);
  class_code = grub_pci_read (addr) >> 8;
  interf = class_code & 0xFF;
  subclass = (class_code >> 8) & 0xFF;
  class = class_code >> 16;

  /* If this is not an xHCI controller, just return.  */
  if (class != 0x0c || subclass != 0x03 || interf != 0x30)
    return 0;

  grub_dprintf ("xhci", "xHCI grub_xhci_pci_iter: class OK\n");
#endif

  /* Check Serial Bus Release Number */
  addr = grub_pci_make_address (dev, GRUB_XHCI_PCI_SBRN_REG);
  release = grub_pci_read_byte (addr);
  if (release != 0x30)
    {
      grub_dprintf ("xhci", "XHCI grub_xhci_pci_iter: Wrong SBRN: %0x\n",
                      release);
      return 0;
    }
  grub_dprintf ("xhci", "XHCI grub_xhci_pci_iter: bus rev. num. OK\n");

  /* We found an xHCI controller, now initialize it */
  status = grub_xhci_init(dev);

  return status;
}

#if 0
static void
grub_xhci_setup_qh (grub_xhci_qh_t qh, grub_usb_transfer_t transfer)
{
  grub_uint32_t ep_char = 0;
  grub_uint32_t ep_cap = 0;

  /* Note: Another part of code is responsible to this QH is
   * Halted ! But it can be linked in AL, so we cannot erase or
   * change qh_hptr ! */
  /* We will not change any TD field because they should/must be
   * in safe state from previous use. */

  /* EP characteristic setup */
  /* Currently not used NAK counter (RL=0),
   * C bit set if EP is not HIGH speed and is control,
   * Max Packet Length is taken from transfer structure,
   * H bit = 0 (because QH[1] has this bit set),
   * DTC bit set to 1 because we are using our own toggle bit control,
   * SPEED is selected according to value from transfer structure,
   * EP number is taken from transfer structure
   * "I" bit must not be set,
   * Device Address is taken from transfer structure
   * */
  if ((transfer->dev->speed != GRUB_USB_SPEED_HIGH)
      && (transfer->type == GRUB_USB_TRANSACTION_TYPE_CONTROL))
    ep_char |= GRUB_XHCI_C;
  ep_char |= (transfer->max << GRUB_XHCI_MAXPLEN_OFF)
    & GRUB_XHCI_MAXPLEN_MASK;
  ep_char |= GRUB_XHCI_DTC;
  switch (transfer->dev->speed)
    {
    case GRUB_USB_SPEED_LOW:
      ep_char |= GRUB_XHCI_SPEED_LOW;
      break;
    case GRUB_USB_SPEED_FULL:
      ep_char |= GRUB_XHCI_SPEED_FULL;
      break;
    case GRUB_USB_SPEED_HIGH:
    default:
      ep_char |= GRUB_XHCI_SPEED_HIGH;
      /* XXX: How we will handle unknown value of speed? */
    }
  ep_char |= (transfer->endpoint << GRUB_XHCI_EP_NUM_OFF)
    & GRUB_XHCI_EP_NUM_MASK;
  ep_char |= transfer->devaddr & GRUB_XHCI_DEVADDR_MASK;
  qh->ep_char = grub_cpu_to_le32 (ep_char);
  /* EP capabilities setup */
  /* MULT field - we try to use max. number
   * PortNumber - included now in device structure referenced
   *              inside transfer structure
   * HubAddress - included now in device structure referenced
   *              inside transfer structure
   * SplitCompletionMask - AFAIK it is ignored in asynchronous list,
   * InterruptScheduleMask - AFAIK it should be zero in async. list */
  ep_cap |= GRUB_XHCI_MULT_THREE;
  ep_cap |= (transfer->dev->split_hubport << GRUB_XHCI_DEVPORT_OFF)
    & GRUB_XHCI_DEVPORT_MASK;
  ep_cap |= (transfer->dev->split_hubaddr << GRUB_XHCI_HUBADDR_OFF)
    & GRUB_XHCI_HUBADDR_MASK;
  if (transfer->dev->speed == GRUB_USB_SPEED_LOW
      && transfer->type != GRUB_USB_TRANSACTION_TYPE_CONTROL)
  {
    ep_cap |= (1<<0) << GRUB_XHCI_SMASK_OFF;
    ep_cap |= (7<<2) << GRUB_XHCI_CMASK_OFF;
  }
  qh->ep_cap = grub_cpu_to_le32 (ep_cap);

  grub_dprintf ("xhci", "setup_qh: qh=%p, not changed: qh_hptr=%08x\n",
		qh, grub_le_to_cpu32 (qh->qh_hptr));
  grub_dprintf ("xhci", "setup_qh: ep_char=%08x, ep_cap=%08x\n",
		ep_char, ep_cap);
  grub_dprintf ("xhci", "setup_qh: end\n");
  grub_dprintf ("xhci", "setup_qh: not changed: td_current=%08x\n",
		grub_le_to_cpu32 (qh->td_current));
  grub_dprintf ("xhci", "setup_qh: not changed: next_td=%08x\n",
		grub_le_to_cpu32 (qh->td_overlay.next_td));
  grub_dprintf ("xhci", "setup_qh: not changed: alt_next_td=%08x\n",
		grub_le_to_cpu32 (qh->td_overlay.alt_next_td));
  grub_dprintf ("xhci", "setup_qh: not changed: token=%08x\n",
		grub_le_to_cpu32 (qh->td_overlay.token));
}
#endif

#if 0
static grub_xhci_td_t
grub_xhci_transaction (struct grub_xhci *xhci,
		       grub_transfer_type_t type,
		       unsigned int toggle, grub_size_t size,
		       grub_uint32_t data, grub_xhci_td_t td_alt)
{
  grub_xhci_td_t td;
  grub_uint32_t token;
  grub_uint32_t bufadr;
  int i;

  /* Test of transfer size, it can be:
   * <= GRUB_XHCI_MAXBUFLEN if data aligned to page boundary
   * <= GRUB_XHCI_MAXBUFLEN - GRUB_XHCI_BUFPAGELEN if not aligned
   *    (worst case)
   */
  if ((((data % GRUB_XHCI_BUFPAGELEN) == 0)
       && (size > GRUB_XHCI_MAXBUFLEN))
      ||
      (((data % GRUB_XHCI_BUFPAGELEN) != 0)
       && (size > (GRUB_XHCI_MAXBUFLEN - GRUB_XHCI_BUFPAGELEN))))
    {
      grub_error (GRUB_ERR_OUT_OF_MEMORY,
		  "too long data buffer for xHCI transaction");
      return 0;
    }

  /* Grab a free Transfer Descriptor and initialize it.  */
  td = 0; //grub_xhci_alloc_td (xhci);
  if (!td)
    {
      grub_error (GRUB_ERR_OUT_OF_MEMORY,
		  "no transfer descriptors available for xHCI transfer");
      return 0;
    }

  grub_dprintf ("xhci",
		"transaction: type=%d, toggle=%d, size=%lu data=0x%x td=%p\n",
		type, toggle, (unsigned long) size, data, td);

  /* Fill whole TD by zeros */
  grub_memset ((void *) td, 0, sizeof (struct grub_xhci_td));

  /* Don't point to any TD yet, just terminate.  */
  td->next_td = grub_cpu_to_le32_compile_time (GRUB_XHCI_TERMINATE);
  /* Set alternate pointer. When short packet occurs, alternate TD
   * will not be really fetched because it is not active. But don't
   * forget, xHCI will try to fetch alternate TD every scan of AL
   * until QH is halted. */
  td->alt_next_td = grub_cpu_to_le32 (grub_dma_virt2phys (td_alt,
							   xhci->td_chunk));
  /* token:
   * TOGGLE - according to toggle
   * TOTAL SIZE = size
   * Interrupt On Complete = FALSE, we don't need IRQ
   * Current Page = 0
   * Error Counter = max. value = 3
   * PID Code - according to type
   * STATUS:
   *  ACTIVE bit should be set to one
   *  SPLIT TRANS. STATE bit should be zero. It is ignored
   *   in HIGH speed transaction, and should be zero for LOW/FULL
   *   speed to indicate state Do Split Transaction */
  token = toggle ? GRUB_XHCI_TOGGLE : 0;
  token |= (size << GRUB_XHCI_TOTAL_OFF) & GRUB_XHCI_TOTAL_MASK;
  token |= GRUB_XHCI_CERR_3;
  switch (type)
    {
    case GRUB_USB_TRANSFER_TYPE_IN:
      token |= GRUB_XHCI_PIDCODE_IN;
      break;
    case GRUB_USB_TRANSFER_TYPE_OUT:
      token |= GRUB_XHCI_PIDCODE_OUT;
      break;
    case GRUB_USB_TRANSFER_TYPE_SETUP:
      token |= GRUB_XHCI_PIDCODE_SETUP;
      break;
    default:			/* XXX: Should not happen, but what to do if it does ? */
      break;
    }
  token |= GRUB_XHCI_STATUS_ACTIVE;
  td->token = grub_cpu_to_le32 (token);

  /* Fill buffer pointers according to size */
  bufadr = data;
  td->buffer_page[0] = grub_cpu_to_le32 (bufadr);
  bufadr = ((bufadr / GRUB_XHCI_BUFPAGELEN) + 1) * GRUB_XHCI_BUFPAGELEN;
  for (i = 1; ((bufadr - data) < size) && (i < GRUB_XHCI_TD_BUF_PAGES); i++)
    {
      td->buffer_page[i] = grub_cpu_to_le32 (bufadr & GRUB_XHCI_BUFPTR_MASK);
      bufadr = ((bufadr / GRUB_XHCI_BUFPAGELEN) + 1) * GRUB_XHCI_BUFPAGELEN;
    }

  /* Remember data size for future use... */
  td->size = (grub_uint32_t) size;

  grub_dprintf ("xhci", "td=%p\n", td);
  grub_dprintf ("xhci", "HW: next_td=%08x, alt_next_td=%08x\n",
		grub_le_to_cpu32 (td->next_td),
		grub_le_to_cpu32 (td->alt_next_td));
  grub_dprintf ("xhci", "HW: token=%08x, buffer[0]=%08x\n",
		grub_le_to_cpu32 (td->token),
		grub_le_to_cpu32 (td->buffer_page[0]));
  grub_dprintf ("xhci", "HW: buffer[1]=%08x, buffer[2]=%08x\n",
		grub_le_to_cpu32 (td->buffer_page[1]),
		grub_le_to_cpu32 (td->buffer_page[2]));
  grub_dprintf ("xhci", "HW: buffer[3]=%08x, buffer[4]=%08x\n",
		grub_le_to_cpu32 (td->buffer_page[3]),
		grub_le_to_cpu32 (td->buffer_page[4]));
  grub_dprintf ("xhci", "link_td=%08x, size=%08x\n",
		td->link_td, td->size);

  return td;
}
#endif

struct grub_xhci_transfer_controller_data
{
  grub_xhci_qh_t qh_virt;
  grub_xhci_td_t td_first_virt;
  grub_xhci_td_t td_alt_virt;
  grub_xhci_td_t td_last_virt;
  grub_uint32_t td_last_phys;
};

#if 0
/* This function expects QH is not active.
 * Function set Halt bit in QH TD overlay and possibly prints
 * necessary debug information. */
static void
grub_xhci_pre_finish_transfer (grub_usb_transfer_t transfer)
{
  struct grub_xhci_transfer_controller_data *cdata =
    transfer->controller_data;

  /* Collect debug data here if necessary */

  /* Set Halt bit in not active QH. AL will not attempt to do
   * Advance Queue on QH with Halt bit set, i.e., we can then
   * safely manipulate with QH TD part. */
  cdata->qh_virt->td_overlay.token = (cdata->qh_virt->td_overlay.token
				      |
				      grub_cpu_to_le32_compile_time
				      (GRUB_XHCI_STATUS_HALTED)) &
    grub_cpu_to_le32_compile_time (~GRUB_XHCI_STATUS_ACTIVE);

  /* Print debug data here if necessary */

}
#endif

#if 0
static grub_usb_err_t
grub_xhci_parse_notrun (grub_usb_controller_t dev,
			grub_usb_transfer_t transfer, grub_size_t * actual)
{
  struct grub_xhci *xhci = dev->data;
  struct grub_xhci_transfer_controller_data *cdata =
    transfer->controller_data;

  grub_dprintf ("xhci", "parse_notrun: info\n");

  /* QH can be in any state in this case. */
  /* But xHCI or AL is not running, so QH is surely not active
   * even if it has Active bit set... */
  grub_xhci_pre_finish_transfer (transfer);
  grub_free (cdata);

  sync_all_caches (xhci);

  /* Additionally, do something with xHCI to make it running (what?) */
  /* Try enable xHCI and AL */
  grub_xhci_oper_write32 (xhci, GRUB_XHCI_OPER_USBCMD,
			  GRUB_XHCI_OPER_USBCMD_RUNSTOP | GRUB_XHCI_CMD_AS_ENABL
			  | GRUB_XHCI_CMD_PS_ENABL
			  | grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD));
  /* Ensure command is written */
  grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD);

  return GRUB_USB_ERR_UNRECOVERABLE;
}
#endif

#if 0
static grub_usb_err_t
grub_xhci_parse_halt (grub_usb_controller_t dev,
		      grub_usb_transfer_t transfer, grub_size_t * actual)
{
  struct grub_xhci *xhci = dev->data;
  struct grub_xhci_transfer_controller_data *cdata =
    transfer->controller_data;
  grub_uint32_t token;
  grub_usb_err_t err = GRUB_USB_ERR_NAK;

  /* QH should be halted and not active in this case. */

  grub_dprintf ("xhci", "parse_halt: info\n");

  /* Remember token before call pre-finish function */
  token = grub_le_to_cpu32 (cdata->qh_virt->td_overlay.token);

  /* Do things like in normal finish */
  grub_xhci_pre_finish_transfer (transfer);
  grub_free (cdata);

  sync_all_caches (xhci);

  /* Evaluation of error code - currently we don't have GRUB USB error
   * codes for some xHCI states, GRUB_USB_ERR_DATA is used for them.
   * Order of evaluation is critical, specially bubble/stall. */
  if ((token & GRUB_XHCI_STATUS_BABBLE) != 0)
    err = GRUB_USB_ERR_BABBLE;
  else if ((token & GRUB_XHCI_CERR_MASK) != 0)
    err = GRUB_USB_ERR_STALL;
  else if ((token & GRUB_XHCI_STATUS_TRANERR) != 0)
    err = GRUB_USB_ERR_DATA;
  else if ((token & GRUB_XHCI_STATUS_BUFERR) != 0)
    err = GRUB_USB_ERR_DATA;
  else if ((token & GRUB_XHCI_STATUS_MISSDMF) != 0)
    err = GRUB_USB_ERR_DATA;

  return err;
}
#endif

#if 0
static grub_usb_err_t
grub_xhci_parse_success (grub_usb_controller_t dev,
			 grub_usb_transfer_t transfer, grub_size_t * actual)
{
  (void)actual;
  struct grub_xhci *xhci = dev->data;
  struct grub_xhci_transfer_controller_data *cdata =
    transfer->controller_data;

  grub_dprintf ("xhci", "parse_success: info\n");

  /* QH should be not active in this case, but it is not halted. */
  grub_xhci_pre_finish_transfer (transfer);
  grub_free (cdata);

  sync_all_caches (xhci);

  return GRUB_USB_ERR_NONE;
}
#endif

#endif

static grub_err_t
grub_xhci_restore_hw (void)
{
  struct grub_xhci *xhci;
  //grub_uint32_t n_ports;
  //int i;

  grub_dprintf("xhci", "grub_xhci_restore_hw enter\n");
  /* We should re-enable all xHCI HW similarly as on inithw */
  for (xhci = xhci_list; xhci; xhci = xhci->next)
    {
      /* Check if xHCI is halted and halt it if not */
      if (grub_xhci_halt (xhci) != GRUB_USB_ERR_NONE)
	grub_error (GRUB_ERR_TIMEOUT, "restore_hw: xHCI halt timeout");

      /* Reset xHCI */
      if (grub_xhci_reset (xhci) != GRUB_USB_ERR_NONE)
	grub_error (GRUB_ERR_TIMEOUT, "restore_hw: xHCI reset timeout");

      /* Setup some xHCI registers and enable xHCI */
//      grub_xhci_oper_write32 (xhci, GRUB_XHCI_OPER_USBCMD,
//			      GRUB_XHCI_OPER_USBCMD_RUNSTOP |
//			      grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD));

      /* Now should be possible to power-up and enumerate ports etc. */
	  /* Power on all ports */
    }

  return GRUB_ERR_NONE;
}

static grub_err_t
grub_xhci_fini_hw (int noreturn __attribute__ ((unused)))
{
  struct grub_xhci *xhci;

  grub_dprintf ("xhci", "grub_xhci_fini_hw enter\n");

  /* We should disable all xHCI HW to prevent any DMA access etc. */
  for (xhci = xhci_list; xhci; xhci = xhci->next)
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

static grub_usb_err_t
grub_xhci_cancel_transfer (grub_usb_controller_t dev,
			   grub_usb_transfer_t transfer)
{
  struct grub_xhci *xhci = dev->data;
  struct grub_xhci_transfer_controller_data *cdata =
    transfer->controller_data;
  (void)cdata;
  (void)xhci;
  grub_dprintf ("xhci", "grub_xhci_cancel_transfer: begin\n");
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

  grub_dprintf ("xhci", "cancel_transfer: begin\n");

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
      grub_dprintf ("xhci", "cancel_transfer: end - xHCI not running\n");
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
                       & GRUB_XHCI_POINTER_MASK) | GRUB_XHCI_HPTR_TYPE_QH);

  grub_free (cdata);

  grub_dprintf ("xhci", "cancel_transfer: end\n");

  sync_all_caches (xhci);

#endif
  return GRUB_USB_ERR_NONE;
}

static grub_usb_speed_t
grub_xhci_detect_dev (grub_usb_controller_t dev, int port, int *changed)
{
  struct grub_xhci *xhci = (struct grub_xhci *) dev->data;
  grub_uint32_t status, line_state;

  (void)port;
  (void)changed;
  (void)xhci;
  (void)line_state;
  (void)status;

  //grub_dprintf ("xhci", "grub_xhci_detect_dev enter\n");
  grub_millisleep(2);
  *changed = 0;
  return GRUB_USB_SPEED_HIGH;

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

static grub_usb_err_t
grub_xhci_portstatus (grub_usb_controller_t dev,
		      unsigned int port, unsigned int enable)
{
  (void)dev;
  (void)port;
  (void)enable;
  grub_dprintf ("xhci", "grub_xhci_portstatus enter (port=%d, enable=%d)\n",
      port, enable);
  return GRUB_USB_ERR_NONE;

#if 0
  struct grub_xhci *xhci = (struct grub_xhci *) dev->data;
  grub_uint64_t endtime;

  grub_dprintf ("xhci", "portstatus: xHCI USBSTS: %08x\n",
		grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBSTS));
  grub_dprintf ("xhci",
		"portstatus: begin, iobase_cap=%p, port=%d, status=0x%02x\n",
		xhci->iobase_cap, port, grub_xhci_port_read (xhci, port));

  /* In any case we need to disable port:
   * - if enable==false - we should disable port
   * - if enable==true we will do the reset and the specification says
   *   PortEnable should be FALSE in such case */
  /* Disable the port and wait for it. */
  grub_xhci_cap_read32 (xhci, GRUB_XHCI_PORTSC(port));
  endtime = grub_get_time_ms () + 1000;
  while (grub_xhci_port_read (xhci, port) & GRUB_XHCI_PORT_ENABLED)
    if (grub_get_time_ms () > endtime)
      return GRUB_USB_ERR_TIMEOUT;

  if (!enable)			/* We don't need reset port */
    {
      grub_dprintf ("xhci", "portstatus: Disabled.\n");
      grub_dprintf ("xhci", "portstatus: end, status=0x%02x\n",
		    grub_xhci_port_read (xhci, port));
      return GRUB_USB_ERR_NONE;
    }

  grub_dprintf ("xhci", "portstatus: enable\n");

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
      grub_dprintf ("xhci", "portstatus: Enabled!\n");
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

  grub_dprintf ("xhci", "portstatus: end, status=0x%02x\n",
		grub_xhci_port_read (xhci, port));

#endif
  return GRUB_USB_ERR_NONE;
}

static int
grub_xhci_hubports (grub_usb_controller_t dev)
{
  (void)dev;
  //struct grub_xhci *xhci = (struct grub_xhci *) dev->data;
  //grub_uint32_t hcsparams1;
  unsigned int nports = 1;

  //hcsparams1 = grub_xhci_cap_read32 (xhci, GRUB_XHCI_CAP_HCSPARAMS1);
  //nports = ((hcsparams1 >> 24) & 0xff);
  //grub_dprintf ("xhci", "root hub ports=%d\n", nports);
  return nports;
}

static grub_usb_err_t
grub_xhci_check_transfer (grub_usb_controller_t dev,
			  grub_usb_transfer_t transfer, grub_size_t * actual)
{
  struct grub_xhci *xhci = dev->data;
  struct grub_xhci_transfer_controller_data *cdata =
    transfer->controller_data;
  (void)cdata;
  (void)xhci;
  (void)actual;

  grub_dprintf ("xhci", "grub_xhci_check_transfer enter\n");
  return GRUB_USB_ERR_NONE;
#if 0
  grub_uint32_t token, token_ftd;

  sync_all_caches (xhci);

  grub_dprintf ("xhci",
		"check_transfer: xHCI STATUS=%08x, cdata=%p, qh=%p\n",
		grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBSTS),
		cdata, cdata->qh_virt);
  grub_dprintf ("xhci", "check_transfer: qh_hptr=%08x, ep_char=%08x\n",
		grub_le_to_cpu32 (cdata->qh_virt->qh_hptr),
		grub_le_to_cpu32 (cdata->qh_virt->ep_char));
  grub_dprintf ("xhci", "check_transfer: ep_cap=%08x, td_current=%08x\n",
		grub_le_to_cpu32 (cdata->qh_virt->ep_cap),
		grub_le_to_cpu32 (cdata->qh_virt->td_current));
  grub_dprintf ("xhci", "check_transfer: next_td=%08x, alt_next_td=%08x\n",
		grub_le_to_cpu32 (cdata->qh_virt->td_overlay.next_td),
		grub_le_to_cpu32 (cdata->qh_virt->td_overlay.alt_next_td));
  grub_dprintf ("xhci", "check_transfer: token=%08x, buffer[0]=%08x\n",
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


static grub_usb_err_t
grub_xhci_setup_transfer (grub_usb_controller_t dev,
			  grub_usb_transfer_t transfer)
{
  (void)dev;
  (void)transfer;
  grub_dprintf ("xhci", "grub_xhci_setup_transfer enter\n");
  /* pretend we managed to start sending data */
  return GRUB_USB_ERR_NONE;

#if 0
  struct grub_xhci *xhci = (struct grub_xhci *) dev->data;
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
      grub_dprintf ("xhci", "setup_transfer: halted, status = 0x%x\n",
		    status);
      return GRUB_USB_ERR_INTERNAL;
    }
  status = grub_xhci_oper_read32 (xhci, GRUB_XHCI_STATUS);
  if ((status
       & (GRUB_XHCI_ST_AS_STATUS | GRUB_XHCI_ST_PS_STATUS)) == 0)
    /* XXX: Fix it: Currently we don't do anything to restart xHCI */
    {
      grub_dprintf ("xhci", "setup_transfer: no AS/PS, status = 0x%x\n",
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
      grub_dprintf ("xhci", "setup_transfer: no QH\n");
      grub_free (cdata);
      return GRUB_USB_ERR_INTERNAL;
    }

  /* To detect short packet we need some additional "alternate" TD,
   * allocate it first. */
  cdata->td_alt_virt = 0; //grub_xhci_alloc_td (xhci);
  if (!cdata->td_alt_virt)
    {
      grub_dprintf ("xhci", "setup_transfer: no TDs\n");
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
	  grub_dprintf ("xhci", "setup_transfer: no TD\n");
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

  grub_dprintf ("xhci", "setup_transfer: cdata=%p, qh=%p\n",
		cdata,cdata->qh_virt);
  grub_dprintf ("xhci", "setup_transfer: td_first=%p, td_alt=%p\n",
		cdata->td_first_virt,
		cdata->td_alt_virt);
  grub_dprintf ("xhci", "setup_transfer: td_last=%p\n",
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


static int
grub_xhci_iterate (grub_usb_controller_iterate_hook_t hook, void *hook_data)
{
  struct grub_xhci *xhci;
  struct grub_usb_controller dev;
  (void)dev;

  grub_dprintf ("xhci", "grub_xhci_iterate enter\n");
  for (xhci = xhci_list; xhci; xhci = xhci->next)
    {
      dev.data = xhci;
      grub_dprintf ("xhci",
          "grub_xhci_iterate: invoking hook (0x%08x) with data (0x%08x)\n",
          (unsigned int)hook, (unsigned int)hook_data);
      if (hook (&dev, hook_data))
        {
          grub_dprintf ("xhci", "grub_xhci_iterate: hook completed with status != 0\n");
          return 1;
        }
      else
        {
          grub_dprintf ("xhci", "grub_xhci_iterate: hook completed with status == 0\n");
        }
    }

  return 0;
}

/* PCI iteration function... */
static int
grub_xhci_pci_iter (grub_pci_device_t dev,
                    grub_pci_id_t pciid __attribute__ ((unused)),
		    void *data __attribute__ ((unused)))
{
  struct grub_xhci *xhci;
  //grub_uint8_t caplen;
  //grub_uint32_t hccparams1;
  grub_uint32_t class_code;

  /* Exit if not USB3.0 xHCI controller */
  class_code = pci_config_read (dev, GRUB_PCI_REG_CLASS);
  if ((class_code >> 8) != 0x0c0330)
    return 0;

  grub_dprintf ("xhci", "found xHCI controller on bus %d device %d "
      "function %d: device|vendor ID 0x%08x\n", dev.bus, dev.device,
      dev.function, pciid);

  xhci = grub_malloc (sizeof (*xhci));
  if (!xhci)
    {
      grub_dprintf ("xhci", "out of memory\n");
      return GRUB_USB_ERR_INTERNAL;
    }

  /* BUild list of xHCI controllers */
  xhci->next = xhci_list;
  xhci_list = xhci;

  return 0;
}

static struct grub_usb_controller_dev usb_controller_dev = {
  .name = "xhci",
  .iterate = grub_xhci_iterate,
  .setup_transfer = grub_xhci_setup_transfer, /* give data to HW, let it go */
  .check_transfer = grub_xhci_check_transfer, /* check if HW has completed transfer, polled by USB framework (see usbtrans.c) */
  .cancel_transfer = grub_xhci_cancel_transfer, /* called if/when check_transfer has failed over a period of time */
  .hubports = grub_xhci_hubports,
  .portstatus = grub_xhci_portstatus,
  .detect_dev = grub_xhci_detect_dev,
  /* estimated max. count of TDs for one bulk transfer */
  .max_bulk_tds = 16, //GRUB_EHCI_N_TD * 3 / 4 
};

GRUB_MOD_INIT (xhci)
{
  grub_dprintf ("xhci", "[loading]\n");

  /* TODO: anything to compile time check? */
  //COMPILE_TIME_ASSERT (sizeof (struct grub_xhci_FOO) == 64);

  grub_stop_disk_firmware ();

  grub_boot_time ("Initing xHCI hardware");
  grub_pci_iterate (grub_xhci_pci_iter, NULL);
  grub_boot_time ("Registering xHCI driver");
  grub_usb_controller_dev_register (&usb_controller_dev);
  grub_boot_time ("xHCI driver registered");
  grub_dprintf ("xhci", "xHCI driver is registered, register preboot hook\n");
  grub_loader_register_preboot_hook (grub_xhci_fini_hw, grub_xhci_restore_hw,
				     GRUB_LOADER_PREBOOT_HOOK_PRIO_DISK);
  grub_dprintf ("xhci", "GRUB_MOD_INIT completed\n");
}

GRUB_MOD_FINI (xhci)
{
  grub_dprintf ("xhci", "[unloading]\n");
  grub_xhci_fini_hw (0);
  grub_usb_controller_dev_unregister (&usb_controller_dev);
}
