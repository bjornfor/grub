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

GRUB_MOD_LICENSE ("GPLv3+");

/* Where is the standard "offsetof" macro */
#define OFFSETOF(T, m) \
  ((grub_size_t) (((grub_uint8_t *) &(((T*)NULL)->m)) - ((grub_uint8_t *) ((T*) NULL))))

/*
 * Some reasons for doing this kind of register access abstraction. It has a
 * fairly good balance between type safety and convenience. Bit-fields are not
 * portable. Plain bitshifts with macros can be error prone - must never forget
 * to use (a bit ad-hoc). It may be a bit slower than using macros, but it's
 * more maintainable/readable.
 */

/** For initializing enum bits values with start and end bit positions. */
#define BITS(start, end) ((start << 16) | (end - start + 1))

enum bits32
{
  ALL_BITS = BITS(0, 31),

  /*
   * Capability Registers
   */
  XHCI_CAP_CAPLENGTH                         = BITS(0, 7),
  XHCI_CAP_HCIVERSION                        = BITS(16, 31),
  XHCI_CAP_HCSPARAMS1_MAX_DEVICE_SLOTS       = BITS(0, 7),
  XHCI_CAP_HCSPARAMS1_MAX_INTERRUPTERS       = BITS(8, 18),
  /* Rsvd */
  XHCI_CAP_HCSPARAMS1_MAX_PORTS              = BITS(24, 31),
  XHCI_CAP_HCSPARAMS2_IST                    = BITS(0, 3),
  XHCI_CAP_HCSPARAMS2_ERST_MAX               = BITS(4, 7),
  /* Rsvd */
  XHCI_CAP_HCSPARAMS2_MAX_SCRATCH_BUFS_HI    = BITS(21, 25),
  XHCI_CAP_HCSPARAMS2_SPR                    = BITS(26, 26),
  XHCI_CAP_HCSPARAMS2_MAX_SCRATCH_BUFS_LO    = BITS(27, 31),
  XHCI_CAP_HCSPARAMS3_U1_DEVICE_EXIT_LATENCY = BITS(0, 7),
  /* Rsvd */
  XHCI_CAP_HCSPARAMS3_U2_DEVICE_EXIT_LATENCY = BITS(16, 31),
  XHCI_CAP_HCCPARAMS1_AC64                   = BITS(0, 0),
  XHCI_CAP_HCCPARAMS1_BNC                    = BITS(1, 1),
  XHCI_CAP_HCCPARAMS1_CSZ                    = BITS(2, 2),
  XHCI_CAP_HCCPARAMS1_PPC                    = BITS(3, 3),
  XHCI_CAP_HCCPARAMS1_PIND                   = BITS(4, 4),
  XHCI_CAP_HCCPARAMS1_LHRC                   = BITS(5, 5),
  XHCI_CAP_HCCPARAMS1_LTC                    = BITS(6, 6),
  XHCI_CAP_HCCPARAMS1_NSS                    = BITS(7, 7),
  XHCI_CAP_HCCPARAMS1_PAE                    = BITS(8, 8),
  XHCI_CAP_HCCPARAMS1_SPC                    = BITS(9, 9),
  XHCI_CAP_HCCPARAMS1_SEC                    = BITS(10, 10),
  XHCI_CAP_HCCPARAMS1_CFC                    = BITS(11, 11),
  XHCI_CAP_HCCPARAMS1_MAX_PSA_SIZE           = BITS(12, 15),
  /* xHCI Extended Capabilities Pointer, offset in 32-bit words (4 bytes) */
  XHCI_CAP_HCCPARAMS1_XECP                   = BITS(16, 31),
  /* Offset in 32-bit words (4 bytes) */
  XHCI_CAP_DBOFF                             = BITS(2, 31),
  /* Offset in 32-byte words (yes BYTES) */
  XHCI_CAP_RTSOFF                            = BITS(5, 31),
  XHCI_CAP_HCCPARAMS2_U3C                    = BITS(0, 0),
  XHCI_CAP_HCCPARAMS2_CMC                    = BITS(1, 1),
  XHCI_CAP_HCCPARAMS2_FSC                    = BITS(2, 2),
  XHCI_CAP_HCCPARAMS2_CTC                    = BITS(3, 3),
  XHCI_CAP_HCCPARAMS2_LEC                    = BITS(4, 4),
  XHCI_CAP_HCCPARAMS2_CIC                    = BITS(5, 5),
  /* Rsvd 0x20 - CAPLENGTH */

  /*
   * Operational Registers
   */
  /* USBCMD */
  XHCI_OP_USBCMD_RUNSTOP = BITS( 0,  0), /* Run = 1, Stop = 0 */
  XHCI_OP_USBCMD_HCRST   = BITS( 1,  1), /* Host Controller Reset */
  XHCI_OP_USBCMD_INTE    = BITS( 2,  2), /* Interrupter Enable */
  XHCI_OP_USBCMD_HSEE    = BITS( 3,  3), /* Host System Error Enable */
  /* RsvdP */
  XHCI_OP_USBCMD_LHCRST  = BITS( 7,  7), /* Light Host Controller Reset */
  XHCI_OP_USBCMD_CSS     = BITS( 8,  8), /* Controller Save State */
  XHCI_OP_USBCMD_CRS     = BITS( 9,  9), /* Controller Restore State */
  XHCI_OP_USBCMD_EWE     = BITS(10, 10), /* Enable Wrap Event */
  XHCI_OP_USBCMD_EU3S    = BITS(11, 11), /* Enable U3 MFINDEX Stop */
  XHCI_OP_USBCMD_SPE     = BITS(12, 12), /* Short Packet Enable */
  XHCI_OP_USBCMD_CME     = BITS(13, 13), /* CEM Enable */
  /* RsvdP */

  /* USBSTS */
  XHCI_OP_USBSTS_HCH  = BITS(0, 0), /* HCHalted */
  /* RsvdZ */
  XHCI_OP_USBSTS_HSE  = BITS(2, 2), /* Host System Error */
  XHCI_OP_USBSTS_EINT = BITS(3, 3), /* Event Interrupt */
  XHCI_OP_USBSTS_PCD  = BITS(4, 4), /* Port Change Detect */
  /* RsvdZ */
  XHCI_OP_USBSTS_SSS  = BITS(8, 8), /* Save State Status */
  XHCI_OP_USBSTS_RSS  = BITS(9, 9), /* Restore State Status */
  XHCI_OP_USBSTS_SRE  = BITS(10, 10), /* Save/Restore Error */
  XHCI_OP_USBSTS_CNR  = BITS(11, 11), /* Controller Not Ready */
  XHCI_OP_USBSTS_HCE  = BITS(12, 12), /* Host Controller Error */
  /* RsvdP */

  /* PAGESIZE */
  XHCI_OP_PAGESIZE    = BITS(0, 15),

  /* DNCTRL */
  XHCI_OP_DNCTRL_N0    = BITS(0, 0),
  XHCI_OP_DNCTRL_N1    = BITS(1, 1),
  XHCI_OP_DNCTRL_N2    = BITS(2, 2),
  XHCI_OP_DNCTRL_N3    = BITS(3, 3),
  XHCI_OP_DNCTRL_N4    = BITS(4, 4),
  XHCI_OP_DNCTRL_N5    = BITS(5, 5),
  XHCI_OP_DNCTRL_N6    = BITS(6, 6),
  XHCI_OP_DNCTRL_N7    = BITS(7, 7),
  XHCI_OP_DNCTRL_N8    = BITS(8, 8),
  XHCI_OP_DNCTRL_N9    = BITS(9, 9),
  XHCI_OP_DNCTRL_N10   = BITS(10, 10),
  XHCI_OP_DNCTRL_N11   = BITS(11, 11),
  XHCI_OP_DNCTRL_N12   = BITS(12, 12),
  XHCI_OP_DNCTRL_N13   = BITS(13, 13),
  XHCI_OP_DNCTRL_N14   = BITS(14, 14),
  XHCI_OP_DNCTRL_N15   = BITS(15, 15),
  /* RsvdP */

  /* CRCR */
  XHCI_OP_CRCR_RCS     = BITS(0, 0),
  XHCI_OP_CRCR_CS      = BITS(1, 1),
  XHCI_OP_CRCR_CA      = BITS(2, 2),
  XHCI_OP_CRCR_CRR     = BITS(3, 3),
  /* RsvdP */
  XHCI_OP_CRCR_CMD_RING_PTR_LO = BITS(6, 31),
  XHCI_OP_CRCR_CMD_RING_PTR_HI = BITS(0, 31),

  /* DCBAAP */
  XHCI_OP_DCBAAP_LO            = BITS(6, 31),
  XHCI_OP_DCBAAP_HI            = BITS(0, 31),

  /* CONFIG */
  XHCI_OP_CONFIG_MAX_SLOTS_EN  = BITS(0, 7),
  XHCI_OP_CONFIG_U3E           = BITS(8, 8),
  XHCI_OP_CONFIG_CIE           = BITS(9, 9),
  /* RsvdP */

  /* PORTSC */
  XHCI_OP_PORTSC_CCS        = BITS(0, 0), /* Current Connect Status */
  XHCI_OP_PORTSC_PED        = BITS(1, 1), /* Port Enabled/Disabled */
  /* RsvdZ */
  XHCI_OP_PORTSC_OCA        = BITS(3, 3), /* Over-current Active */
  XHCI_OP_PORTSC_PR         = BITS(4, 4), /* Port Reset */
  XHCI_OP_PORTSC_PLS        = BITS(5, 8), /* Port Link State */
  XHCI_OP_PORTSC_PP         = BITS(9, 9), /* Port Power */
  XHCI_OP_PORTSC_PS         = BITS(10, 13), /* Port Speed */
  XHCI_OP_PORTSC_PIC        = BITS(14, 15), /* Port Indicator Control */
  XHCI_OP_PORTSC_LWS        = BITS(16, 16), /* Port Link State Write Strobe */
  XHCI_OP_PORTSC_CSC        = BITS(17, 17), /* Connect Status Change */
  XHCI_OP_PORTSC_PEC        = BITS(18, 18), /* Port Enabled/Disabled Change */
  XHCI_OP_PORTSC_WRC        = BITS(19, 19), /* Warm Port Reset Change */
  XHCI_OP_PORTSC_OCC        = BITS(20, 20), /* Over-current Change */
  XHCI_OP_PORTSC_PRC        = BITS(21, 21), /* Port Reset Change */
  XHCI_OP_PORTSC_PLC        = BITS(22, 22), /* Port Link State Change */
  XHCI_OP_PORTSC_CEC        = BITS(23, 23), /* Port Config Error Change */
  XHCI_OP_PORTSC_CAS        = BITS(24, 24), /* Cold Attach Status */
  XHCI_OP_PORTSC_WCE        = BITS(25, 25), /* Wake on Connect Enable */
  XHCI_OP_PORTSC_WDE        = BITS(26, 26), /* Wake on Disconnect Enable */
  XHCI_OP_PORTSC_WOE        = BITS(27, 27), /* Wake on Over-current Enable */
  /* RsvdZ */
  XHCI_OP_PORTSC_DR         = BITS(30, 30), /* Device Removable */
  XHCI_OP_PORTSC_WPR        = BITS(31, 31), /* Warm Port Reset */

  /* PORTPMSC */
  XHCI_OP_PORTPMSC_U1_TIMEOUT = BITS(0, 7),
  XHCI_OP_PORTPMSC_U2_TIMEOUT = BITS(8, 15),
  XHCI_OP_PORTPMSC_FLA        = BITS(16, 16),
  /* RsvdP */

  /* PORTLI */

  /* PORTHLPMC */


  /*
   * Runtime Registers
   */


  /*
   * Doorbell Registers
   */
};

/** Maximum device slots enabled */
#define XHCI_CONFIG_MAX_SLOTS_EN(slots) ( (slots) << 0 )

/** Maximum device slots enabled mask */
#define XHCI_CONFIG_MAX_SLOTS_EN_MASK \
  XHCI_CONFIG_MAX_SLOTS_EN ( 0xff )

/* Offset relative to Operational Base */
#define XHCI_PORTSC(port) (0x400 + (0x10 * (port - 1)))

#define XHCI_ADDR_MEM_MASK	(~0xff)
#define XHCI_POINTER_MASK	(~0x1f)

/* USB Legacy Support Capability (USBLEGSUP) bits. Section 7.1.1 in [spec]. */
enum
{
  XHCI_USBLEGSUP_BIOS_OWNED = (1 << 16),
  XHCI_USBLEGSUP_OS_OWNED = (1 << 24)
};


/* Operational register PORTSC bits */
enum
{
  XHCI_PORTSC_CCS = (  1 <<  0), /* Current Connect Status */
  XHCI_PORTSC_PED = (  1 <<  1), /* Port Enabled/Disabled */
  /* RsvdZ */
  XHCI_PORTSC_OCA = (  1 <<  3), /* Over-current Active */
  XHCI_PORTSC_PR  = (  1 <<  4), /* Port Reset */
  XHCI_PORTSC_PLS = (0xf <<  5), /* Port Link State */
  XHCI_PORTSC_PP  = (  1 <<  9), /* Port Power */
  XHCI_PORTSC_PS  = (0xf << 10), /* Port Speed */
  XHCI_PORTSC_PIC = (0x2 << 14), /* Port Indicator Control */
  XHCI_PORTSC_LWS = (  1 << 16), /* Port Link State Write Strobe */
  XHCI_PORTSC_CSC = (  1 << 17), /* Connect Status Change */
  XHCI_PORTSC_PEC = (  1 << 18), /* Port Enabled/Disabled Change */
  XHCI_PORTSC_WRC = (  1 << 19), /* Warm Port Reset Change */
  XHCI_PORTSC_OCC = (  1 << 20), /* Over-current Change */
  XHCI_PORTSC_PRC = (  1 << 21), /* Port Reset Change */
  XHCI_PORTSC_PLC = (  1 << 22), /* Port Link State Change */
  XHCI_PORTSC_CEC = (  1 << 23), /* Port Config Error Change */
  XHCI_PORTSC_CAS = (  1 << 24), /* Cold Attach Status */
  XHCI_PORTSC_WCE = (  1 << 25), /* Wake on Connect Enable */
  XHCI_PORTSC_WDE = (  1 << 26), /* Wake on Disconnect Enable */
  XHCI_PORTSC_WOE = (  1 << 27), /* Wake on Over-current Enable */
  /* RsvdZ */
  XHCI_PORTSC_DR  = (  1 << 30), /* Device Removable */
  XHCI_PORTSC_WPR = (  1 << 31), /* Warm Port Reset */
};

/** Number of device slots */
#define XHCI_HCSPARAMS1_SLOTS(params) ( ( (params) >> 0 ) & 0xff )

/** Number of interrupters */
#define XHCI_HCSPARAMS1_INTRS(params) ( ( (params) >> 8 ) & 0x3ff )

/** Number of ports */
#define XHCI_HCSPARAMS1_PORTS(params) ( ( (params) >> 24 ) & 0xff )

/** Structural parameters 2 */
#define XHCI_CAP_HCSPARAMS2 0x08

/** Number of page-sized scratchpad buffers */
#define XHCI_HCSPARAMS2_SCRATCHPADS(params) \
	( ( ( (params) >> 16 ) & 0x3e0 ) | ( ( (params) >> 27 ) & 0x1f ) )

/** Capability parameters */
#define XHCI_CAP_HCCPARAMS1 0x10

static int debug_enabled(void)
{
  const char *debug = grub_env_get ("debug");

  return debug &&
    (grub_strword (debug, "all") || grub_strword (debug, "xhci"));
}

/**
 * A printf function which prefixes output with "FILE:LINENO: "
 * Only output anything if the GRUB debug variable contains "all" or "xhci".
 */
static void
xhci_trace(const char *fmt, ...)
{
  va_list args;

  if (debug_enabled())
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
static void
xhci_dbg(const char *fmt, ...)
{
  va_list args;

  if (debug_enabled())
    {
      va_start (args, fmt);
      grub_vprintf (fmt, args);
      va_end (args);
      grub_refresh ();
    }
}

static void
xhci_err(const char *fmt, ...)
{
  va_list args;

  va_start (args, fmt);
  grub_vprintf (fmt, args);
  va_end (args);
  grub_refresh ();
}

static grub_uint32_t
pci_config_read (grub_pci_device_t dev, unsigned int reg)
{
  grub_pci_address_t addr;
  addr = grub_pci_make_address (dev, reg);
  return grub_pci_read (addr);
}


static grub_uint8_t
pci_config_read8 (grub_pci_device_t dev, unsigned int reg)
{
  grub_pci_address_t addr;
  addr = grub_pci_make_address (dev, reg);
  return grub_pci_read_byte (addr);
}

static grub_uint16_t
pci_config_read16 (grub_pci_device_t dev, unsigned int reg)
{
  grub_pci_address_t addr;
  addr = grub_pci_make_address (dev, reg);
  return grub_le_to_cpu16 (grub_pci_read_word (addr) );
}

static grub_uint32_t
pci_config_read32 (grub_pci_device_t dev, unsigned int reg)
{
  grub_pci_address_t addr;
  addr = grub_pci_make_address (dev, reg);
  return grub_le_to_cpu32 (grub_pci_read (addr) );
}

static void
pci_config_write32 (grub_pci_device_t dev, unsigned int reg, grub_uint32_t val)
{
  grub_pci_address_t addr;
  addr = grub_pci_make_address (dev, reg);
  return grub_pci_write (addr, grub_cpu_to_le32 (val));
}

#define DBOFF_TO_BYTES(value) ((value) * 4)
#define RTSOFF_TO_BYTES(value) ((value) * 32)

/** Capability registers */
struct xhci_cap_regs {
  /* These are read only, so we don't need volatile */
  //const grub_uint8_t caplength;
  //const grub_uint8_t _rsvd1;
  //const grub_uint16_t hciversion;
  const grub_uint32_t caplength_and_hciversion;
  const grub_uint32_t hcsparams1;
  const grub_uint32_t hcsparams2;
  const grub_uint32_t hcsparams3;
  const grub_uint32_t hccparams1;
  const grub_uint32_t dboff;
  const grub_uint32_t rtsoff;
  const grub_uint32_t hccparams2;
  /* Reserved up to (caplength - 0x20) */
};

/** Operational registers */
struct xhci_oper_regs {
  /** USB Command */
  volatile grub_uint32_t usbcmd;
  /** USB Status */
  volatile grub_uint32_t usbsts;
  /* Page Size */
  volatile grub_uint32_t pagesize;
  /** Reserved 0x0c-0x13 */
  grub_uint32_t _rsvdz1[2];
  /** Device Notification Control */
  volatile grub_uint32_t dnctrl;
  /** Command Ring Control */
  union {
    struct {
      volatile grub_uint32_t crcr_lo;
      volatile grub_uint32_t crcr_hi;
    };
    volatile grub_uint64_t crcr;
  };
  /** Reserved 0x20-0x2F */
  grub_uint32_t _rsvdz2[4];
  /** Device Context Base Address Array Pointer */
  union {
    struct {
      volatile grub_uint32_t dcbaap_lo;
      volatile grub_uint32_t dcbaap_hi;
    };
    volatile grub_uint64_t dcbaap;
  };
  /** Configure */
  volatile grub_uint32_t config;
  /** Reserved 0x03c-0x3ff */
  grub_uint32_t _rsvdz4[241];
  /** Port Register Set 1-MaxPorts (0x400-0x13ff) */
  volatile grub_uint32_t _reserved[1024];
};

/** Runtime registers */
struct xhci_run_regs {
  const volatile grub_uint32_t microframe_index;
};

/** Port Register Set */
//struct xhci_port_reg_set {
//  volatile grub_uint32_t portsc;
//  volatile grub_uint32_t portpmsc;
//  volatile grub_uint32_t portli;
//  volatile grub_uint32_t porthlpmc;
//};

#define MAX_DOORBELL_ENTRIES 256

/** Doorbell array registers */
struct xhci_doorbell_regs {
  volatile grub_uint32_t doorbell[MAX_DOORBELL_ENTRIES];
};

/** Construct slot context device info */
#define XHCI_SLOT_INFO(entries, hub, speed, route) \
  (((entries) << 27) | ((hub) << 26) | ((speed) << 20) | (route))

struct xhci_slot_context {
  grub_uint32_t info;
  grub_uint16_t max_exit_latency;
  grub_uint8_t root_hub_port_number;
  grub_uint8_t number_of_ports;
  /* TODO */
};

struct xhci_foo {
  int TODO;
};

#define NUM_DEVICE_CONTEXT_SUB_ELEMENTS 32
/**
 * Describes the characteristics and current state of individual USB devices
 * attached to the host controller
 */
struct xhci_device_context {
  struct xhci_foo foo[NUM_DEVICE_CONTEXT_SUB_ELEMENTS];
};

/**
 * Max USB devices supported by xHCI. Each device is assigned to a Device Slot
 * ID.
 */
#define MAX_USB_DEVICES 255
struct xhci_dcbaa
{
  //struct xhci_device_context *device_context[MAX_USB_DEVICES];
  /* points to a Device Context. We have to ensure this is 64-bit, else we
   * would have used real pointers.
   */
  grub_uint64_t device_context_ptr; /* dynamically allocated array, up to MAX_USB_DEVICES in size */
};

struct xhci
{
  /* Register addresses. */
  volatile struct xhci_cap_regs *cap_regs;
  volatile struct xhci_oper_regs *oper_regs;
  volatile struct xhci_run_regs *run_regs;
  volatile struct xhci_doorbell_regs *db_regs;

  /* Cached from xHCI */
  grub_uint8_t max_device_slots; /* valid range 1-255 */
  grub_uint8_t max_ports; /* valid range 1-255 */

  /* Other data */
  grub_uint8_t num_enabled_slots;
  grub_uint64_t *dcbaa;     /* virtual address */
  grub_uint32_t dcbaa_len;  /* size in bytes */

  /* linked list */
  struct xhci *next;


  /* DEPRECATED STUFF BELOW */

  volatile void *regs;     /* Start of registers (same addr as capability) */
  /* Pointers to specific register areas */
  volatile grub_uint8_t *cap;	   /* Capability registers */
  volatile grub_uint8_t *oper;	   /* Operational registers */
  volatile grub_uint8_t *runtime;  /* Runtime registers */
  //volatile grub_uint8_t *doorbell; /* Doorbell Array */

  unsigned int slots;  /* number of device slots */
  unsigned int ports;  /* number of ports */

  /* grub stuff */
  volatile grub_uint32_t *iobase_cap;	   /* Capability registers */
  volatile grub_uint32_t *iobase_oper;	   /* Operational registers */
  //unsigned int reset;
};

static struct xhci *xhci_list;

/* xHCI capability registers access functions */
static inline grub_uint32_t
xhci_cap_read32 (struct xhci *xhci, grub_uint32_t off)
{
  return grub_le_to_cpu32 (*((volatile grub_uint32_t *) xhci->cap +
		       (off / sizeof (grub_uint32_t))));
}

static inline grub_uint8_t
mmio_read8 (const volatile grub_uint8_t *addr)
{
  return *addr;
}

static inline grub_uint16_t
mmio_read16 (const volatile grub_uint16_t *addr)
{
  return grub_le_to_cpu16 (*addr);
}

static inline grub_uint32_t
mmio_read32 (const volatile grub_uint32_t *addr)
{
  return grub_le_to_cpu32 (*addr);
}

static inline grub_uint64_t
mmio_read64 (const volatile grub_uint64_t *addr)
{
  return grub_le_to_cpu64 (*addr);
}

static inline void
mmio_write8 (volatile grub_uint8_t *addr, grub_uint8_t val)
{
  *addr = val;
}

static inline void
mmio_write16 (volatile grub_uint16_t *addr, grub_uint16_t val)
{
  *addr = grub_cpu_to_le16 (val);
}

static inline void
mmio_write32 (volatile grub_uint32_t *addr, grub_uint32_t val)
{
  *addr = grub_cpu_to_le32 (val);
}

static inline void
mmio_set_bits(volatile grub_uint32_t *addr, grub_uint32_t bits)
{
  mmio_write32(addr, mmio_read32(addr) | bits);
}

static inline void
mmio_clear_bits(volatile grub_uint32_t *addr, grub_uint32_t bits)
{
  mmio_write32(addr, mmio_read32(addr) & ~bits);
}

static inline grub_uint32_t
parse_reg(grub_uint32_t regval, const enum bits32 bits)
{
  const grub_uint32_t bitno = bits >> 16;
  const grub_uint32_t width = bits & 0xff;

  regval >>= bitno;
  regval &= ((1 << width) - 1);
  return regval;
}

/**
 * Read a MMIO register. Masking and shifting is done automatically with
 * 'bits'.
 */
static inline grub_uint32_t
mmio_read_bits(const volatile grub_uint32_t *addr, const enum bits32 bits)
{
  grub_uint32_t regval;

  regval = grub_le_to_cpu32 (*addr);
  return parse_reg(regval, bits);
}

/**
 * Write a MMIO register. Masking and shifting is done automatically with
 * 'bits'. The register is read first, so existing bits are preserved.
 */
static inline void
mmio_write_bits(volatile grub_uint32_t *addr, const enum bits32 bits, grub_uint32_t val)
{
  grub_uint32_t regval;
  const grub_uint32_t bitno = bits >> 16;
  const grub_uint32_t width = bits & 0xff;

  regval = mmio_read32(addr);
  regval &= ~(((1 << width) - 1) << bitno);
  regval |= val << bitno;
  mmio_write32(addr, regval);
}

enum xhci_portrs_type
{
  PORTSC = 0,
  PORTPMSC = 4,
  PORTLI = 8,
  PORTHLPMC = 12,
};

/* Read Port Register Set n of given type */
static inline grub_uint32_t
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

static int
xhci_dump_cap(struct xhci *xhci)
{
  if (!debug_enabled())
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

static void
xhci_dump_oper_portsc(struct xhci *xhci, int port)
{
  grub_uint32_t portsc;

  portsc = xhci_read_portrs (xhci, port, PORTSC);
  grub_printf (" PORTSC(%02d)=0x%08x%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s",
      port, portsc
      , parse_reg(portsc, XHCI_OP_PORTSC_CCS) ? " CCS" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_PED) ? " PED" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_OCA) ? " OCA" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_PR) ? " PR" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_PLS) ? " PLS" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_PP) ? " PP" : ""
      , parse_reg(portsc, XHCI_OP_PORTSC_PS) ? " PS" : ""
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

static int
xhci_dump_oper(struct xhci *xhci)
{
  int i;
  grub_uint32_t val32;
  char extra[256];

  if (!debug_enabled())
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
      val32, 1 << (val32 + 12));

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

static grub_err_t
xhci_restore_hw (void)
{
  struct xhci *xhci;
  //grub_uint32_t n_ports;
  //int i;

  xhci_trace("grub_xhci_restore_hw enter\n");
  /* We should re-enable all xHCI HW similarly as on inithw */
  for (xhci = xhci_list; xhci; xhci = xhci->next)
    {
      /* Check if xHCI is halted and halt it if not */
      if (xhci_halt (xhci) != GRUB_USB_ERR_NONE)
	grub_error (GRUB_ERR_TIMEOUT, "restore_hw: xHCI halt timeout");

      /* Reset xHCI */
      if (xhci_reset (xhci) != GRUB_USB_ERR_NONE)
	grub_error (GRUB_ERR_TIMEOUT, "restore_hw: xHCI reset timeout");

      /* Setup some xHCI registers and enable xHCI */
//      grub_xhci_oper_write32 (xhci, GRUB_XHCI_OPER_USBCMD,
//			      XHCI_USBCMD_RUNSTOP |
//			      grub_xhci_oper_read32 (xhci, GRUB_XHCI_OPER_USBCMD));

      /* Now should be possible to power-up and enumerate ports etc. */
	  /* Power on all ports */
    }

  return GRUB_ERR_NONE;
}

static grub_err_t
xhci_fini_hw (int noreturn __attribute__ ((unused)))
{
  struct xhci *xhci;

  xhci_trace ("grub_xhci_fini_hw enter\n");

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
xhci_cancel_transfer (grub_usb_controller_t dev,
			   grub_usb_transfer_t transfer)
{
  struct xhci *xhci = dev->data;
  struct grub_xhci_transfer_controller_data *cdata =
    transfer->controller_data;
  (void)cdata;
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

static grub_usb_speed_t
xhci_detect_dev (grub_usb_controller_t dev, int port, int *changed)
{
  struct xhci *xhci = (struct xhci *) dev->data;
  grub_uint32_t status, line_state;

  (void)port;
  (void)changed;
  (void)xhci;
  (void)line_state;
  (void)status;
  static int state;
  grub_uint32_t portsc;

  xhci_trace ("xhci_detect_dev port=%d\n", port);
  if (debug_enabled())
  {
    xhci_dump_oper_portsc(xhci, port);
  }
  portsc = xhci_read_portrs (xhci, port, PORTSC);
  if (portsc & XHCI_PORTSC_CCS)
  {
    grub_printf ("xHCI port %d IS CONNECTED!!!\n", port);
  }

  if (debug_enabled())
    grub_millisleep (1000);

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

static grub_usb_err_t
xhci_portstatus (grub_usb_controller_t dev,
		      unsigned int port, unsigned int enable)
{
  (void)dev;
  (void)port;
  (void)enable;
  xhci_trace ("xhci_portstatus enter (port=%d, enable=%d)\n",
      port, enable);
  return GRUB_USB_ERR_NONE;

#if 0
  struct xhci *xhci = (struct xhci *) dev->data;
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

static int
xhci_hubports (grub_usb_controller_t dev)
{
  struct xhci *xhci = (struct xhci *) dev->data;
  unsigned int nports = 0;

  nports = xhci->max_ports;
  xhci_trace ("xhci_hubports nports=%d\n", nports);

  //xhci_trace ("xhci_hubports force nports=0 (prevent hang)\n");
  //nports = 0;
  //xhci->max_ports = nports;
  return nports;
}

static grub_usb_err_t
xhci_check_transfer (grub_usb_controller_t dev,
			  grub_usb_transfer_t transfer, grub_size_t * actual)
{
  struct xhci *xhci = dev->data;
  struct grub_xhci_transfer_controller_data *cdata =
    transfer->controller_data;
  (void)cdata;
  (void)xhci;
  (void)actual;

  xhci_trace ("xhci_check_transfer enter (TODO: implement)\n");
  return GRUB_USB_ERR_NONE;
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


static grub_usb_err_t
xhci_setup_transfer (grub_usb_controller_t dev,
			  grub_usb_transfer_t transfer)
{
  (void)dev;
  (void)transfer;
  xhci_trace ("xhci_setup_transfer enter (TODO: implement)\n");
  /* pretend we managed to start sending data */
  return GRUB_USB_ERR_NONE;

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


static int
xhci_iterate (grub_usb_controller_iterate_hook_t hook, void *hook_data)
{
  struct xhci *xhci;
  struct grub_usb_controller dev;
  (void)dev;

  xhci_trace ("xhci_iterate enter\n");
  for (xhci = xhci_list; xhci; xhci = xhci->next)
    {
      dev.data = xhci;
      if (hook (&dev, hook_data))
          return 1;
    }

  return 0;
}

#if 0
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
static int xhci_command(struct xhci *xhci, union xhci_trb *trb)
{
  struct xhci_trb_complete *complete = &trb->complete;
  unsigned int i;
  int rc;

  /* Record the pending command */
  xhci->pending = trb;

  /* Enqueue the command */
  if ( ( rc = xhci_enqueue ( &xhci->command, NULL, trb ) ) != 0 )
    goto err_enqueue;

  /* Ring the command doorbell */
  xhci_doorbell ( &xhci->command );

  /* Wait for the command to complete */
  for ( i = 0 ; i < XHCI_COMMAND_MAX_WAIT_MS ; i++ ) {

    /* Poll event ring */
    xhci_event_poll ( xhci );

    /* Check for completion */
    if ( ! xhci->pending ) {
      if ( complete->code != XHCI_CMPLT_SUCCESS ) {
        rc = -ECODE ( complete->code );
        DBGC ( xhci, "XHCI %s command failed (code "
            "%d): %s\n", xhci->name, complete->code,
            strerror ( rc ) );
        DBGC_HDA ( xhci, 0, trb, sizeof ( *trb ) );
        return rc;
      }
      return 0;
    }

    /* Delay */
    mdelay ( 1 );
  }

  /* Timeout */
  DBGC ( xhci, "XHCI %s timed out waiting for completion\n", xhci->name );
  rc = -ETIMEDOUT;

  /* Abort command */
  xhci_abort ( xhci );

err_enqueue:
  xhci->pending = NULL;
  return rc;
}

/**
 * Issue NOP and wait for completion
 *
 * @v xhci		xHCI device
 * @ret rc		Return status code
 */
static inline int xhci_nop ( struct xhci_device *xhci ) {
	union xhci_trb trb;
	struct xhci_trb_common *nop = &trb.common;
	int rc;

	/* Construct command */
	memset ( nop, 0, sizeof ( *nop ) );
	nop->flags = XHCI_TRB_IOC;
	nop->type = XHCI_TRB_NOP_CMD;

	/* Issue command and wait for completion */
	if ( ( rc = xhci_command ( xhci, &trb ) ) != 0 )
		return rc;

	return 0;
}
#endif

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

  xhci->dcbaa_len = xhci->max_device_slots * sizeof (xhci->dcbaa[0]) + 1;
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

/** Allocate command ring memory and program xHC */
static int
xhci_setup_command_ring(struct xhci *xhci)
{
  (void)xhci;
  xhci_trace("%s: TODO: implement", __func__);
  return 0;
}

static int
xhci_init (struct xhci *xhci, volatile void *mmio_base_addr)
{
  grub_int32_t hcsparams1;
  //grub_uint32_t hcsparams2;
  //grub_uint32_t hccparams1;
  //grub_uint32_t pagesize;
  grub_uint64_t maxtime;

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
  xhci_allocate_dcbaa(xhci);
  xhci_program_dcbaap(xhci);

  /* Setup Command Ring */
  xhci_setup_command_ring(xhci);

  /* Setup interrupts (not supported in this driver) */

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

  if (debug_enabled())
  {
    xhci_dump_cap(xhci);
    xhci_dump_oper(xhci);
    grub_millisleep (10000);
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
  xhci->next = xhci_list;
  xhci_list = xhci;

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


/* PCI iteration function, to be passed to grub_pci_iterate.
 *
 * grub_pci_iterate will invoke this function for each PCI device that exists
 * in the system. This function checks if the device is an xHC and initializes
 * it. Return 0 to continue iterating over devices, != 0 to abort.
 */
static int
xhci_pci_iter (grub_pci_device_t dev,
                    grub_pci_id_t pciid __attribute__ ((unused)),
		    void *data __attribute__ ((unused)))
{
  int err;
  struct xhci *xhci;
  grub_uint32_t class_code;
  grub_uint32_t base;
  volatile grub_uint32_t *mmio_base_addr;
  grub_uint32_t base_h;

  /* Exit if not USB3.0 xHCI controller */
  class_code = pci_config_read32 (dev, GRUB_PCI_REG_CLASS) >> 8;
  if (class_code != 0x0c0330)
    return 0;

  xhci_trace ("found xHCI controller on bus %d device %d "
      "function %d: device|vendor ID 0x%08x\n", dev.bus, dev.device,
      dev.function, pciid);

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

  xhci_trace ("xHCI 32-bit MMIO regs (BAR0) at 0x%08x\n",
      (unsigned long int)mmio_base_addr);

  xhci = grub_zalloc (sizeof (*xhci));
  if (!xhci)
    {
      xhci_err ("out of memory\n");
      return GRUB_USB_ERR_INTERNAL;
    }

  err = xhci_init (xhci, mmio_base_addr);
  if (err)
  {
    grub_free(xhci);
    return err;
  }

  /* Build list of xHCI controllers */
  xhci->next = xhci_list;
  xhci_list = xhci;

  return 0;
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
  /* Sanity check register addresses.
   * No limits.h or CHAR_BIT available, use GRUB_CHAR_BIT.
   */
  COMPILE_TIME_ASSERT(GRUB_CHAR_BIT == 8);
  COMPILE_TIME_ASSERT(OFFSETOF(struct xhci_cap_regs, hccparams2) == 0x1c);
  COMPILE_TIME_ASSERT(OFFSETOF(struct xhci_oper_regs, config) == 0x38);

  xhci_trace ("[loading]\n");
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
  xhci_trace ("[unloading]\n");
  xhci_fini_hw (0);
  grub_usb_controller_dev_unregister (&usb_controller_dev);
}
