#ifndef XHCI_PRIVATE_H
#define XHCI_PRIVATE_H

#include <stdint.h> /* uint32_t */
#include <stddef.h> /* size_t */

#define COMPILE_TIME_ASSERT(cond) switch (0) { case 1: case !(cond): ; }

/* Where is the standard "offsetof" macro */
#define OFFSETOF(T, m) \
  ((size_t) (((uint8_t *) &(((T*)NULL)->m)) - ((uint8_t *) ((T*) NULL))))

/*
 * Some reasons for doing this kind of register access abstraction. It has a
 * fairly good balance between type safety and convenience. Bit-fields are not
 * portable. Plain bitshifts with macros can be error prone - must never forget
 * to use (a bit ad-hoc). It may be a bit slower than using macros, but it's
 * more maintainable/readable.
 */

/** For initializing enum bits values with start and end bit positions. */
#define BITS(start, end) ((start << 16) | (end - start + 1))

/** Obtain bitmask from BITS definition (only works for single bit masks) */
#define BIT(bits_value) (1 << (bits_value >> 16))

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

  /*
   * TRB control word bits. These bits apply to the 3rd word (of 4) in a TRB. A
   * word is 32-bit here. The meaning of the bits depend on the TRB type.
   */
  /* Only the TRB Type and the C bit (cycle bit) is common to all TRBs. All
   * other fields may vary according to the TRB Type.
   */
  XHCI_TRB_CTRL__TRB_TYPE = BITS(10, 15),
  XHCI_TRB_CTRL__C = BITS(0, 0),

  /* Normal TRB (Bulk and Interrupt transfer) */
  XHCI_TRB_CTRL__BEI = BITS(9, 9),
  XHCI_TRB_CTRL__IDT = BITS(6, 6),
  XHCI_TRB_CTRL__IOC = BITS(5, 5),
  XHCI_TRB_CTRL__CH  = BITS(4, 4),
  XHCI_TRB_CTRL__NS  = BITS(3, 3),
  XHCI_TRB_CTRL__ISP = BITS(2, 2),
  XHCI_TRB_CTRL__ENT = BITS(1, 1),

  /*
   * TRB status word bits. These bits apply to the 2nd word (of 4) in a TRB.
   */
  /* Command Completion Event */
  XHCI_TRB_STAT__COMPLETION_CODE = BITS(24, 31),
};

#define XHCI_PCI_SBRN_REG  0x60

#define XHCI_POINTER_MASK	(~0x1f)
#define XHCI_COMMAND_MAX_WAIT_MS 100

/** Number of TRBs in the event ring
 *
 * This is a policy decision.
 */
#define XHCI_EVENT_TRBS_LOG2 6

/* USB Legacy Support Capability (USBLEGSUP) bits. Section 7.1.1 in [spec]. */
enum
{
  XHCI_USBLEGSUP_BIOS_OWNED = (1 << 16),
  XHCI_USBLEGSUP_OS_OWNED = (1 << 24)
};

#define DBOFF_TO_BYTES(value) ((value) * 4)
#define RTSOFF_TO_BYTES(value) ((value) * 32)

/** Capability registers */
struct xhci_cap_regs {
  const uint32_t caplength_and_hciversion;
  const uint32_t hcsparams1;
  const uint32_t hcsparams2;
  const uint32_t hcsparams3;
  const uint32_t hccparams1;
  const uint32_t dboff;
  const uint32_t rtsoff;
  const uint32_t hccparams2;
  /* Reserved up to (caplength - 0x20) */
};

/** Operational registers */
struct xhci_oper_regs {
  /** USB Command */
  uint32_t usbcmd;
  /** USB Status */
  uint32_t usbsts;
  /* Page Size */
  uint32_t pagesize;
  /** Reserved 0x0c-0x13 */
  uint32_t _rsvdz1[2];
  /** Device Notification Control */
  uint32_t dnctrl;
  /** Command Ring Control */
  union {
    struct {
      uint32_t crcr_lo;
      uint32_t crcr_hi;
    };
    uint64_t crcr;
  };
  /** Reserved 0x20-0x2F */
  uint32_t _rsvdz2[4];
  /** Device Context Base Address Array Pointer */
  union {
    struct {
      uint32_t dcbaap_lo;
      uint32_t dcbaap_hi;
    };
    uint64_t dcbaap;
  };
  /** Configure */
  uint32_t config;
  /** Reserved 0x03c-0x3ff */
  uint32_t _rsvdz4[241];
  /** Port Register Set 1-MaxPorts (0x400-0x13ff) */
  uint32_t _reserved[1024];
};

struct xhci_interrupter_register_set
{
  uint32_t iman;
  uint32_t imod;
  uint32_t erstsz;
  uint32_t _rsvdp;
  uint64_t erstba;
  uint64_t erdp;
};

#define NUM_INTERRUPTER_REGISTER_SETS 1024

/** Runtime registers */
struct xhci_run_regs {
  const uint32_t microframe_index;
  uint8_t _rsvdz[0x1c];
  struct xhci_interrupter_register_set ir_set[NUM_INTERRUPTER_REGISTER_SETS];
};

/** Port Register Set */
//struct xhci_port_reg_set {
//  uint32_t portsc;
//  uint32_t portpmsc;
//  uint32_t portli;
//  uint32_t porthlpmc;
//};

/*
 * A.k.a. number of slots (index 0 is special, it's for the Host Controller
 * itself)
 */
#define MAX_DOORBELL_ENTRIES 256

/** Doorbell array registers */
struct xhci_doorbell_regs {
  uint32_t doorbell[MAX_DOORBELL_ENTRIES];
};

/** Construct slot context device info */
#define XHCI_SLOT_INFO(entries, hub, speed, route) \
  (((entries) << 27) | ((hub) << 26) | ((speed) << 20) | (route))

struct xhci_slot_context {
  uint32_t info;
  uint16_t max_exit_latency;
  uint8_t root_hub_port_number;
  uint8_t number_of_ports;
  /* TODO */
};

//struct xhci_foo {
//  int TODO;
//};
//
//#define NUM_DEVICE_CONTEXT_SUB_ELEMENTS 32
///**
// * Describes the characteristics and current state of individual USB devices
// * attached to the host controller
// */
//struct xhci_device_context {
//  struct xhci_slot_context slot_context;
//  //struct xhci_foo foo[NUM_DEVICE_CONTEXT_SUB_ELEMENTS];
//  union xhci_foo foo[NUM_DEVICE_CONTEXT_SUB_ELEMENTS];
//};

/* TRB Types */
enum
{
  /* Allowed in transfer ring */
  XHCI_TRB_TYPE_RESERVED = 0,
  XHCI_TRB_TYPE_NORMAL = 1,
  XHCI_TRB_TYPE_SETUP_STAGE = 2,
  XHCI_TRB_TYPE_DATA_STAGE = 3,
  XHCI_TRB_TYPE_STATUS_STAGE = 4,
  XHCI_TRB_TYPE_ISOCH_ALLOWED = 5,
  XHCI_TRB_TYPE_LINK = 6, /* exception: also allowed in command ring */
  XHCI_TRB_TYPE_EVENT_DATA = 7,
  XHCI_TRB_TYPE_NO_OP = 8,

  /* Allowed in command ring */
  XHCI_TRB_TYPE_ENABLE_SLOT_COMMAND = 9,
  XHCI_TRB_TYPE_DISABLE_SLOT_COMMAND = 10,
  XHCI_TRB_TYPE_ADDRESS_DEVICE_COMMAND = 11,
  XHCI_TRB_TYPE_CONFIGURE_ENDPOINT_COMMAND = 12,
  XHCI_TRB_TYPE_EVALUATE_CONTEXT_COMMAND = 13,
  XHCI_TRB_TYPE_RESET_ENDPOINT_COMMAND = 14,
  XHCI_TRB_TYPE_STOP_ENDPOINT_COMMAND = 15,
  XHCI_TRB_TYPE_SET_TR_DEQUEUE_POINTER_COMMAND = 16,
  XHCI_TRB_TYPE_RESET_DEVICE_COMMAND = 17,
  XHCI_TRB_TYPE_FORCE_EVENT_COMMAND = 18, /* optional, used with virtualization only */
  XHCI_TRB_TYPE_NEGOTIATE_BANDWIDTH_COMMAND = 19, /* optional */
  XHCI_TRB_TYPE_SET_LATENCY_TOLERANCE_VALUE_COMMAND = 20, /* optional */
  XHCI_TRB_TYPE_GET_PORT_BANDWIDTH_COMMAND = 21,
  XHCI_TRB_TYPE_FORCE_HEADER_COMMAND = 22,
  XHCI_TRB_TYPE_NO_OP_COMMAND = 23,

  /* 24-31 reserved */

  /* Allowed in event ring */
  XHCI_TRB_TYPE_TRANSFER_EVENT = 32,
  XHCI_TRB_TYPE_COMMAND_COMPLETION_EVENT = 33,
  XHCI_TRB_TYPE_PORT_STATUS_CHANGE_EVENT = 34,
  XHCI_TRB_TYPE_BANDWIDTH_REQUEST_EVENT = 35, /* optional */
  XHCI_TRB_TYPE_DOORBELL_EVENT = 36, /* optional, used with virtualization only */
  XHCI_TRB_TYPE_HOST_CONTROLLER_EVENT = 37,
  XHCI_TRB_TYPE_DEVICE_NOTIFICATION_EVENT = 38,
  XHCI_TRB_TYPE_MFINDEX_WRAP_EVENT = 39,

  /* 40-47 reserved */

  /* 48-63 vendor defined, optional in all rings */
};

/* TRB Completion Codes */
enum
{
  XHCI_TRB_COMPLETION_CODE__INVALID = 0,
  XHCI_TRB_COMPLETION_CODE__SUCCESS = 1,
  /* ... */
};

/** xHCI completion codes */
enum xhci_completion_code
{
  /** Success */
  XHCI_CMPLT_SUCCESS = 1,
  /** Error */
  XHCI_CMPLT_ERROR = 5,
  /** Short packet */
  XHCI_CMPLT_SHORT = 13,
  /** Command ring stopped */
  XHCI_CMPLT_CMD_STOPPED = 24,
};

/** A transfer request block */
struct xhci_trb_common
{
  /** Reserved */
  uint64_t _reserved1;
  /** Reserved */
  uint32_t _reserved2;
  /** Flags */
  uint8_t flags;
  /** Type */
  uint8_t type;
  /** Reserved */
  uint16_t _reserved3;
};

/* The generalized TRB template */
struct xhci_trb_template
{
  uint64_t parameter;
  uint32_t status;
  uint32_t control;
};

/** A port status change transfer request block */
struct xhci_trb_port_status
{
  /** Reserved */
  uint8_t reserved_a[3];
  /** Port ID */
  uint8_t port;
  /** Reserved */
  uint8_t reserved_b[7];
  /** Completion code */
  uint8_t code;
  /** Flags */
  uint8_t flags;
  /** Type */
  uint8_t type;
  /** Reserved */
  uint16_t reserved_c;
};

/** A port status change transfer request block */
struct xhci_trb_host_controller
{
  /** Reserved */
  uint64_t _reserved1;
  /** Reserved */
  uint8_t _reserved2[3];
  /** Completion code */
  uint8_t code;
  /** Flags */
  uint8_t flags;
  /** Type */
  uint8_t type;
  /** Reserved */
  uint16_t _reserved3;
};

/** A command completion event transfer request block */
struct xhci_trb_complete
{
  /** Command TRB pointer */
  uint64_t command;
  /** Parameter */
  uint8_t parameter[3];
  /** Completion code */
  uint8_t code;
  /** Flags */
  uint8_t flags;
  /** Type */
  uint8_t type;
  /** Virtual function ID */
  uint8_t vf;
  /** Slot ID */
  uint8_t slot;
};

//struct xhci_trb_nop
//{
//  uint32_t _resvdz[3];
//  uint32_t control;
//};

/* Transfer Request Block */
union xhci_trb
{
  struct xhci_trb_template templ;
  struct xhci_trb_common common;
  struct xhci_trb_complete complete;
  struct xhci_trb_host_controller host;
  //struct xhci_trb_nop nop;
  uint32_t raw[4];
};

/* Transfer Request Block ring */
struct xhci_trb_ring
{
  unsigned int prod;
  unsigned int cons;
  unsigned int mask;
  unsigned int shift;
  unsigned int link;
  unsigned int slot;

  /* Pointer to the Doorbell corresponding to this ring */
  volatile uint32_t *db_reg;
  volatile uint32_t db_val; // the value written to *db_reg to notify the xHC

  /* The actual TRBs in the ring, dynamically allocated from DMA pool, for
   * contiguous physical memory
   */
  volatile union xhci_trb *trbs;
};

/** An event ring segment */
struct xhci_event_ring_segment
{
  /** Base address */
  uint64_t base;
  /** Number of TRBs */
  uint32_t count;
  /** Reserved */
  uint32_t _reserved;
};

/** An event ring */
struct xhci_event_ring
{
  /** Consumer counter */
  unsigned int cons;
  /** Event ring segment table */
  struct xhci_event_ring_segment *segment;
  /** Transfer request blocks */
  volatile union xhci_trb *trb;
};

struct xhci
{
  /* Register addresses. */
  volatile struct xhci_cap_regs *cap_regs;
  volatile struct xhci_oper_regs *oper_regs;
  volatile struct xhci_run_regs *run_regs;
  volatile struct xhci_doorbell_regs *db_regs;

  /* Cached from xHCI */
  uint8_t max_device_slots; /* valid range 1-255 */
  uint8_t max_ports; /* valid range 1-255 */

  /* Other data */
  char name[16]; /* for identification purposes in debug output */
  int sbrn; /* Serial Bus Release Number register value */
  int pagesize; /* in bytes */
  uint8_t num_enabled_slots;
  uint64_t *dcbaa;     /* virtual address, dynamically allocated array
                               where each element points to a Slot Context */
  uint32_t dcbaa_len;  /* size in bytes */
  uint8_t *scratchpads;    /* virtual address, dynamically allocated array */
  uint8_t scratchpads_len;
  int num_scratch_bufs;
  uint64_t *scratchpad_arr;  /* virtual address, dynamically allocated array
                                     with physical address pointers to
                                     pagesized areas in "scratchpads" memory
                                     for xHC private use */
  uint32_t scratchpad_arr_len; /* size in bytes */

  volatile union xhci_trb *pending;

  struct xhci_trb_ring command_ring;

  struct xhci_event_ring event_ring;

  struct xhci_trb_ring transfer_ring;

  /* linked list */
  struct xhci *next;
};

/** Transfer request block cycle bit flag */
#define XHCI_TRB_C 0x01

/**
 * Calculate doorbell register value
 *
 * @v target		Doorbell target
 * @v stream		Doorbell stream ID
 * @ret dbval		Doorbell register value
 */
#define XHCI_DBVAL(target, stream) (((stream) << 16) | (target))

enum xhci_portrs_type
{
  /* byte offsets relative to the PORTRS(?) base register */
  PORTSC = 0,
  PORTPMSC = 4,
  PORTLI = 8,
  PORTHLPMC = 12,
};

#endif /* XHCI_PRIVATE_H */
