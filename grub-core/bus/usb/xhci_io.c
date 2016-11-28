#include <grub/types.h>
#include <grub/pci.h>
#include <grub/env.h>
#include <grub/misc.h> /* grub_strword */
#include <grub/term.h> /* grub_refresh */

#include "xhci_io.h"

grub_uint32_t pci_config_read (grub_pci_device_t dev, unsigned int reg)
{
  grub_pci_address_t addr;
  addr = grub_pci_make_address (dev, reg);
  return grub_pci_read (addr);
}

grub_uint8_t pci_config_read8 (grub_pci_device_t dev, unsigned int reg)
{
  grub_pci_address_t addr;
  addr = grub_pci_make_address (dev, reg);
  return grub_pci_read_byte (addr);
}

grub_uint16_t pci_config_read16 (grub_pci_device_t dev, unsigned int reg)
{
  grub_pci_address_t addr;
  addr = grub_pci_make_address (dev, reg);
  return grub_le_to_cpu16 (grub_pci_read_word (addr) );
}

grub_uint32_t pci_config_read32 (grub_pci_device_t dev, unsigned int reg)
{
  grub_pci_address_t addr;
  addr = grub_pci_make_address (dev, reg);
  return grub_le_to_cpu32 (grub_pci_read (addr) );
}

void pci_config_write32 (grub_pci_device_t dev, unsigned int reg, grub_uint32_t val)
{
  grub_pci_address_t addr;
  addr = grub_pci_make_address (dev, reg);
  return grub_pci_write (addr, grub_cpu_to_le32 (val));
}

///* xHCI capability registers access functions */
//grub_uint32_t
//xhci_cap_read32 (struct xhci *xhci, grub_uint32_t off)
//{
//  return grub_le_to_cpu32 (*((volatile grub_uint32_t *) xhci->cap +
//		       (off / sizeof (grub_uint32_t))));
//}

grub_uint8_t
mmio_read8 (const volatile grub_uint8_t *addr)
{
  return *addr;
}

grub_uint16_t
mmio_read16 (const volatile grub_uint16_t *addr)
{
  return grub_le_to_cpu16 (*addr);
}

grub_uint32_t
mmio_read32 (const volatile grub_uint32_t *addr)
{
  return grub_le_to_cpu32 (*addr);
}

grub_uint64_t
mmio_read64 (const volatile grub_uint64_t *addr)
{
  return grub_le_to_cpu64 (*addr);
}

void
mmio_write8 (volatile grub_uint8_t *addr, grub_uint8_t val)
{
  *addr = val;
}

void
mmio_write16 (volatile grub_uint16_t *addr, grub_uint16_t val)
{
  *addr = grub_cpu_to_le16 (val);
}

void
mmio_write32 (volatile grub_uint32_t *addr, grub_uint32_t val)
{
  *addr = grub_cpu_to_le32 (val);
}

void
mmio_write64 (volatile grub_uint64_t *addr, grub_uint64_t val)
{
  *addr = grub_cpu_to_le64 (val);
}

void
mmio_set_bits(volatile grub_uint32_t *addr, grub_uint32_t bits)
{
  mmio_write32(addr, mmio_read32(addr) | bits);
}

void
mmio_clear_bits(volatile grub_uint32_t *addr, grub_uint32_t bits)
{
  mmio_write32(addr, mmio_read32(addr) & ~bits);
}

grub_uint32_t
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
grub_uint32_t
mmio_read_bits(const volatile grub_uint32_t *addr, const enum bits32 bits)
{
  grub_uint32_t regval;

  regval = grub_le_to_cpu32 (*addr);
  return parse_reg(regval, bits);
}

/* Return modified copy of regval, shifting and maskin 'val' according to
 * 'bits'.
 */
grub_uint32_t
build_reg(grub_uint32_t regval, const enum bits32 bits, grub_uint32_t val)
{
  const grub_uint32_t bitno = bits >> 16;
  const grub_uint32_t width = bits & 0xff;

  regval &= ~(((1 << width) - 1) << bitno);
  regval |= val << bitno;
  return regval;
}

/**
 * Write a MMIO register. Masking and shifting is done automatically with
 * 'bits'. The register is read first, so existing bits are preserved.
 */
void
mmio_write_bits(volatile grub_uint32_t *addr, const enum bits32 bits, grub_uint32_t val)
{
  grub_uint32_t regval;
  regval = mmio_read32(addr);
  mmio_write32(addr, build_reg(regval, bits, val));
}
