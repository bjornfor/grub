/*
 * IO layer (or more generic: "interface layer") for xHCI driver, custom
 * implementation for GRUB.
 */

#include <grub/types.h> /* grub_uint32_t, grub_cpu_to_le32 */
#include <grub/pci.h> /* grub_memalign_dma32, grub_dma_get_phys */
#include <grub/mm.h> /* grub_zalloc */
#include <grub/misc.h> /* grub_strword */
//#include <grub/term.h> /* grub_refresh */
#include <grub/time.h> /* grub_millisleep */
#include <grub/env.h> /* grub_env_get */

#include "xhci_io.h"

uint32_t le_to_cpu32(uint32_t val)
{
  return grub_le_to_cpu32(val);
}

uint32_t cpu_to_le32(uint32_t val)
{
  return grub_cpu_to_le32(val);
}


uint64_t cpu_to_le64(uint64_t val)
{
  return grub_cpu_to_le64(val);
}

int xhci_debug_enabled(void)
{
  const char *debug = grub_env_get ("debug");

  return debug &&
    (grub_strword (debug, "all") || grub_strword (debug, "xhci"));
}

void xhci_mdelay(unsigned int delay)
{
  grub_millisleep(delay);
}

int xhci_printf (const char *fmt, ...)
{
  va_list ap;
  int ret;

  va_start (ap, fmt);
  ret = grub_vprintf (fmt, ap);
  va_end (ap);

  return ret;
}

int
xhci_snprintf (char *str, size_t n, const char *fmt, ...)
{
  va_list ap;
  int ret;

  va_start (ap, fmt);
  ret = grub_vsnprintf (str, n, fmt, ap);
  va_end (ap);

  return ret;
}

int
xhci_vprintf (const char *fmt, va_list ap)
{
  return grub_vprintf (fmt, ap);
}

void *xhci_calloc(size_t nmemb, size_t size)
{
  return grub_zalloc(nmemb*size);
}

void *xhci_memset(void *s, int c, size_t n)
{
  return grub_memset(s, c, n);
}

void xhci_free(void *ptr)
{
  grub_free(ptr);
}

void *xhci_dma_alloc(size_t align, size_t size)
{
  return grub_memalign_dma32 (align, size);
}

uintptr_t xhci_dma_get_phys(void *ptr)
{
  return grub_dma_get_phys(ptr);
}

//uint32_t pci_config_read (grub_pci_device_t dev, unsigned int reg)
//{
//  grub_pci_address_t addr;
//  addr = grub_pci_make_address (dev, reg);
//  return grub_pci_read (addr);
//}
//
//uint8_t pci_config_read8 (grub_pci_device_t dev, unsigned int reg)
//{
//  grub_pci_address_t addr;
//  addr = grub_pci_make_address (dev, reg);
//  return grub_pci_read_byte (addr);
//}
//
//uint16_t pci_config_read16 (grub_pci_device_t dev, unsigned int reg)
//{
//  grub_pci_address_t addr;
//  addr = grub_pci_make_address (dev, reg);
//  return grub_le_to_cpu16 (grub_pci_read_word (addr) );
//}
//
//uint32_t pci_config_read32 (grub_pci_device_t dev, unsigned int reg)
//{
//  grub_pci_address_t addr;
//  addr = grub_pci_make_address (dev, reg);
//  return grub_le_to_cpu32 (grub_pci_read (addr) );
//}
//
//void pci_config_write32 (grub_pci_device_t dev, unsigned int reg, uint32_t val)
//{
//  grub_pci_address_t addr;
//  addr = grub_pci_make_address (dev, reg);
//  return grub_pci_write (addr, grub_cpu_to_le32 (val));
//}

///* xHCI capability registers access functions */
//uint32_t
//xhci_cap_read32 (struct xhci *xhci, uint32_t off)
//{
//  return grub_le_to_cpu32 (*((volatile uint32_t *) xhci->cap +
//		       (off / sizeof (uint32_t))));
//}

uint8_t
mmio_read8 (const volatile uint8_t *addr)
{
  return *addr;
}

uint16_t
mmio_read16 (const volatile uint16_t *addr)
{
  return grub_le_to_cpu16 (*addr);
}

uint32_t
mmio_read32 (const volatile uint32_t *addr)
{
  return grub_le_to_cpu32 (*addr);
}

uint64_t
mmio_read64 (const volatile uint64_t *addr)
{
  return grub_le_to_cpu64 (*addr);
}

void
mmio_write8 (volatile uint8_t *addr, uint8_t val)
{
  *addr = val;
}

void
mmio_write16 (volatile uint16_t *addr, uint16_t val)
{
  *addr = grub_cpu_to_le16 (val);
}

void
mmio_write32 (volatile uint32_t *addr, uint32_t val)
{
  *addr = grub_cpu_to_le32 (val);
}

void
mmio_write64 (volatile uint64_t *addr, uint64_t val)
{
  *addr = grub_cpu_to_le64 (val);
}
