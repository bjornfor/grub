#ifndef XHCI_IO_H
#define XHCI_IO_H

#include <grub/types.h>
#include <grub/pci.h>

#include "xhci_private.h"

grub_uint32_t pci_config_read (grub_pci_device_t dev, unsigned int reg);

grub_uint8_t pci_config_read8 (grub_pci_device_t dev, unsigned int reg);

grub_uint16_t pci_config_read16 (grub_pci_device_t dev, unsigned int reg);

grub_uint32_t pci_config_read32 (grub_pci_device_t dev, unsigned int reg);

void pci_config_write32 (grub_pci_device_t dev, unsigned int reg, grub_uint32_t val);

grub_uint8_t mmio_read8 (const volatile grub_uint8_t *addr);

grub_uint16_t mmio_read16 (const volatile grub_uint16_t *addr);

grub_uint32_t mmio_read32 (const volatile grub_uint32_t *addr);

grub_uint64_t mmio_read64 (const volatile grub_uint64_t *addr);

void mmio_write8 (volatile grub_uint8_t *addr, grub_uint8_t val);

void mmio_write16 (volatile grub_uint16_t *addr, grub_uint16_t val);

void mmio_write32 (volatile grub_uint32_t *addr, grub_uint32_t val);

void mmio_write64 (volatile grub_uint64_t *addr, grub_uint64_t val);

void mmio_set_bits (volatile grub_uint32_t *addr, grub_uint32_t bits);

void mmio_clear_bits (volatile grub_uint32_t *addr, grub_uint32_t bits);

grub_uint32_t mmio_read_bits (const volatile grub_uint32_t *addr,
                              const enum bits32 bits);

void mmio_write_bits (volatile grub_uint32_t *addr,
                      const enum bits32 bits, grub_uint32_t val);

grub_uint32_t parse_reg(grub_uint32_t regval, const enum bits32 bits);

grub_uint32_t build_reg(grub_uint32_t regval, const enum bits32 bits,
    grub_uint32_t val);

grub_uint32_t xhci_read_portrs(struct xhci *xhci, unsigned int port,
    enum xhci_portrs_type type);

grub_size_t
xhci_pagesize_to_bytes(int pagesize);

#endif /* XHCI_IO_H */
