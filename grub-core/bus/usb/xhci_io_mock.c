#include <stdint.h> /* uint32_t */
#include <stddef.h> /* size_t */
#include <stdarg.h> /* va_list */
#include <unistd.h> /* usleep */
#include <stdio.h> /* printf */
#include <stdlib.h> /* malloc */
#include <string.h> /* memset */

#include "xhci_io.h"

uint32_t le_to_cpu32(uint32_t val)
{
  return val;
}

uint32_t cpu_to_le32(uint32_t val)
{
  return val;
}

uint64_t cpu_to_le64(uint64_t val)
{
  return val;
}

int xhci_debug_enabled(void)
{
  //const char *debug = getenv ("debug");
  //return debug &&
    //(grub_strword (debug, "all") || grub_strword (debug, "xhci"));
  return 1;
}

void xhci_mdelay(unsigned int delay)
{
  usleep(1000UL*delay);
}

int xhci_printf (const char *fmt, ...)
{
  va_list ap;
  int ret;

  va_start (ap, fmt);
  ret = vprintf (fmt, ap);
  va_end (ap);

  return ret;
}

int
xhci_snprintf (char *str, size_t n, const char *fmt, ...)
{
  va_list ap;
  int ret;

  va_start (ap, fmt);
  ret = snprintf (str, n, fmt, ap);
  va_end (ap);

  return ret;
}

int
xhci_vprintf (const char *fmt, va_list ap)
{
  return vprintf (fmt, ap);
}

void *xhci_calloc(size_t nmemb, size_t size)
{
  return calloc(nmemb, size);
}

void *xhci_memset(void *s, int c, size_t n)
{
  return memset(s, c, n);
}

void xhci_free(void *ptr)
{
  free(ptr);
}

void *xhci_dma_alloc(size_t align, size_t size)
{
  return malloc (size);
}

uintptr_t xhci_dma_get_phys(void *ptr)
{
  return (uintptr_t)ptr;
}

uint8_t
mmio_read8 (const volatile uint8_t *addr)
{
  printf("reading 8-bits from %p\n", addr);
  return 0;
}

uint16_t
mmio_read16 (const volatile uint16_t *addr)
{
  printf("reading 16-bits from %p\n", addr);
  return 0;
}

uint32_t
mmio_read32 (const volatile uint32_t *addr)
{
  printf("reading 32-bits from %p\n", addr);
  return 0;
}

uint64_t
mmio_read64 (const volatile uint64_t *addr)
{
  printf("reading 64-bits from %p\n", addr);
  return 0;
}

void
mmio_write8 (volatile uint8_t *addr, uint8_t val)
{
  printf("writing 8-bits to %p: %x\n", addr, val);
}

void
mmio_write16 (volatile uint16_t *addr, uint16_t val)
{
  printf("writing 16-bits to %p: %x\n", addr, val);
}

void
mmio_write32 (volatile uint32_t *addr, uint32_t val)
{
  printf("writing 32-bits to %p: %x\n", addr, val);
}

void
mmio_write64 (volatile uint64_t *addr, uint64_t val)
{
  printf("writing 64-bits to %p: %llx\n", addr, (unsigned long long int)val);
}
